# NoMagic ROS2 port notes (`get_latest_frame` + lazy filtering)

This fork re-implements the NoMagic proprietary additions on top of Intel's ROS2 driver
(`ros2-master`, pinned tag **4.58.3**, librealsense **≥ 2.58.0**). The ROS1 originals lived in
`base_realsense_node.cpp/.h` on branch `nomagic-development` (Intel base 2.3.2). Because the 4.x
`BaseRealSenseNode` is heavily restructured, the block was **re-implemented**, not cherry-picked.

## Where the code lives
- `srv/GetLatestFrame.srv` → moved to **`realsense2_camera_msgs/srv/GetLatestFrame.srv`**. The ROS1
  fields are kept first and unchanged; ROS2 appends `bool success` + `string error_message` to the
  response, because a ROS2 service callback cannot fail the call the way ROS1's `return false` did.
  **Clients must check `success`.**
- New translation unit **`src/nomagic_realsense_node.cpp`** + helper header **`include/nomagic_realsense_node.h`** (`Clock`).
- Member/method declarations live in the `NOMAGIC` block of `include/base_realsense_node.h`.
- Thin hooks: `frame_callback()` (`base_realsense_node.cpp`), `getParameters()` (`parameters.cpp`),
  `setup()` (`rs_node_setup.cpp`), and a subscriber accessor on `PointcloudFilter`.

## ROS1 → ROS2 mapping (notable differences)
| ROS1 (2.3.2) | ROS2 (4.58.3) |
|---|---|
| `NamedFilter._name` string dispatch | `NamedFilter::_filter->is<rs2::spatial_filter/temporal_filter/…>()` |
| second `setupFilters(nomagic_filters)` | bare `rs2::spatial_filter`/`rs2::temporal_filter`, options copied from `_filters` via `get_supported_options()` |
| `ros::ServiceServer` + `boost::function` | `_node.create_service<…>()` on a **MutuallyExclusive** callback group |
| `_pnh.param(...)` | `_parameters->setParam<T>(name, default)` (native backend) |
| `_pointcloud_publisher.getNumSubscribers()` | `PointcloudFilter::getNumSubscribers()` (new accessor) |
| `getNumSubscribers()` | `get_subscription_count()` |
| `_enable[]` / `IMAGE_STREAMS` / `STREAM_NAME` | `_image_publishers` membership + `create_graph_resource_name(ros_stream_to_string(...))` |
| muxer as ctor member | `std::shared_ptr<rs2::processing_block>` built in `nomagicSetup()` |
| separate `diagnostic_updater::Updater` | reuse the driver's `_diagnostics_updater` |
| `ros::Time::now()` / `ROS_*` | `_node.now()` / `ROS_*` macros (RCLCPP wrappers, `constants.h`) |

## Dropped / obsolete vs the ROS1 fork
- **D405 `RS405_PID = 0x0B5B`** — already upstream in 4.x. Not re-added.
- **HDR ordering fix (FIONA-0)** — obsolete: 4.x has no `registerHDRoptions()` (`hdr_merge` is a
  normal filter). Not re-added.

## Threading
`get_latest_frame` service callbacks wait (bounded, 5 s, then a `success=false` response) for a
non-empty history queue and mutate shared temporal-filter state. All such services share **one
MutuallyExclusive callback group** so they are serialized regardless of the component container's
executor (reproducing the ROS1 nodelet guarantee). The producer (muxer) runs on librealsense's own
callback threads, so the wait never starves frame production — but it does occupy the executor, hence
the bound. Mutexes: `nomagic_frameset_queue_mutex` (history queue), `nomagic_streams_mutex`
(expected-stream set + latest-frame buffer, which the monitoring thread rewrites via
`nomagicUpdateStreamsAndServices()` on runtime profile changes), `nomagic_diagnostics_mutex`
(fragmentation counters). Never hold `nomagic_streams_mutex` while taking the queue mutex. The
service thread uses its own `nomagic_images` CV buffers — the driver's `_images` belongs to the
frame thread. **Do not** move these services to a Reentrant group without adding locking around the
isolated filter set.

## Verify on a live camera (not compilable without the SDK)
- `ros2 service list | grep get_latest_frame` → `<stream>/get_latest_frame` + `aligned_depth_to_*`.
- Call the service; timing fields (`*_duration`) are populated; with `nomagic_lazy_filtering:=true`,
  depth filtering runs on-demand only (subscriber-gated on the streaming path).
- `ros2 topic echo --once /<cam>/diagnostics` shows `[NOMAGIC] Framesets Fragmentation Status`.
- Confirm the generated `GetLatestFrame` field order matches the ROS1 `.srv` for the shared fields
  (consumer contract); `success`/`error_message` are ROS2-only additions at the end of the response.
- With `enable_sync:=false`, a call must return `success=false` after ~5 s (bounded wait), not hang.

## One API to watch at first compile
`rs2::options::get_supported_options()` (used in `nomagicSetup()` to copy filter options) requires
librealsense ≥ 2.53; we pin 2.58, so it is available. Everything else mirrors APIs the ROS1 fork
already used (`processing_block`, `invoke`, `frame_source::allocate_composite_frame`/`frame_ready`,
`frameset::apply_filter`).
