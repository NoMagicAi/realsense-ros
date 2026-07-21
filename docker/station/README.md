# Station capture / acquisition-timing test

A self-contained image that runs the **ported ROS2 RealSense driver** and calls the
NoMagic `get_latest_frame` service on a station with a real RealSense camera. It saves
each returned image to a mounted volume and records acquisition timing (round-trip and
the server-side duration fields) to `acquisition_timing.csv`.

## Build (context = repo root `realsense-ros/`)

```bash
docker build -f docker/station/Dockerfile -t nomagic-realsense-station .
```

## Run

The container needs USB access to the camera and a mounted output directory:

```bash
mkdir -p out
docker run --rm \
    --privileged \
    -v /dev:/dev \
    -v "$PWD/out:/data" \
    nomagic-realsense-station
```

Results land in `./out/`: `color_000.png …`, `aligned_depth_to_color_000.png …`
(depth also written as `.npy`), plus `acquisition_timing.csv` and `driver.log`.

If `--privileged -v /dev:/dev` is undesirable, pass the specific device cgroup instead
(e.g. `--device-cgroup-rule='c 81:* rmw' --device-cgroup-rule='c 189:* rmw' -v /dev/bus/usb:/dev/bus/usb`).

## Configuration (env vars, all optional)

| Var | Default | Meaning |
|---|---|---|
| `STREAMS` | `color aligned_depth_to_color` | stream keys to capture (`color`, `depth`, `aligned_depth_to_color`, `infra1`, …) |
| `NUM_SHOTS` | `10` | shots per stream |
| `INTERVAL` | `0.0` | seconds between shots |
| `LAZY` | `true` | `nomagic_lazy_filtering` (on-demand filtering path) |
| `ENABLE_SYNC` | `true` | **required** — gathers depth+color into one frameset. With sync off, `get_latest_frame` never sees a complete frameset and every call returns `success=false` after the driver-side 5 s wait bound. |
| `ALIGN_DEPTH` | `true` | enable aligned-depth-to-color (needed for the `aligned_depth_to_color` service) |
| `ENABLE_COLOR` / `ENABLE_DEPTH` | `true` | streams the driver enables |
| `DEPTH_PROFILE` / `COLOR_PROFILE` | `1280x720x15` | stream profile `WxHxFPS` (driver default is `1280x720x30`; 15 fps halves USB load — keep both FPS equal so the syncer can pair framesets) |
| `INITIAL_RESET` | `false` | hard-reset the device before configuring (clears a sensor wedged by a killed session) |
| `TIMEOUT` | `60.0` | seconds the client waits for services / each call |
| `SERIAL_NO` | *(empty)* | pin a specific camera by serial (recommended on multi-camera stations) |
| `CAMERA_NAMESPACE` / `CAMERA_NAME` | `camera` | driver namespace / name |
| `OUTPUT_DIR` | `/data` | output dir inside the container (mount it) |
| `SKIP_DRIVER` | `0` | set `1` to only run the client against an already-running driver |

Example — 30 lazy-filtered depth-aligned shots from a specific camera:

```bash
docker run --rm --privileged -v /dev:/dev -v "$PWD/out:/data" \
    -e SERIAL_NO=123456789012 \
    -e STREAMS="color aligned_depth_to_color depth" \
    -e NUM_SHOTS=30 -e LAZY=true \
    nomagic-realsense-station
```

## What to check

- **Images** in `out/` look correct (color sharp, aligned depth registered to color).
- **`acquisition_timing.csv`** — per shot: `roundtrip_ms`, `server_total_ms`, and the
  breakdown (`wait_for_frames_ms`, `reset_temporal_filter_ms`, `filtering_ms`,
  `depth_alignment_ms`). With `LAZY=true`, `filtering_ms` / `depth_alignment_ms` are
  non-zero (work happens on demand); with `LAZY=false` the streaming path already
  filtered, so those are ~0 and depth is returned pre-filtered.
- **`driver.log`** — the driver reached the camera and advertised the services
  (`[NOMAGIC] Successfully started service …`).

## Notes

- This harness is a NoMagic addition for validating the ROS1→ROS2 port; it is not part
  of upstream Intel's driver. The production runtime image is the `realsense-ros-node`
  package (migration Part B).
- Full behavioural verification requires the physical camera; without one the driver
  logs `No RealSense devices were found!` and the client times out waiting for services.
