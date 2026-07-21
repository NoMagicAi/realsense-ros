// Copyright(c) NoMagic. All Rights Reserved.
//
// NoMagic additions to the Intel RealSense ROS2 driver: a per-stream
// `get_latest_frame` service backed by a frameset muxer + history buffer, with
// on-demand (lazy) depth filtering through an isolated filter pair. Declarations
// live in the NOMAGIC block of base_realsense_node.h; see ROS2_PORT_NOTES.md for
// design and threading details.

#include <base_realsense_node.h>
#include <ros_utils.h>
#include <ros_sensor.h>
#include <image_publisher.h>      // full type: image_publisher::get_subscription_count()
#include <pointcloud_filter.h>
#include <align_depth_filter.h>   // full type: AlignDepthFilter (is_enabled)

#include <chrono>
#include <iomanip>
#include <set>
#include <sstream>
#include <thread>

using namespace realsense2_camera;

namespace realsense2_camera
{

namespace
{
    // Image streams the get_latest_frame service is offered for (gated on
    // _image_publishers below).
    const std::vector<stream_index_pair> NOMAGIC_IMAGE_STREAMS = {DEPTH, INFRA0, INFRA1, INFRA2, COLOR};

    // Bound on the service-side wait for a complete frameset. The wait occupies the
    // executor thread, so it must be bounded or an unproductive pipeline wedges
    // every other callback of the node.
    constexpr double NOMAGIC_WAIT_FOR_FRAMESET_TIMEOUT_SECS = 5.0;

    // Build the stream portion of a topic/service name the same way the driver
    // builds its image topics (create_graph_resource_name + optional index suffix).
    std::string nomagicStreamName(const stream_index_pair& stream)
    {
        std::string name = create_graph_resource_name(ros_stream_to_string(stream.first));
        if (stream.second > 0)
            name += std::to_string(stream.second);
        return name;
    }
}

double BaseRealSenseNode::nomagicGetUnixTimestamp()
{
    std::chrono::duration<double> timestamp =
        std::chrono::high_resolution_clock::now().time_since_epoch();
    return timestamp.count();
}

void BaseRealSenseNode::nomagicGetParameters()
{
    // setParam declares (with default), reads and keeps the parameter in sync.
    std::string param_name;

    param_name = std::string("nomagic_lazy_filtering_frame_history_size");
    nomagic_lazy_filtering_frame_history_size =
        _parameters->setParam<int>(param_name, nomagic_lazy_filtering_frame_history_size);
    _parameters_names.push_back(param_name);

    param_name = std::string("nomagic_skip_spatial_filter_for_inner_frames");
    nomagic_skip_spatial_filter_for_inner_frames =
        _parameters->setParam<bool>(param_name, nomagic_skip_spatial_filter_for_inner_frames);
    _parameters_names.push_back(param_name);

    param_name = std::string("nomagic_lazy_filtering");
    nomagic_lazy_filtering =
        _parameters->setParam<bool>(param_name, nomagic_lazy_filtering);
    _parameters_names.push_back(param_name);

    ROS_INFO_STREAM("[NOMAGIC] lazy_filtering=" << nomagic_lazy_filtering
                    << " history_size=" << nomagic_lazy_filtering_frame_history_size
                    << " skip_spatial_for_inner_frames=" << nomagic_skip_spatial_filter_for_inner_frames);
}

void BaseRealSenseNode::nomagicSetupService(stream_index_pair stream, bool is_aligned_depth)
{
    // name + [index]: color, depth, infra1, infra2, etc.
    std::string stream_name = (is_aligned_depth ? std::string("aligned_depth_to_") : std::string("")) + nomagicStreamName(stream);
    std::string full_service_name = stream_name + "/get_latest_frame";

    auto& service_container = is_aligned_depth ? nomagic_get_latest_aligned_frame_servers
                                               : nomagic_get_latest_frame_servers;

    auto callback = [this, stream, is_aligned_depth](
        const realsense2_camera_msgs::srv::GetLatestFrame::Request::SharedPtr request,
        realsense2_camera_msgs::srv::GetLatestFrame::Response::SharedPtr response)
    {
        nomagicGetLatestFrameCallback(stream, is_aligned_depth, request, response);
    };

    // All get_latest_frame services share one MutuallyExclusive callback group so
    // their callbacks are serialized regardless of the executor - the callback
    // mutates shared state (isolated temporal filter, nomagic_images).
    service_container[stream] = _node.create_service<realsense2_camera_msgs::srv::GetLatestFrame>(
        full_service_name, callback, rmw_qos_profile_services_default, nomagic_service_cb_group);

    ROS_INFO_STREAM("[NOMAGIC] Successfully started service " << full_service_name);
}

void BaseRealSenseNode::nomagicSetup()
{
    // set_capacity, NOT resize: resize() would fill the buffer with null framesets,
    // making it look non-empty before any real frame arrived. History size 9 is the
    // minimum for the temporal filter to work reasonably.
    nomagic_frameset_queue.set_capacity(
        nomagic_lazy_filtering ? nomagic_lazy_filtering_frame_history_size : 1);

    nomagic_service_cb_group = _node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Isolated spatial/temporal filters for the on-demand replay, seeded from the
    // driver's primary filters. Separate instances keep the streaming path's
    // temporal-filter state undisturbed by service calls.
    nomagic_spatial_filter = std::make_shared<rs2::spatial_filter>();
    nomagic_temporal_filter = std::make_shared<rs2::temporal_filter>();
    auto copy_options = [](rs2::options& from, rs2::options& to)
    {
        for (auto opt : from.get_supported_options())
        {
            if (!from.supports(opt) || from.is_option_read_only(opt) || !to.supports(opt))
                continue;
            try { to.set_option(opt, from.get_option(opt)); }
            catch (const rs2::error&) { /* best-effort option copy */ }
        }
    };
    for (const auto& named_filter : _filters)
    {
        if (!named_filter || !named_filter->_filter)
            continue;
        if (named_filter->_filter->is<rs2::spatial_filter>())
            copy_options(*named_filter->_filter, *nomagic_spatial_filter);
        else if (named_filter->_filter->is<rs2::temporal_filter>())
            copy_options(*named_filter->_filter, *nomagic_temporal_filter);
    }

    // Frameset-fragmentation diagnostic on the driver's existing updater.
    if (_diagnostics_updater)
    {
        _diagnostics_updater->add("[NOMAGIC] Framesets Fragmentation Status", this,
                                  &BaseRealSenseNode::nomagicFramesetsDiagnosticsCallback);
    }

    // Muxer processing block: defrags framesets (nomagicMuxerCallback) and emits
    // complete composite framesets into the history queue via its output sink.
    // Must exist before the sensors start - the frame callback reads it unlocked.
    nomagic_muxer = std::make_shared<rs2::processing_block>(
        [this](rs2::frame frame, rs2::frame_source& src) { nomagicMuxerCallback(frame, src); });
    nomagic_muxer->start([this](rs2::frame frame) {
        nomagicStoreFramesetForLazyProcessing(frame.as<rs2::frameset>());
    });

    nomagicUpdateStreamsAndServices();

    ROS_INFO_STREAM("[NOMAGIC] Setup complete");
}

void BaseRealSenseNode::nomagicUpdateStreamsAndServices()
{
    // Recompute the expected-stream set and the advertised services from the
    // current publishers, so runtime profile changes do not leave the muxer
    // compositing stale frames or the services out of sync. Runs holding
    // _update_sensor_mutex; the nomagic mutexes protect against the frame thread.
    if (!nomagic_service_cb_group)
        return;  // defensive: not yet initialized by nomagicSetup()

    std::set<stream_index_pair> expected;
    std::set<stream_index_pair> aligned;
    for (const auto& stream : NOMAGIC_IMAGE_STREAMS)
    {
        // An image stream is "enabled" iff the driver created a publisher for it.
        if (_image_publishers.find(stream) == _image_publishers.end())
            continue;
        expected.insert(stream);
        if (_align_depth_filter && _align_depth_filter->is_enabled() &&
            (stream != DEPTH) && (stream.second < 2))
            aligned.insert(stream);
    }

    // Advertise services for newly enabled streams; dropping the shared_ptr of a
    // disabled stream unadvertises its service.
    auto sync_services = [this](auto& container, const std::set<stream_index_pair>& wanted, bool is_aligned)
    {
        for (auto it = container.begin(); it != container.end();)
        {
            if (!wanted.count(it->first))
            {
                ROS_INFO("[NOMAGIC] Removing get_latest_frame service for disabled stream %s_%d",
                         rs2_stream_to_string(it->first.first), it->first.second);
                it = container.erase(it);
            }
            else
                ++it;
        }
        for (const auto& stream : wanted)
        {
            if (!container.count(stream))
                nomagicSetupService(stream, is_aligned);
        }
    };
    sync_services(nomagic_get_latest_frame_servers, expected, false);
    sync_services(nomagic_get_latest_aligned_frame_servers, aligned, true);

    size_t expected_count = expected.size();
    {
        std::lock_guard<std::mutex> lock(nomagic_streams_mutex);
        if (expected == nomagic_expected_streams)
            return;  // stream set unchanged; keep the warm history
        nomagic_expected_streams = std::move(expected);
        // Frames of removed streams must not be composited into future framesets.
        nomagic_latest_frame_buffer.clear();
    }
    {
        // Queued framesets have the old stream composition; refill from scratch.
        std::lock_guard<std::mutex> lock(nomagic_frameset_queue_mutex);
        nomagic_frameset_queue.clear();
    }
    ROS_INFO_STREAM("[NOMAGIC] Stream set updated: " << expected_count << " expected stream(s)");
}

void BaseRealSenseNode::nomagicMuxerCallback(rs2::frame frame, rs2::frame_source& src)
{
    auto frameset = frame.as<rs2::frameset>();
    if (!frameset)
    {
        // Raw sensor frame from the pre-syncer feed: wrap in a 1-frame composite so
        // the defrag logic handles a single shape. keep() first - the frame outlives
        // its librealsense callback once buffered.
        frame.keep();
        frameset = src.allocate_composite_frame({frame}).as<rs2::frameset>();
        if (!frameset)
            return;
    }
    auto missing_streams = nomagicFindMissingStreamsInFrameset(frameset);
    frameset.keep();

    if (frameset.get_frame_timestamp_domain() != RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME)
    {
        ROS_WARN("[NOMAGIC] Frameset timestamp has invalid domain, published "
                 "timestamps and durations may be skewed");
    }

    // Update the latest-frame buffer with the streams that arrived (possibly from an
    // incomplete frameset).
    {
        std::lock_guard<std::mutex> lock(nomagic_streams_mutex);
        for (auto&& f : frameset)
        {
            stream_index_pair stream_index{f.get_profile().stream_type(), f.get_profile().stream_index()};
            nomagic_latest_frame_buffer[stream_index] = frameset;
        }
    }

    // Fast path: the input already contains every expected stream. Store it
    // directly, avoiding the cross-frameset frame-lifetime pitfalls of rebuilding
    // a composite below.
    if (missing_streams.empty())
    {
        {
            std::lock_guard<std::mutex> lock(nomagic_diagnostics_mutex);
            nomagic_received_framesets_last_period += 1;
        }
        nomagicStoreFramesetForLazyProcessing(frameset);
        return;
    }

    // No new depth frame means there is nothing to defrag against; the frame was
    // buffered above and will be picked up by the next depth-triggered assembly.
    if (missing_streams.count(DEPTH))
        return;

    // Defrag path: build a complete frameset from a fresh depth frame + the latest
    // of every other expected stream. Each source frame must be keep()-ed: it
    // outlives its origin frameset once placed in the new composite.
    std::vector<rs2::frame> frames_vec;
    std::set<stream_index_pair> absent_streams;
    size_t expected_count = 0;
    {
        std::lock_guard<std::mutex> lock(nomagic_streams_mutex);
        expected_count = nomagic_expected_streams.size();
        for (auto&& stream_index : nomagic_expected_streams)
        {
            auto buffered = nomagic_latest_frame_buffer.find(stream_index);
            if (buffered == nomagic_latest_frame_buffer.end())
            {
                absent_streams.insert(stream_index); // Given stream did not yet arrive.
                continue;
            }
            for (auto&& f : buffered->second)
            {
                stream_index_pair f_stream_index{f.get_profile().stream_type(), f.get_profile().stream_index()};
                if (stream_index == f_stream_index)
                {
                    f.keep();
                    frames_vec.push_back(f);
                    break;
                }
            }
        }
    }

    if (frames_vec.size() != expected_count)
    {
        {
            std::lock_guard<std::mutex> lock(nomagic_diagnostics_mutex);
            nomagic_received_framesets_last_period += 1;
            nomagic_incomplete_framesets_last_period += 1;
            nomagic_missing_depth_framesets_last_period += absent_streams.count(DEPTH) ? 1 : 0;
            nomagic_missing_color_framesets_last_period += absent_streams.count(COLOR) ? 1 : 0;
        }
        // Expected on startup; if it continues it indicates a problem on an earlier
        // processing stage.
        ROS_WARN("[NOMAGIC] Dropping muxed frame: observed size: %lu, expected size: %lu",
                 (unsigned long)frames_vec.size(), (unsigned long)expected_count);
        return;
    }
    {
        std::lock_guard<std::mutex> lock(nomagic_diagnostics_mutex);
        nomagic_received_framesets_last_period += 1;
    }

    src.frame_ready(src.allocate_composite_frame(frames_vec));
}

// Assumes service callbacks are serialized (nomagic_service_cb_group is
// MutuallyExclusive). Do NOT move to a Reentrant group without adding locking.
void BaseRealSenseNode::nomagicGetLatestFrameCallback(
    stream_index_pair stream, bool is_aligned_depth,
    const realsense2_camera_msgs::srv::GetLatestFrame::Request::SharedPtr /*request*/,
    realsense2_camera_msgs::srv::GetLatestFrame::Response::SharedPtr response)
{
    response->request_timestamp = nomagicGetUnixTimestamp();
    ROS_INFO("[NOMAGIC] get_latest_frame: stream=%s_%d aligned_depth=%d",
             rs2_stream_to_string(stream.first), stream.second, (int)is_aligned_depth);
    Clock clock;

    try
    {
        clock.restart();
        auto queue = nomagicGetNonEmptyFramesetQueue();
        response->wait_for_frames_duration = clock.getElapsedSecs();
        ROS_INFO("[NOMAGIC] Queue contains %lu frameset(s)", (unsigned long)queue.size());

        rs2::frame final_frame;
        // RGB / infra frames need no processing when aligned depth is not requested.
        if (!is_aligned_depth && stream != DEPTH)
        {
            rs2::frameset frameset = queue.back();
            final_frame = nomagicFramesetToFrame(stream, frameset);
        }
        else
        {
            if (nomagic_lazy_filtering)
            {
                // Clear stale temporal state carried over from a previous request.
                clock.restart();
                nomagicResetTemporalFilter();
                response->reset_temporal_filter_duration = clock.getElapsedSecs();
            }

            clock.restart();
            rs2::frameset frameset = nomagicApplyFilters(std::move(queue));
            response->filtering_duration = clock.getElapsedSecs();

            if (is_aligned_depth)
            {
                clock.restart();
                final_frame = nomagicGetDepthAlignedTo(stream, frameset);
                response->depth_alignment_duration = clock.getElapsedSecs();
            }
            else
            {
                final_frame = nomagicFramesetToFrame(stream, frameset);
            }
        }

        response->image = nomagicFrameToMessage(is_aligned_depth ? DEPTH : stream, final_frame);
        response->frame_timestamp = final_frame.get_timestamp() / 1000.0;
        response->response_timestamp = nomagicGetUnixTimestamp();
        response->success = true;
    }
    catch (const std::exception& e)
    {
        // A ROS2 service callback cannot fail the call itself; the error is carried
        // in the response (clients must check `success`).
        ROS_ERROR("[NOMAGIC] get_latest_frame failed: %s", e.what());
        response->response_timestamp = nomagicGetUnixTimestamp();
        response->success = false;
        response->error_message = e.what();
    }
}

rs2::frameset BaseRealSenseNode::nomagicApplyFilters(boost::circular_buffer<rs2::frameset>&& queue)
{
    rs2::frameset frameset;
    int frames_processed = 0;
    // Key the first/last check off the element count, not the capacity: a
    // partially-filled history must not classify the final frame as "inner".
    const int last_index = static_cast<int>(queue.size()) - 1;
    while (!queue.empty())
    {
        frameset = queue.front();
        queue.pop_front();

        // Spatial filter is a per-frame operation; skip it for inner frames of the
        // history when configured (they exist only to warm up the temporal filter).
        bool is_inner_frame = !(frames_processed == 0 || frames_processed == last_index);
        bool skip_spatial = nomagic_skip_spatial_filter_for_inner_frames && is_inner_frame;

        // Capture apply_filter's return so the returned frameset carries the
        // filtered depth.
        if (nomagic_spatial_filter && !skip_spatial)
            frameset = frameset.apply_filter(*nomagic_spatial_filter);
        if (nomagic_temporal_filter)
            frameset = frameset.apply_filter(*nomagic_temporal_filter);

        frames_processed += 1;
    }
    ROS_INFO("[NOMAGIC] Filtered %d most recent frame(s)", frames_processed);
    return frameset;
}

void BaseRealSenseNode::nomagicResetTemporalFilter()
{
    // The temporal filter resets its internal state whenever any of its options is
    // (re)written, even to the same value.
    if (nomagic_temporal_filter)
    {
        auto value = nomagic_temporal_filter->get_option(RS2_OPTION_FILTER_SMOOTH_ALPHA);
        nomagic_temporal_filter->set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, value);
    }
}

rs2::frame BaseRealSenseNode::nomagicGetDepthAlignedTo(stream_index_pair stream, rs2::frameset frameset)
{
    ROS_INFO("[NOMAGIC] Computing depth aligned to %s%d",
             rs2_stream_to_string(stream.first), stream.second);

    // Aligning depth to depth is meaningless; index > 1 streams are skipped for
    // parity with the streaming path.
    if (RS2_STREAM_DEPTH == stream.first || stream.second > 1)
    {
        ROS_WARN("[NOMAGIC] Unexpected align-to-depth request; returning unaligned frame");
        return nomagicFramesetToFrame(stream, frameset);
    }

    rs2::align align(stream.first);
    rs2::frame aligned_depth = frameset.apply_filter(align).as<rs2::frameset>().get_depth_frame();

    // Colorize only if the driver's colorizer filter is enabled (matches the
    // streaming path; disabled by default, so raw 16UC1 depth is returned).
    if (_colorizer_filter && _colorizer_filter->is_enabled())
    {
        ROS_INFO("[NOMAGIC] Applying colorizer to aligned depth");
        return _colorizer_filter->Process(aligned_depth);
    }
    return aligned_depth;
}

sensor_msgs::msg::Image BaseRealSenseNode::nomagicFrameToMessage(stream_index_pair stream, rs2::frame frame)
{
    // Reuses the driver's image-construction helpers (depth-scale fixup, rs2->ROS
    // encoding mapping); stamped with the node clock.
    sensor_msgs::msg::Image img;
    if (!frame.is<rs2::video_frame>())
    {
        ROS_ERROR("[NOMAGIC] Requested frame is not a video frame");
        return img;
    }
    auto vframe = frame.as<rs2::video_frame>();
    unsigned int width = vframe.get_width();
    unsigned int height = vframe.get_height();
    auto stream_format = vframe.get_profile().format();
    rclcpp::Time t = _node.now();

    // nomagic_images, not the driver's _images: the frame thread writes _images
    // concurrently, so sharing the map would tear the Mats.
    if (fillCVMatImageAndReturnStatus(frame, nomagic_images, width, height, stream))
    {
        fillROSImageMsgAndReturnStatus(nomagic_images[stream], stream, width, height, stream_format, t, &img);
    }
    return img;
}

bool BaseRealSenseNode::nomagicAnyDepthHasSubscribers(const rs2::frameset& frameset)
{
    for (auto&& f : frameset)
    {
        stream_index_pair stream{f.get_profile().stream_type(), f.get_profile().stream_index()};
        bool has_subscribers = false;
        if (stream == DEPTH)
        {
            if (_info_publishers.count(stream) && _info_publishers.at(stream)->get_subscription_count() != 0)
                has_subscribers = true;
            if (_image_publishers.count(stream) && _image_publishers.at(stream)->get_subscription_count() != 0)
                has_subscribers = true;
        }
        // second < 2 matches where the driver creates aligned-depth publishers.
        else if (_align_depth_filter && _align_depth_filter->is_enabled() && stream.second < 2)
        {
            if (_depth_aligned_info_publisher.count(stream) && _depth_aligned_info_publisher.at(stream)->get_subscription_count() != 0)
                has_subscribers = true;
            if (_depth_aligned_image_publishers.count(stream) && _depth_aligned_image_publishers.at(stream)->get_subscription_count() != 0)
                has_subscribers = true;
        }
        if (has_subscribers)
            return true;
    }
    return false;
}

std::set<stream_index_pair> BaseRealSenseNode::nomagicFindMissingStreamsInFrameset(const rs2::frameset& frameset)
{
    std::set<stream_index_pair> required;
    {
        std::lock_guard<std::mutex> lock(nomagic_streams_mutex);
        required = nomagic_expected_streams;
    }
    for (auto frame : frameset)
    {
        required.erase(stream_index_pair{frame.get_profile().stream_type(), frame.get_profile().stream_index()});
    }
    return required;
}

void BaseRealSenseNode::nomagicStoreFramesetForLazyProcessing(rs2::frameset frameset)
{
    auto missing = nomagicFindMissingStreamsInFrameset(frameset);
    if (!missing.empty())
    {
        std::stringstream missingStr;
        for (auto&& stream : missing)
            missingStr << rs2_stream_to_string(stream.first) << stream.second << ", ";
        // Natural on startup; if seen later it indicates a bug in nomagicMuxerCallback.
        ROS_WARN("[NOMAGIC] Received an incomplete frameset, missing: %s", missingStr.str().c_str());
        return;
    }

    // Tick the aligned-depth frequency diagnostic - every complete frameset passes
    // through here, so it reflects the true rate at which aligned-depth can be
    // served. Created lazily; only the librealsense frame thread touches
    // nomagic_aligned_depth_freq.
    for (auto&& entry : _depth_aligned_image_publishers)
    {
        auto freq = nomagic_aligned_depth_freq.find(entry.first);
        if (freq == nomagic_aligned_depth_freq.end() && _diagnostics_updater)
        {
            int fps = 0;
            auto depth = frameset.get_depth_frame();
            if (depth) fps = depth.get_profile().fps();
            auto inserted = nomagic_aligned_depth_freq.emplace(
                entry.first,
                std::make_shared<FrequencyDiagnostics>(
                    "[NOMAGIC] " + nomagicStreamName(entry.first) + "_aligned_depth_frequency",
                    fps, _diagnostics_updater));
            freq = inserted.first;
        }
        if (freq != nomagic_aligned_depth_freq.end())
            freq->second->Tick();
    }

    std::lock_guard<std::mutex> lock(nomagic_frameset_queue_mutex);
    frameset.keep();
    nomagic_frameset_queue.push_back(frameset);
}

boost::circular_buffer<rs2::frameset> BaseRealSenseNode::nomagicGetNonEmptyFramesetQueue()
{
    Clock wait_clock;
    int times_waited = 0;
    while (true)
    {
        {
            std::lock_guard<std::mutex> lock(nomagic_frameset_queue_mutex);
            if (!nomagic_frameset_queue.empty())
                return nomagic_frameset_queue;
        }
        // The wait occupies the executor thread, so it must be bounded; the service
        // callback turns the throw into a success=false response.
        if (wait_clock.getElapsedSecs() > NOMAGIC_WAIT_FOR_FRAMESET_TIMEOUT_SECS)
            throw std::runtime_error(
                "[NOMAGIC] Timed out waiting for a complete frameset "
                "(is the camera streaming all expected streams?)");
        // The producer (muxer) runs on librealsense's own threads, so this wait
        // cannot starve frame production.
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        times_waited += 1;
        if (times_waited % 1000 == 0)
            ROS_WARN("[NOMAGIC] Still waiting for a non-empty frameset queue (%d ms)", times_waited);
    }
}

rs2::frame BaseRealSenseNode::nomagicFramesetToFrame(stream_index_pair stream, rs2::frameset frameset)
{
    for (auto frame : frameset)
    {
        stream_index_pair sip{frame.get_profile().stream_type(), frame.get_profile().stream_index()};
        if (stream == sip)
            return frame;
    }
    throw std::runtime_error("[NOMAGIC] Stream frame not found in the frameset");
}

std::string BaseRealSenseNode::nomagicFramesetDescriptionString(const rs2::frameset& frameset)
{
    std::stringstream str;
    str << "(";
    for (auto&& frame : frameset)
    {
        str << rs2_stream_to_string(frame.get_profile().stream_type()) << "_"
            << frame.get_profile().stream_index() << " ok=" << static_cast<bool>(frame) << ", ";
    }
    str << std::setprecision(20) << (frameset.get_timestamp() / 1000.0) << " "
        << rs2_timestamp_domain_to_string(frameset.get_frame_timestamp_domain()) << ")";
    return str.str();
}

void BaseRealSenseNode::nomagicFramesetsDiagnosticsCallback(diagnostic_updater::DiagnosticStatusWrapper& status)
{
    static Clock period_clock;
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                   "Statistics of framesets received from the camera");
    std::lock_guard<std::mutex> lock(nomagic_diagnostics_mutex);
    status.add("period", period_clock.getElapsedSecs());

    status.add("received_framesets", nomagic_received_framesets_last_period);
    nomagic_received_framesets_last_period = 0;

    status.add("incomplete_framesets", nomagic_incomplete_framesets_last_period);
    nomagic_incomplete_framesets_last_period = 0;

    status.add("missing_color_framesets", nomagic_missing_color_framesets_last_period);
    nomagic_missing_color_framesets_last_period = 0;

    status.add("missing_depth_framesets", nomagic_missing_depth_framesets_last_period);
    nomagic_missing_depth_framesets_last_period = 0;

    period_clock.restart();
}

}  // namespace realsense2_camera
