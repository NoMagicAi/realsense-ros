#!/bin/bash
# Entrypoint: launch the ported RealSense driver, wait for it, then run the
# get_latest_frame capture/timing client. Everything is configurable via env vars
# (all optional). Images + acquisition_timing.csv land in ${OUTPUT_DIR} (mount it).
# Note: no `set -u` — ROS setup.bash references unbound vars (AMENT_TRACE_SETUP_FILES).
set -eo pipefail

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

: "${CAMERA_NAMESPACE:=camera}"
: "${CAMERA_NAME:=camera}"
: "${SERIAL_NO:=}"                 # pin a specific camera by serial (recommended on multi-cam stations)
: "${STREAMS:=color aligned_depth_to_color}"
: "${NUM_SHOTS:=10}"
: "${INTERVAL:=0.0}"
: "${LAZY:=true}"                  # nomagic_lazy_filtering
: "${ENABLE_SYNC:=true}"          # REQUIRED for get_latest_frame: gathers depth+color into one frameset
: "${ALIGN_DEPTH:=true}"
: "${ENABLE_DEPTH:=true}"
: "${ENABLE_COLOR:=true}"
: "${DEPTH_PROFILE:=1280x720x15}"  # WxHxFPS; driver default is 1280x720x30 - 15 fps halves USB load
: "${COLOR_PROFILE:=1280x720x15}"  # keep depth/color FPS equal so the syncer can pair framesets
: "${INITIAL_RESET:=false}"        # hard-reset the device before configuring (clears wedged sensors)
: "${OUTPUT_DIR:=/data}"
: "${SKIP_DRIVER:=0}"              # set 1 to only run the client (driver already running elsewhere)
: "${TIMEOUT:=60.0}"              # seconds to wait for services / each service call

mkdir -p "${OUTPUT_DIR}"
DRIVER_PID=""
cleanup() { [[ -n "${DRIVER_PID}" ]] && kill "${DRIVER_PID}" 2>/dev/null || true; }
trap cleanup EXIT

if [[ "${SKIP_DRIVER}" != "1" ]]; then
    echo "[station] launching driver (ns=${CAMERA_NAMESPACE} name=${CAMERA_NAME} lazy=${LAZY} align=${ALIGN_DEPTH})"
    ros2 launch realsense2_camera rs_launch.py \
        camera_namespace:="${CAMERA_NAMESPACE}" \
        camera_name:="${CAMERA_NAME}" \
        serial_no:="${SERIAL_NO}" \
        enable_color:="${ENABLE_COLOR}" \
        enable_depth:="${ENABLE_DEPTH}" \
        enable_sync:="${ENABLE_SYNC}" \
        align_depth.enable:="${ALIGN_DEPTH}" \
        depth_module.depth_profile:="${DEPTH_PROFILE}" \
        rgb_camera.color_profile:="${COLOR_PROFILE}" \
        initial_reset:="${INITIAL_RESET}" \
        pointcloud.enable:=false \
        nomagic_lazy_filtering:="${LAZY}" \
        > "${OUTPUT_DIR}/driver.log" 2>&1 &
    DRIVER_PID=$!
    echo "[station] driver pid=${DRIVER_PID}; log -> ${OUTPUT_DIR}/driver.log"
fi

echo "[station] running capture: streams='${STREAMS}' num=${NUM_SHOTS} -> ${OUTPUT_DIR}"
# shellcheck disable=SC2086
python3 /opt/station/capture_frames.py \
    --streams ${STREAMS} \
    --num "${NUM_SHOTS}" \
    --interval "${INTERVAL}" \
    --timeout "${TIMEOUT}" \
    --output "${OUTPUT_DIR}"

echo "[station] done. Results in ${OUTPUT_DIR}"
