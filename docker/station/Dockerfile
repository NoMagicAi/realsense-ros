# Station test image: builds the ported RealSense driver (realsense2_camera[_msgs])
# and ships a get_latest_frame capture/timing client. Run it on a station with a
# RealSense camera attached; it writes images + timing CSV to a mounted volume.
#
# Build (context = repo root):
#   docker build -f docker/station/Dockerfile -t nomagic-realsense-station .
# Run (see docker/station/README.md):
#   docker run --rm --privileged -v /dev:/dev -v "$PWD/out:/data" nomagic-realsense-station
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        libboost-dev \
        python3-opencv \
        python3-numpy && \
    rm -rf /var/lib/apt/lists/*

# Build the two ported packages. rosdep pulls ros-humble-librealsense2 (>= 2.58)
# and ros-humble-cv-bridge (provides the python cv_bridge used by the client).
WORKDIR /ws
COPY realsense2_camera_msgs /ws/src/realsense-ros/realsense2_camera_msgs
COPY realsense2_camera     /ws/src/realsense-ros/realsense2_camera
RUN apt-get update && rosdep update --rosdistro humble && \
    rosdep install -y --rosdistro humble \
        --from-paths src/realsense-ros/realsense2_camera_msgs src/realsense-ros/realsense2_camera \
        --ignore-src && \
    rm -rf /var/lib/apt/lists/*
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select realsense2_camera_msgs realsense2_camera \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/station/capture_frames.py /opt/station/capture_frames.py
COPY docker/station/station_capture.sh /opt/station/station_capture.sh
RUN chmod +x /opt/station/station_capture.sh /opt/station/capture_frames.py

ENTRYPOINT ["/opt/station/station_capture.sh"]
