# RealSense D435 layer on eclipse-test-2:humble.
# Build on Jetson:
#   docker build -f docker/jetson_realsense.Dockerfile -t eclipse-test-2:humble-realsense .
#
# aarch64 ros-humble-diagnostic-updater often ships headers only, while
# realsense2_camera links libdiagnostic_updater.so. Inject a prebuilt aarch64 .so.

FROM eclipse-test-2:humble

USER root

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-diagnostic-updater \
        ros-humble-realsense2-camera \
        ros-humble-realsense2-camera-msgs \
        ros-humble-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

# Prebuilt on Jetson (ros2-humble diagnostic_updater). Avoids git/apt build during image build.
COPY docker/libdiagnostic_updater.aarch64.so /opt/ros/humble/lib/libdiagnostic_updater.so
RUN ldconfig \
    && test -f /opt/ros/humble/lib/libdiagnostic_updater.so
