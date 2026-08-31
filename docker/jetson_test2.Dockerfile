FROM eclipse-v2:humble

USER root

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-joy \
        ros-humble-usb-cam \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-foxglove-bridge \
        ros-humble-realsense2-camera \
        ros-humble-diagnostic-updater \
        ros-humble-vision-msgs \
        python3-pyproj \
        python3-shapely \
        python3-pip \
    && pip install pyshp \
    && rm -rf /var/lib/apt/lists/*

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
