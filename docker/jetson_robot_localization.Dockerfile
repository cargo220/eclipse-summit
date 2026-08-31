FROM eclipse-v2:humble

USER root

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-usb-cam \
    && rm -rf /var/lib/apt/lists/*
