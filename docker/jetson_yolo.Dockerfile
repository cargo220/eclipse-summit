# Thin YOLO (ultralytics) on RealSense layer for eclipse-test-2.
# Build on Jetson:
#   docker build -f docker/jetson_realsense.Dockerfile -t eclipse-test-2:humble-realsense .
#   docker build -f docker/jetson_yolo.Dockerfile -t eclipse-test-2:humble-yolo .
# CPU first; GPU: YOLO_DEVICE=0 after JetPack torch is available.

FROM eclipse-test-2:humble-realsense

USER root

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-numpy \
        ros-humble-vision-msgs \
        ros-humble-cv-bridge \
        ros-humble-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --upgrade 'pip<25' \
    && python3 -m pip install --no-cache-dir \
        'setuptools>=65,<80' \
        'packaging>=23,<26' \
        'numpy>=1.23,<2' \
        'opencv-python-headless>=4.8,<4.11' \
        'ultralytics>=8.0,<9'

RUN python3 -c "from ultralytics import YOLO; print('yolo-docker-ok')"
