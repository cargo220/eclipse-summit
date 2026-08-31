# JetPack-matched YOLO+TensorRT runtime for eclipse-test-2.
# Base: ROS Humble (dustynv) + PyTorch CUDA + TensorRT 8.5.
# Do NOT apt from packages.ros.org (expired GPG on this base).
# Do NOT let pip upgrade torch/torchvision (breaks JetPack CUDA match).
# Do NOT import tensorrt during docker build (needs host tegra libs / nvidia runtime).
#
# Build on Jetson:
#   docker build -f docker/jetson_yolo_trt.Dockerfile -t eclipse-test-2:humble-yolo-trt .
#
# numpy 1.23.x is required for Ultralytics TRT offline path (np.bool); pin at image
# build so smoke/live do not runtime-pip on Jetson (slow/OOM/hang).

FROM dustynv/ros:humble-desktop-pytorch-l4t-r35.4.1

USER root

# Ultralytics only; keep base torch (2.0+nv).
# dustynv base has torch but no torchvision; pin 0.15.1 to match torch 2.0 without upgrade.
# Do NOT install opencv-python(-headless); base OpenCV already provides cv2.
RUN python3 -m pip install --no-cache-dir --upgrade 'pip<25' \
    && python3 -m pip install --no-cache-dir \
        'setuptools>=65,<80' \
        'packaging>=23,<26' \
    && python3 -m pip install --no-cache-dir --no-deps 'torchvision==0.15.1' \
    && python3 -m pip install --no-cache-dir --no-deps 'ultralytics>=8.0,<9' \
    && python3 -m pip install --no-cache-dir \
        'matplotlib>=3.3,<3.8' \
        'pillow>=9.0,<11' \
        'pyyaml>=5.3' \
        'requests>=2.23' \
        'scipy>=1.4' \
        'tqdm>=4.64' \
        'seaborn>=0.11' \
        'pandas>=1.1' \
        'psutil>=5.8' \
        'py-cpuinfo' \
        'ultralytics-thop>=2.0' \
        'nvidia-ml-py>=12.0.0' \
        'polars>=0.20.0' \
    && python3 -m pip install --no-cache-dir --force-reinstall --no-deps 'numpy==1.23.5'

# Build-time check: ultralytics + torch only (tensorrt needs nvidia runtime at run).
RUN python3 -c "import numpy as np; import torch, torchvision; from ultralytics import YOLO; \
assert np.__version__.startswith('1.23.'), np.__version__; \
print('yolo-trt-docker-ok', 'numpy', np.__version__, torch.__version__, torchvision.__version__, 'cuda_build', torch.cuda.is_available())"
