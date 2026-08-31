"""Thin Ultralytics YOLO 2D detector for RealSense color images.

Subscribes to a color Image (Best Effort), runs YOLO predict, publishes
vision_msgs/Detection2DArray and optional annotated debug Image.

License note: Ultralytics is AGPL; this node does not depend on GPL yolo_ros.
"""

from __future__ import annotations

import os
import threading
import time
from typing import Dict, List, Optional

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)

from eclipse_pkg.yolo_class_filter import (
    class_name_allowed,
    debug_box_bgr,
    debug_box_label,
    names_from_override,
    parse_class_csv,
    parse_class_thresholds,
    parse_ordered_class_csv,
    predict_conf_floor,
    resolve_predict_class_ids,
    score_meets_threshold,
    unmatched_allowed_names,
)

# yolov8n COCO-80 names — TensorRT .engine often has no class metadata.
_COCO80_NAMES: Dict[int, str] = {
    0: 'person',
    1: 'bicycle',
    2: 'car',
    3: 'motorcycle',
    4: 'airplane',
    5: 'bus',
    6: 'train',
    7: 'truck',
    8: 'boat',
    9: 'traffic light',
    10: 'fire hydrant',
    11: 'stop sign',
    12: 'parking meter',
    13: 'bench',
    14: 'bird',
    15: 'cat',
    16: 'dog',
    17: 'horse',
    18: 'sheep',
    19: 'cow',
    20: 'elephant',
    21: 'bear',
    22: 'zebra',
    23: 'giraffe',
    24: 'backpack',
    25: 'umbrella',
    26: 'handbag',
    27: 'tie',
    28: 'suitcase',
    29: 'frisbee',
    30: 'skis',
    31: 'snowboard',
    32: 'sports ball',
    33: 'kite',
    34: 'baseball bat',
    35: 'baseball glove',
    36: 'skateboard',
    37: 'surfboard',
    38: 'tennis racket',
    39: 'bottle',
    40: 'wine glass',
    41: 'cup',
    42: 'fork',
    43: 'knife',
    44: 'spoon',
    45: 'bowl',
    46: 'banana',
    47: 'apple',
    48: 'sandwich',
    49: 'orange',
    50: 'broccoli',
    51: 'carrot',
    52: 'hot dog',
    53: 'pizza',
    54: 'donut',
    55: 'cake',
    56: 'chair',
    57: 'couch',
    58: 'potted plant',
    59: 'bed',
    60: 'dining table',
    61: 'toilet',
    62: 'tv',
    63: 'laptop',
    64: 'mouse',
    65: 'remote',
    66: 'keyboard',
    67: 'cell phone',
    68: 'microwave',
    69: 'oven',
    70: 'toaster',
    71: 'sink',
    72: 'refrigerator',
    73: 'book',
    74: 'clock',
    75: 'vase',
    76: 'scissors',
    77: 'teddy bear',
    78: 'hair drier',
    79: 'toothbrush',
}


def _torch_nms_cpu(boxes, scores, iou_threshold):
    """Pure-torch NMS (CPU). Avoids broken torchvision CUDA/C++ ops on Jetson."""
    import torch

    if boxes is None or boxes.numel() == 0:
        return torch.empty((0,), dtype=torch.int64)

    boxes = boxes.detach().float().cpu()
    scores = scores.detach().float().cpu()
    x1, y1, x2, y2 = boxes.unbind(dim=1)
    areas = (x2 - x1).clamp(min=0) * (y2 - y1).clamp(min=0)
    order = scores.argsort(descending=True)
    keep = []
    while order.numel() > 0:
        i = int(order[0].item())
        keep.append(i)
        if order.numel() == 1:
            break
        rest = order[1:]
        xx1 = torch.maximum(x1[i], x1[rest])
        yy1 = torch.maximum(y1[i], y1[rest])
        xx2 = torch.minimum(x2[i], x2[rest])
        yy2 = torch.minimum(y2[i], y2[rest])
        w = (xx2 - xx1).clamp(min=0)
        h = (yy2 - yy1).clamp(min=0)
        inter = w * h
        iou = inter / (areas[i] + areas[rest] - inter + 1e-6)
        order = rest[iou <= float(iou_threshold)]
    return torch.tensor(keep, dtype=torch.int64)


def _patch_torchvision_nms_cpu() -> None:
    """Replace torchvision NMS with pure-torch CPU NMS.

    Jetson TensorRT path often hits either:
    - torchvision CUDA NMS 'no kernel image', or
    - torchvision C++ ops load failure after numpy/torch version skew.
    """
    import torchvision

    def _batched_nms_cpu(boxes, scores, idxs, iou_threshold):
        import torch

        if boxes.numel() == 0:
            return torch.empty((0,), dtype=torch.int64)
        # Standard class-aware batched NMS via coordinate offset.
        boxes = boxes.detach().float().cpu()
        scores = scores.detach().float().cpu()
        idxs = idxs.detach().to(dtype=torch.int64).cpu()
        max_coord = boxes.max()
        offsets = idxs.to(boxes.dtype) * (max_coord + 1.0)
        boxes_for_nms = boxes + offsets[:, None]
        return _torch_nms_cpu(boxes_for_nms, scores, iou_threshold)

    torchvision.ops.nms = _torch_nms_cpu
    if hasattr(torchvision.ops, 'batched_nms'):
        torchvision.ops.batched_nms = _batched_nms_cpu
    if hasattr(torchvision.ops, 'boxes'):
        if hasattr(torchvision.ops.boxes, 'nms'):
            torchvision.ops.boxes.nms = _torch_nms_cpu
        if hasattr(torchvision.ops.boxes, 'batched_nms'):
            torchvision.ops.boxes.batched_nms = _batched_nms_cpu

    # Ultralytics may hold a direct imported reference.
    try:
        import ultralytics.utils.ops as uops

        if hasattr(uops, 'nms'):
            uops.nms = _torch_nms_cpu
        if hasattr(uops, 'torchvision'):
            uops.torchvision.ops.nms = _torch_nms_cpu
    except Exception:
        pass


def _names_look_placeholder(names: Dict[int, str]) -> bool:
    """True for TensorRT placeholders like class0/class1 (not real COCO labels)."""
    if not names:
        return True
    sample = [str(names[k]) for k in sorted(names)[:8]]
    if not sample:
        return True
    # All generic classN → unusable for allowed_classes filters like "person".
    return all(
        s.lower().startswith('class') and s[5:].isdigit()
        for s in sample
    )


def _safe_class_names(model) -> tuple[Dict[int, str], bool]:
    """Return (class-id → name, used_coco_fallback).

    TensorRT .engine loads often leave model.names unusable because the
    underlying model handle is still a path string, or names are class0..N.
    """
    names = None
    try:
        names = model.names
    except Exception:  # noqa: BLE001 — engine path may lack names attr
        names = None

    if isinstance(names, dict) and names:
        try:
            parsed = {int(k): str(v) for k, v in names.items()}
            if not _names_look_placeholder(parsed):
                return parsed, False
        except (TypeError, ValueError):
            pass
    if isinstance(names, (list, tuple)) and names:
        parsed = {i: str(n) for i, n in enumerate(names)}
        if not _names_look_placeholder(parsed):
            return parsed, False

    # Ultralytics engine wrappers sometimes leave model.model as a path string.
    inner = getattr(model, 'model', None)
    if inner is not None and not isinstance(inner, str):
        try:
            inner_names = getattr(inner, 'names', None)
            if isinstance(inner_names, dict) and inner_names:
                parsed = {int(k): str(v) for k, v in inner_names.items()}
                if not _names_look_placeholder(parsed):
                    return parsed, False
            if isinstance(inner_names, (list, tuple)) and inner_names:
                parsed = {i: str(n) for i, n in enumerate(inner_names)}
                if not _names_look_placeholder(parsed):
                    return parsed, False
        except Exception:  # noqa: BLE001
            pass

    return dict(_COCO80_NAMES), True


def _best_effort_image_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class _H264UdpSink:
    """Push annotated BGR frames as H.264 RTP/UDP via OpenCV+GStreamer."""

    def __init__(
        self,
        host: str,
        port: int,
        bitrate: int,
        fps: int,
        logger,
    ) -> None:
        self._host = host
        self._port = port
        self._bitrate = bitrate
        self._fps = fps
        self._logger = logger
        self._writer = None
        self._size: Optional[tuple] = None
        self._failed = False
        self._backend = ''

    def write(self, frame_bgr) -> None:
        if self._failed or frame_bgr is None:
            return
        import cv2

        h, w = frame_bgr.shape[:2]
        if self._writer is None or self._size != (w, h):
            self.close()
            if not self._open(w, h):
                self._failed = True
                return
        assert self._writer is not None
        self._writer.write(frame_bgr)

    def _pipelines(self, w: int, h: int) -> List[tuple]:
        fps = self._fps
        br = self._bitrate
        host = self._host
        port = self._port
        gop = max(1, fps)
        nv = (
            f'appsrc ! video/x-raw,format=BGR,width={w},height={h},framerate={fps}/1 ! '
            f'videoconvert ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! '
            f'nvv4l2h264enc bitrate={br} iframeinterval={gop} insert-sps-pps=1 '
            f'preset-level=1 control-rate=1 maxperf-enable=1 ! '
            f'h264parse ! rtph264pay config-interval=1 pt=96 ! '
            f'udpsink host={host} port={port} sync=false async=false'
        )
        x264_kbps = max(250, br // 1000)
        soft = (
            f'appsrc ! videoconvert ! video/x-raw,format=I420,width={w},height={h},'
            f'framerate={fps}/1 ! '
            f'x264enc tune=zerolatency bitrate={x264_kbps} speed-preset=ultrafast '
            f'key-int-max={gop} ! '
            f'h264parse ! rtph264pay config-interval=1 pt=96 ! '
            f'udpsink host={host} port={port} sync=false async=false'
        )
        return [('nvv4l2h264enc', nv), ('x264enc', soft)]

    def _open(self, w: int, h: int) -> bool:
        import cv2

        for name, pipe in self._pipelines(w, h):
            writer = cv2.VideoWriter(
                pipe, cv2.CAP_GSTREAMER, 0, float(self._fps), (w, h), True
            )
            if writer is not None and writer.isOpened():
                self._writer = writer
                self._size = (w, h)
                self._backend = name
                self._logger.info(
                    f'H.264 UDP open backend={name} {w}x{h}@{self._fps} '
                    f'-> udp://{self._host}:{self._port} bitrate={self._bitrate}'
                )
                return True
            if writer is not None:
                writer.release()
        self._logger.error(
            f'H.264 UDP open failed {w}x{h} -> {self._host}:{self._port} '
            f'(need OpenCV GStreamer + nvv4l2h264enc or x264enc)'
        )
        return False

    def close(self) -> None:
        if self._writer is not None:
            try:
                self._writer.release()
            except Exception:
                pass
            self._writer = None
            self._size = None


class YoloDetectNode(Node):
    """Minimal color → YOLO → Detection2DArray pipeline."""

    def __init__(self) -> None:
        super().__init__('yolo_detect_node')

        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('threshold', 0.3)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('debug_image_topic', '/yolo/debug_image')
        self.declare_parameter('publish_debug', True)
        # JPEG over Wi-Fi (default). Raw Image is large (~300KB @424x240).
        self.declare_parameter('publish_debug_compressed', True)
        self.declare_parameter('publish_debug_raw', False)
        self.declare_parameter('jpeg_quality', 65)
        # Fit annotated debug into this box before JPEG/raw (0 = no limit).
        self.declare_parameter('debug_max_width', 320)
        self.declare_parameter('debug_max_height', 180)
        # Optional HW H.264 RTP/UDP preview (bypasses ROS image over Wi-Fi).
        self.declare_parameter('h264_udp_enable', False)
        self.declare_parameter('h264_udp_host', '')
        self.declare_parameter('h264_udp_port', 5600)
        self.declare_parameter('h264_bitrate', 3000000)
        self.declare_parameter('h264_fps', 30)
        self.declare_parameter('allowed_classes', '')
        self.declare_parameter('denied_classes', '')
        # TRT engines often lack class-name metadata and fall back to COCO-80.
        # Custom engines (merged5 person+shell) must inject the real names here
        # or class 1 is labeled bicycle and dropped by allowed_classes=person.
        self.declare_parameter('class_names', '')
        # Optional per-class score cuts, e.g. person:0.65,shell:0.3
        self.declare_parameter('class_thresholds', '')
        # auto | true | false — auto enables for *.engine models
        self.declare_parameter('force_cpu_nms', 'auto')
        # 0 = unlimited; >0 caps inference Hz (independent of debug)
        self.declare_parameter('infer_rate', 0.0)
        # 0 = unlimited (camera rate); >0 caps debug JPEG/H264 publish Hz
        self.declare_parameter('debug_rate', 0.0)

        self._model_path = (
            self.get_parameter('model').get_parameter_value().string_value
        )
        self._device = (
            self.get_parameter('device').get_parameter_value().string_value
        )
        self._threshold = (
            self.get_parameter('threshold').get_parameter_value().double_value
        )
        self._imgsz = (
            self.get_parameter('imgsz').get_parameter_value().integer_value
        )
        image_topic = (
            self.get_parameter('image_topic').get_parameter_value().string_value
        )
        detections_topic = (
            self.get_parameter('detections_topic')
            .get_parameter_value()
            .string_value
        )
        debug_image_topic = (
            self.get_parameter('debug_image_topic')
            .get_parameter_value()
            .string_value
        )
        self._publish_debug = (
            self.get_parameter('publish_debug')
            .get_parameter_value()
            .bool_value
        )
        self._publish_debug_compressed = (
            self.get_parameter('publish_debug_compressed')
            .get_parameter_value()
            .bool_value
        )
        self._publish_debug_raw = (
            self.get_parameter('publish_debug_raw')
            .get_parameter_value()
            .bool_value
        )
        self._jpeg_quality = int(
            self.get_parameter('jpeg_quality')
            .get_parameter_value()
            .integer_value
        )
        self._jpeg_quality = max(1, min(100, self._jpeg_quality))
        self._debug_max_width = int(
            self.get_parameter('debug_max_width')
            .get_parameter_value()
            .integer_value
        )
        self._debug_max_height = int(
            self.get_parameter('debug_max_height')
            .get_parameter_value()
            .integer_value
        )
        self._debug_max_width = max(0, self._debug_max_width)
        self._debug_max_height = max(0, self._debug_max_height)
        self._h264_udp_enable = (
            self.get_parameter('h264_udp_enable')
            .get_parameter_value()
            .bool_value
        )
        self._h264_udp_host = (
            self.get_parameter('h264_udp_host')
            .get_parameter_value()
            .string_value
            .strip()
        )
        self._h264_udp_port = int(
            self.get_parameter('h264_udp_port')
            .get_parameter_value()
            .integer_value
        )
        self._h264_bitrate = int(
            self.get_parameter('h264_bitrate')
            .get_parameter_value()
            .integer_value
        )
        self._h264_fps = int(
            self.get_parameter('h264_fps')
            .get_parameter_value()
            .integer_value
        )
        self._h264_udp_port = max(1, min(65535, self._h264_udp_port))
        self._h264_bitrate = max(250000, min(20000000, self._h264_bitrate))
        self._h264_fps = max(1, min(60, self._h264_fps))
        self._h264_sink: Optional[_H264UdpSink] = None
        if self._h264_udp_enable and self._h264_udp_host:
            self._h264_sink = _H264UdpSink(
                host=self._h264_udp_host,
                port=self._h264_udp_port,
                bitrate=self._h264_bitrate,
                fps=self._h264_fps,
                logger=self.get_logger(),
            )
        elif self._h264_udp_enable:
            self.get_logger().warn(
                'h264_udp_enable=true but h264_udp_host empty; H.264 disabled'
            )
        self._allowed = parse_class_csv(
            self.get_parameter('allowed_classes')
            .get_parameter_value()
            .string_value
        )
        self._denied = parse_class_csv(
            self.get_parameter('denied_classes')
            .get_parameter_value()
            .string_value
        )
        self._class_names_override = parse_ordered_class_csv(
            self.get_parameter('class_names')
            .get_parameter_value()
            .string_value
        )
        self._class_thresholds = parse_class_thresholds(
            self.get_parameter('class_thresholds')
            .get_parameter_value()
            .string_value
        )
        self._predict_conf = predict_conf_floor(
            self._threshold, self._class_thresholds
        )
        force_cpu_nms_raw = (
            self.get_parameter('force_cpu_nms')
            .get_parameter_value()
            .string_value
            .strip()
            .lower()
        )
        is_engine = self._model_path.lower().endswith('.engine')
        if force_cpu_nms_raw in ('true', '1', 'yes'):
            self._force_cpu_nms = True
        elif force_cpu_nms_raw in ('false', '0', 'no'):
            self._force_cpu_nms = False
        else:
            self._force_cpu_nms = is_engine

        infer_rate = (
            self.get_parameter('infer_rate')
            .get_parameter_value()
            .double_value
        )
        self._infer_interval = (1.0 / infer_rate) if infer_rate > 0.0 else 0.0
        self._last_infer_mono = 0.0
        debug_rate = (
            self.get_parameter('debug_rate')
            .get_parameter_value()
            .double_value
        )
        self._debug_interval = (1.0 / debug_rate) if debug_rate > 0.0 else 0.0
        self._last_debug_mono = 0.0
        self._last_boxes: Optional[List[tuple]] = None

        self._bridge = CvBridge()
        self._busy = False
        self._busy_lock = threading.Lock()
        self._model = None
        self._class_names: Dict[int, str] = dict(_COCO80_NAMES)
        self._predict_classes: Optional[List[int]] = None

        self._detections_pub = self.create_publisher(
            Detection2DArray, detections_topic, 10
        )
        self._debug_pub = None
        self._debug_compressed_pub = None
        if self._publish_debug:
            qos_dbg = _best_effort_image_qos(1)
            # image_transport convention: base/compressed
            if self._publish_debug_compressed:
                self._debug_compressed_pub = self.create_publisher(
                    CompressedImage,
                    f'{debug_image_topic}/compressed',
                    qos_dbg,
                )
            if self._publish_debug_raw:
                self._debug_pub = self.create_publisher(
                    Image, debug_image_topic, qos_dbg
                )
            if self._debug_compressed_pub is None and self._debug_pub is None:
                # Fallback so publish_debug=true always has an output.
                self._debug_compressed_pub = self.create_publisher(
                    CompressedImage,
                    f'{debug_image_topic}/compressed',
                    qos_dbg,
                )
                self._publish_debug_compressed = True

        self._image_sub = self.create_subscription(
            Image,
            image_topic,
            self._on_image,
            _best_effort_image_qos(1),
        )

        self.get_logger().info(
            f'Loading YOLO model={self._model_path} device={self._device} '
            f'force_cpu_nms={self._force_cpu_nms} ...'
        )
        # Avoid Ultralytics online AutoUpdate / requirement downloads on Jetson.
        os.environ.setdefault('YOLO_OFFLINE', '1')
        os.environ.setdefault('ULTRALYTICS_OFFLINE', '1')
        # NumPy>=1.24 removed np.bool; some TRT/ultralytics paths still use it.
        try:
            import numpy as np

            if not hasattr(np, 'bool'):
                np.bool = bool  # type: ignore[attr-defined, assignment]
            if not hasattr(np, 'int'):
                np.int = int  # type: ignore[attr-defined, assignment]
            if not hasattr(np, 'float'):
                np.float = float  # type: ignore[attr-defined, assignment]
        except Exception:  # noqa: BLE001
            pass
        # Patch NMS before ultralytics imports torchvision.ops bindings.
        if self._force_cpu_nms:
            try:
                _patch_torchvision_nms_cpu()
                self.get_logger().info('torchvision NMS patched to pure-torch CPU')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'CPU NMS patch failed: {exc}')

        try:
            from ultralytics import YOLO
        except ImportError as exc:
            self.get_logger().fatal(
                'ultralytics is not installed. '
                "Install with: python3 -m pip install --user 'ultralytics>=8.0'"
            )
            raise SystemExit(1) from exc

        if self._force_cpu_nms:
            try:
                _patch_torchvision_nms_cpu()
            except Exception:  # noqa: BLE001
                pass

        # task=detect is required for bare TensorRT engines without sidecar metadata.
        self._model = YOLO(self._model_path, task='detect')
        is_engine = str(self._model_path).lower().endswith('.engine')
        override_map = names_from_override(self._class_names_override)
        names_source = 'model'
        if override_map is not None:
            self._class_names = override_map
            used_coco = False
            names_source = 'override'
        elif is_engine:
            # .engine exports often lack usable names; COCO-80 only if no override.
            self._class_names = dict(_COCO80_NAMES)
            used_coco = True
            names_source = 'engine_coco_fallback'
        else:
            self._class_names, used_coco = _safe_class_names(self._model)
            if used_coco:
                names_source = 'model_coco_fallback'
        self._bind_model_names(self._class_names)
        if used_coco:
            self.get_logger().warn(
                'model.names missing/unusable; using COCO-80 class names'
            )
        unmatched = unmatched_allowed_names(self._class_names, self._allowed)
        if unmatched:
            self.get_logger().warn(
                'allowed_classes names not in model map (dropped, not inferred): '
                f'{unmatched}. For merged5 set class_names:=person,shell'
            )
        self._predict_classes = resolve_predict_class_ids(
            self._class_names, self._allowed, self._denied
        )
        # Empty name maps can yield classes=[]; never pass that to predict.
        # Do not COCO-retry when the operator injected class_names — that
        # would rename shell back to bicycle.
        if self._allowed and not self._predict_classes and override_map is None:
            self.get_logger().warn(
                'class filter matched nothing; retrying with COCO-80 names'
            )
            self._class_names = dict(_COCO80_NAMES)
            names_source = 'coco_retry'
            self._bind_model_names(self._class_names)
            self._predict_classes = resolve_predict_class_ids(
                self._class_names, self._allowed, self._denied
            )
        if self._predict_classes is not None and len(self._predict_classes) == 0:
            self._predict_classes = None
        self._names_source = names_source
        # Warmup once so first camera frame does not pay engine deserialize cost.
        try:
            import numpy as _np

            dummy = _np.zeros((self._imgsz, self._imgsz, 3), dtype=_np.uint8)
            warm_kw = {
                'source': dummy,
                'conf': self._predict_conf,
                'iou': 0.45,
                'max_det': 50,
                'imgsz': self._imgsz,
                'device': self._device,
                'verbose': False,
            }
            if self._predict_classes:
                warm_kw['classes'] = self._predict_classes
            self._model.predict(**warm_kw)
            self.get_logger().info('YOLO warmup predict done')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'YOLO warmup failed (continuing): {exc}')
        if self._publish_debug:
            dbg_parts = []
            if self._debug_compressed_pub is not None:
                size_hint = ''
                if self._debug_max_width > 0 or self._debug_max_height > 0:
                    size_hint = (
                        f' max={self._debug_max_width or "∞"}x'
                        f'{self._debug_max_height or "∞"}'
                    )
                dbg_parts.append(
                    f'{debug_image_topic}/compressed'
                    f'(jpeg q={self._jpeg_quality}{size_hint})'
                )
            if self._debug_pub is not None:
                dbg_parts.append(f'{debug_image_topic}(raw)')
            dbg_desc = ','.join(dbg_parts) if dbg_parts else 'off'
        else:
            dbg_desc = 'off'
        infer_hz = (
            f'{1.0 / self._infer_interval:.1f}Hz'
            if self._infer_interval > 0.0
            else 'unlimited'
        )
        debug_hz = (
            f'{1.0 / self._debug_interval:.1f}Hz'
            if self._debug_interval > 0.0
            else 'unlimited'
        )
        preview = {
            idx: self._class_names[idx]
            for idx in sorted(self._class_names)[:4]
        }
        self.get_logger().info(
            f'YOLO ready. sub={image_topic} pub={detections_topic} '
            f'debug={dbg_desc} infer_rate={infer_hz} debug_rate={debug_hz} '
            f'threshold={self._threshold} predict_conf={self._predict_conf} '
            f'class_thresholds={self._class_thresholds or "none"} '
            f'imgsz={self._imgsz} '
            f'allowed={sorted(self._allowed) or "all"} '
            f'denied={sorted(self._denied) or "none"} '
            f'names_source={self._names_source} names={preview} '
            f'class_ids='
            f'{self._predict_classes if self._predict_classes is not None else "all"}'
        )

    def _bind_model_names(self, names: Dict[int, str]) -> None:
        try:
            object.__setattr__(self._model, 'names', dict(names))
        except Exception:  # noqa: BLE001
            try:
                setattr(self._model, 'names', dict(names))
            except Exception:  # noqa: BLE001
                pass

    def _class_ok(self, name: str) -> bool:
        return class_name_allowed(name, self._allowed, self._denied)

    def _on_image(self, msg: Image) -> None:
        with self._busy_lock:
            if self._busy:
                return
            self._busy = True

        try:
            self._process_image(msg)
        except Exception as exc:  # noqa: BLE001 - keep node alive on frame errors
            self.get_logger().error(f'YOLO frame failed: {exc}')
        finally:
            with self._busy_lock:
                self._busy = False

    def _process_image(self, msg: Image) -> None:
        assert self._model is not None

        need_debug = (
            self._debug_pub is not None
            or self._debug_compressed_pub is not None
            or self._h264_sink is not None
        )

        now = time.monotonic()
        do_infer = (
            self._infer_interval <= 0.0
            or self._last_boxes is None
            or (now - self._last_infer_mono) >= self._infer_interval
        )
        # debug_rate independent of infer_rate (0 = every processed debug path).
        do_debug = False
        if need_debug:
            if self._debug_interval <= 0.0:
                do_debug = True
            elif self._last_debug_mono <= 0.0:
                do_debug = True
            elif (now - self._last_debug_mono) >= self._debug_interval:
                do_debug = True

        # Skip decode/draw/JPEG entirely when neither infer nor debug is due.
        if not do_infer and not do_debug:
            return

        cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if not do_infer:
            # Light path: no predict; redraw cached boxes at debug_rate only.
            self._emit_debug(msg, self._draw_cached(cv_image))
            self._last_debug_mono = time.monotonic()
            return

        predict_kwargs = {
            'source': cv_image,
            'conf': self._predict_conf,
            'iou': 0.45,  # 표준 YOLO NMS IoU — 기본 0.7은 중복 박스 과다 유발
            'max_det': 50,  # 장면당 최대 박스 수 제한 (기본 300)
            'imgsz': self._imgsz,
            'device': self._device,
            'verbose': False,
        }
        if self._predict_classes:
            predict_kwargs['classes'] = self._predict_classes

        results = self._model.predict(**predict_kwargs)
        result = results[0]

        det_array = Detection2DArray()
        det_array.header = msg.header

        cached_boxes: List[tuple] = []
        boxes = result.boxes
        if boxes is not None and len(boxes) > 0:
            # Prefer node COCO/fallback names. TRT result.names is often class0..N
            # which breaks allowed_classes filters like "person".
            names = self._class_names
            result_names = getattr(result, 'names', None)
            if isinstance(result_names, dict) and result_names:
                try:
                    parsed = {int(k): str(v) for k, v in result_names.items()}
                    if not _names_look_placeholder(parsed):
                        names = parsed
                except (TypeError, ValueError):
                    pass
            xyxy = boxes.xyxy.cpu().numpy()
            confs = boxes.conf.cpu().numpy()
            clss = boxes.cls.cpu().numpy().astype(int)

            for i in range(len(xyxy)):
                x1, y1, x2, y2 = xyxy[i]
                w = float(x2 - x1)
                h = float(y2 - y1)
                cx = float(x1 + w / 2.0)
                cy = float(y1 + h / 2.0)
                class_id = int(clss[i])
                score = float(confs[i])
                class_name = str(names.get(class_id, class_id))
                if not self._class_ok(class_name):
                    continue
                if not score_meets_threshold(
                    class_name,
                    score,
                    self._threshold,
                    self._class_thresholds,
                ):
                    continue

                det = Detection2D()
                det.header = msg.header
                det.bbox = BoundingBox2D()
                # vision_msgs: Pose2D.center.position.{x,y} + size_x/size_y
                det.bbox.center = Pose2D()
                det.bbox.center.position = Point2D()
                det.bbox.center.position.x = cx
                det.bbox.center.position.y = cy
                det.bbox.center.theta = 0.0
                det.bbox.size_x = w
                det.bbox.size_y = h

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = class_name
                hyp.hypothesis.score = score
                det.results.append(hyp)
                det.id = class_name
                det_array.detections.append(det)
                cached_boxes.append(
                    (int(x1), int(y1), int(x2), int(y2), class_name, score)
                )

        self._detections_pub.publish(det_array)
        self._last_infer_mono = time.monotonic()
        self._last_boxes = cached_boxes

        if do_debug:
            # Do not call result.plot(): it draws unfiltered predict() boxes
            # and ignores infer_rate cache. Labels come from published names.
            try:
                result.names = dict(self._class_names)
            except Exception:
                pass
            self._emit_debug(msg, self._draw_cached(cv_image))
            self._last_debug_mono = time.monotonic()

    def _draw_cached(self, frame):
        """Draw cached boxes on a fresh frame (no inference this tick)."""
        import cv2

        annotated = frame
        for item in (self._last_boxes or []):
            if len(item) == 6:
                x1, y1, x2, y2, class_name, score = item
                label = debug_box_label(str(class_name), float(score))
                color = debug_box_bgr(str(class_name))
            else:
                # Older 5-tuple cache (label only) — keep drawing if present.
                x1, y1, x2, y2, label = item
                color = debug_box_bgr(str(label).split(' ', 1)[0])
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                annotated, str(label), (x1, max(10, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA,
            )
        return annotated

    def _emit_debug(self, msg: Image, annotated) -> None:
        if self._h264_sink is not None:
            self._h264_sink.write(annotated)
        view = self._downscale_debug(annotated)
        if self._debug_compressed_pub is not None:
            import cv2

            ok, buf = cv2.imencode(
                '.jpg',
                view,
                [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality],
            )
            if ok:
                cmsg = CompressedImage()
                cmsg.header = msg.header
                # image_transport compressed plugin style
                cmsg.format = 'bgr8; jpeg compressed'
                cmsg.data = buf.tobytes()
                self._debug_compressed_pub.publish(cmsg)
        if self._debug_pub is not None:
            debug_msg = self._bridge.cv2_to_imgmsg(view, encoding='bgr8')
            debug_msg.header = msg.header
            self._debug_pub.publish(debug_msg)

    def _downscale_debug(self, frame):
        """Shrink annotated frame to debug_max_* for Wi-Fi JPEG (aspect kept)."""
        max_w = self._debug_max_width
        max_h = self._debug_max_height
        if max_w <= 0 and max_h <= 0:
            return frame
        h, w = frame.shape[:2]
        scale = 1.0
        if max_w > 0:
            scale = min(scale, float(max_w) / float(w))
        if max_h > 0:
            scale = min(scale, float(max_h) / float(h))
        if scale >= 1.0:
            return frame
        import cv2

        new_w = max(1, int(round(w * scale)))
        new_h = max(1, int(round(h * scale)))
        return cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)

    def destroy_node(self) -> None:
        if self._h264_sink is not None:
            self._h264_sink.close()
            self._h264_sink = None
        super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = YoloDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
