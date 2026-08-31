"""Lift YOLO Detection2DArray to camera-frame 3D using aligned depth.

Matches each detection stamp to the nearest buffered depth frame (time sync),
samples a robust median depth over the box interior, then deprojects to (x,y,z).
"""

from __future__ import annotations

import threading
from collections import deque
from typing import Deque, List, Optional, Set, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from vision_msgs.msg import (
    BoundingBox3D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray


def _best_effort_image_qos(depth: int = 5) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _parse_class_csv(raw: str) -> Set[str]:
    return {p.strip().lower() for p in raw.split(',') if p.strip()}


class Yolo3dNode(Node):
    """Detection2DArray + aligned depth → Detection3DArray (camera frame)."""

    def __init__(self) -> None:
        super().__init__('yolo_3d_node')

        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter(
            'depth_topic',
            '/camera/camera/aligned_depth_to_color/image_raw',
        )
        self.declare_parameter(
            'camera_info_topic',
            '/camera/camera/color/camera_info',
        )
        self.declare_parameter('detections_3d_topic', '/yolo/detections_3d')
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('publish_box_markers', True)
        self.declare_parameter('use_optical_frame', False)
        self.declare_parameter('markers_topic', '/yolo/detections_3d_markers')
        self.declare_parameter(
            'marker_frame',
            'camera_link',
        )
        self.declare_parameter('marker_lifetime_sec', 1.0)
        self.declare_parameter('depth_window', 3)
        self.declare_parameter('depth_bbox_scale', 0.35)
        self.declare_parameter('min_valid_depth_ratio', 0.15)
        self.declare_parameter('depth_trim_fraction', 0.1)
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_depth_m', 8.0)
        # YOLO latency + multi-container hop often exceeds 80ms stamp gap.
        self.declare_parameter('max_depth_dt_sec', 0.35)
        self.declare_parameter('depth_buffer_size', 60)
        # If no stamp match, still use newest depth (prefer markers over silence).
        self.declare_parameter('depth_use_latest_fallback', True)
        self.declare_parameter('allowed_classes', '')
        self.declare_parameter('denied_classes', '')

        detections_topic = (
            self.get_parameter('detections_topic')
            .get_parameter_value()
            .string_value
        )
        depth_topic = (
            self.get_parameter('depth_topic').get_parameter_value().string_value
        )
        camera_info_topic = (
            self.get_parameter('camera_info_topic')
            .get_parameter_value()
            .string_value
        )
        detections_3d_topic = (
            self.get_parameter('detections_3d_topic')
            .get_parameter_value()
            .string_value
        )
        self._publish_markers = (
            self.get_parameter('publish_markers')
            .get_parameter_value()
            .bool_value
        )
        self._publish_box_markers = (
            self.get_parameter('publish_box_markers')
            .get_parameter_value()
            .bool_value
        )
        self._use_optical_frame = (
            self.get_parameter('use_optical_frame')
            .get_parameter_value()
            .bool_value
        )
        markers_topic = (
            self.get_parameter('markers_topic').get_parameter_value().string_value
        )
        self._marker_frame = (
            self.get_parameter('marker_frame').get_parameter_value().string_value
        )
        self._marker_lifetime_sec = (
            self.get_parameter('marker_lifetime_sec')
            .get_parameter_value()
            .double_value
        )
        self._depth_window = max(
            0,
            self.get_parameter('depth_window')
            .get_parameter_value()
            .integer_value,
        )
        self._depth_bbox_scale = max(
            0.05,
            self.get_parameter('depth_bbox_scale')
            .get_parameter_value()
            .double_value,
        )
        self._min_valid_depth_ratio = float(
            np.clip(
                self.get_parameter('min_valid_depth_ratio')
                .get_parameter_value()
                .double_value,
                0.0,
                1.0,
            )
        )
        self._depth_trim_fraction = float(
            np.clip(
                self.get_parameter('depth_trim_fraction')
                .get_parameter_value()
                .double_value,
                0.0,
                0.4,
            )
        )
        self._min_depth_m = (
            self.get_parameter('min_depth_m').get_parameter_value().double_value
        )
        self._max_depth_m = (
            self.get_parameter('max_depth_m').get_parameter_value().double_value
        )
        self._max_depth_dt_sec = (
            self.get_parameter('max_depth_dt_sec')
            .get_parameter_value()
            .double_value
        )
        self._depth_buffer_size = max(
            1,
            self.get_parameter('depth_buffer_size')
            .get_parameter_value()
            .integer_value,
        )
        self._depth_use_latest_fallback = (
            self.get_parameter('depth_use_latest_fallback')
            .get_parameter_value()
            .bool_value
        )
        self._fallback_depth = 0
        self._allowed = _parse_class_csv(
            self.get_parameter('allowed_classes')
            .get_parameter_value()
            .string_value
        )
        self._denied = _parse_class_csv(
            self.get_parameter('denied_classes')
            .get_parameter_value()
            .string_value
        )

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        # (stamp_sec, frame_id, depth_img)
        self._depth_buf: Deque[Tuple[float, str, np.ndarray]] = deque(
            maxlen=self._depth_buffer_size
        )
        self._fx = 0.0
        self._fy = 0.0
        self._cx = 0.0
        self._cy = 0.0
        self._have_info = False
        self._drop_sync = 0
        self._drop_depth = 0

        self._pub = self.create_publisher(
            Detection3DArray, detections_3d_topic, 10
        )
        self._marker_pub = None
        if self._publish_markers:
            self._marker_pub = self.create_publisher(
                MarkerArray, markers_topic, 10
            )
        self.create_subscription(
            Detection2DArray,
            detections_topic,
            self._on_detections,
            10,
        )
        self.create_subscription(
            Image,
            depth_topic,
            self._on_depth,
            _best_effort_image_qos(5),
        )
        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._on_camera_info,
            10,
        )

        self.get_logger().info(
            f'yolo_3d ready. det={detections_topic} depth={depth_topic} '
            f'info={camera_info_topic} out={detections_3d_topic} '
            f'markers={markers_topic if self._publish_markers else "off"} '
            f'max_dt={self._max_depth_dt_sec}s buf={self._depth_buffer_size} '
            f'allowed={sorted(self._allowed) or "all"} '
            f'denied={sorted(self._denied) or "none"}'
        )

    def _class_ok(self, name: str) -> bool:
        key = (name or '').strip().lower()
        if self._denied and key in self._denied:
            return False
        if self._allowed and key not in self._allowed:
            return False
        return True

    def _on_camera_info(self, msg: CameraInfo) -> None:
        with self._lock:
            self._fx = float(msg.k[0])
            self._fy = float(msg.k[4])
            self._cx = float(msg.k[2])
            self._cy = float(msg.k[5])
            self._have_info = self._fx > 0.0 and self._fy > 0.0

    def _on_depth(self, msg: Image) -> None:
        try:
            if msg.encoding in ('16UC1', 'mono16'):
                depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            elif msg.encoding == '32FC1':
                depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                depth = self._bridge.imgmsg_to_cv2(
                    msg, desired_encoding='passthrough'
                )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'depth convert failed: {exc}')
            return

        stamp = _stamp_to_sec(msg.header.stamp)
        with self._lock:
            self._depth_buf.append(
                (stamp, msg.header.frame_id, np.asarray(depth).copy())
            )

    def _pick_depth(
        self, target_stamp: float
    ) -> Optional[Tuple[float, str, np.ndarray]]:
        with self._lock:
            if not self._depth_buf:
                return None
            best = None
            best_dt = float('inf')
            for item in self._depth_buf:
                dt = abs(item[0] - target_stamp)
                if best is None or dt < best_dt:
                    best = item
                    best_dt = dt
            if best is None:
                return None
            if best_dt <= self._max_depth_dt_sec:
                return best[0], best[1], best[2]
            if self._depth_use_latest_fallback:
                latest = self._depth_buf[-1]
                return latest[0], latest[1], latest[2]
            return None

    def _sample_depth_m(
        self,
        depth_img: np.ndarray,
        u: float,
        v: float,
        size_x: float,
        size_y: float,
    ) -> Optional[float]:
        if depth_img.ndim != 2:
            if depth_img.ndim == 3 and depth_img.shape[2] == 1:
                depth_img = depth_img[:, :, 0]
            else:
                return None

        h, w = depth_img.shape[:2]
        ui = int(round(u))
        vi = int(round(v))
        if ui < 0 or vi < 0 or ui >= w or vi >= h:
            return None

        half_x = max(
            self._depth_window,
            int(round(0.5 * abs(size_x) * self._depth_bbox_scale)),
        )
        half_y = max(
            self._depth_window,
            int(round(0.5 * abs(size_y) * self._depth_bbox_scale)),
        )
        half_x = min(half_x, max(1, w // 4))
        half_y = min(half_y, max(1, h // 4))

        x0 = max(0, ui - half_x)
        x1 = min(w, ui + half_x + 1)
        y0 = max(0, vi - half_y)
        y1 = min(h, vi + half_y + 1)
        window = depth_img[y0:y1, x0:x1]
        if window.size == 0:
            return None

        if depth_img.dtype == np.float32 or depth_img.dtype == np.float64:
            meters = window.astype(np.float64).reshape(-1)
        else:
            meters = window.astype(np.float64).reshape(-1) * 0.001

        valid = meters[
            (meters >= self._min_depth_m) & (meters <= self._max_depth_m)
        ]
        if valid.size == 0:
            return None
        if valid.size / float(window.size) < self._min_valid_depth_ratio:
            return None

        # Trim extremes then median (stable against edges / holes).
        if valid.size >= 8 and self._depth_trim_fraction > 0.0:
            lo = np.quantile(valid, self._depth_trim_fraction)
            hi = np.quantile(valid, 1.0 - self._depth_trim_fraction)
            trimmed = valid[(valid >= lo) & (valid <= hi)]
            if trimmed.size > 0:
                valid = trimmed

        return float(np.median(valid))

    def _deproject(
        self, u: float, v: float, z: float, fx: float, fy: float, cx: float, cy: float
    ) -> Tuple[float, float, float]:
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return x, y, z

    def _on_detections(self, msg: Detection2DArray) -> None:
        with self._lock:
            have_info = self._have_info
            fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy

        if not have_info:
            return

        target = _stamp_to_sec(msg.header.stamp)
        picked = self._pick_depth(target)
        if picked is None:
            self._drop_sync += 1
            if self._drop_sync % 30 == 1:
                self.get_logger().warn(
                    f'no depth within {self._max_depth_dt_sec}s of '
                    f'detection stamp (drops={self._drop_sync})'
                )
            return

        depth_stamp, depth_frame, depth_img = picked
        dt = abs(depth_stamp - target)
        if dt > self._max_depth_dt_sec:
            self._fallback_depth += 1
            if self._fallback_depth % 60 == 1:
                self.get_logger().warn(
                    f'depth stamp fallback dt={dt:.3f}s '
                    f'(max={self._max_depth_dt_sec}s, n={self._fallback_depth})'
                )

        out = Detection3DArray()
        out.header = msg.header
        if depth_frame:
            out.header.frame_id = depth_frame

        for det2d in msg.detections:
            class_name = det2d.id or ''
            if det2d.results:
                class_name = det2d.results[0].hypothesis.class_id or class_name
            if not self._class_ok(class_name):
                continue

            u = float(det2d.bbox.center.position.x)
            v = float(det2d.bbox.center.position.y)
            size_x = float(det2d.bbox.size_x)
            size_y = float(det2d.bbox.size_y)
            z = self._sample_depth_m(depth_img, u, v, size_x, size_y)
            if z is None:
                self._drop_depth += 1
                continue

            x, y, z = self._deproject(u, v, z, fx, fy, cx, cy)

            size_x_m = max(0.01, size_x * z / fx)
            size_y_m = max(0.01, size_y * z / fy)
            size_z_m = max(0.01, min(size_x_m, size_y_m))

            det3d = Detection3D()
            det3d.header = out.header
            det3d.id = det2d.id

            pose = Pose()
            pose.position = Point(x=x, y=y, z=z)
            pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

            det3d.bbox = BoundingBox3D()
            det3d.bbox.center = pose
            det3d.bbox.size = Vector3(x=size_x_m, y=size_y_m, z=size_z_m)

            if det2d.results:
                for hyp2d in det2d.results:
                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = hyp2d.hypothesis.class_id
                    hyp.hypothesis.score = hyp2d.hypothesis.score
                    hyp.pose.pose = pose
                    det3d.results.append(hyp)
            else:
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = det2d.id or 'unknown'
                hyp.hypothesis.score = 0.0
                hyp.pose.pose = pose
                det3d.results.append(hyp)

            out.detections.append(det3d)

        self._pub.publish(out)
        if self._marker_pub is not None:
            self._marker_pub.publish(self._to_markers(out))

    def _marker_lifetime(self, life: float):
        sec = int(life)
        nsec = int((life - sec) * 1e9)
        return sec, nsec

    @staticmethod
    def _optical_to_link(x: float, y: float, z: float) -> Tuple[float, float, float]:
        """REP-103 camera_link (X fwd, Y left, Z up) ← optical (X right, Y down, Z fwd)."""
        return z, -x, -y

    def _to_markers(self, dets: Detection3DArray) -> MarkerArray:
        """RViz / Foxglove 3D markers with Bounding Box CUBE and optional person figures.

        Detections are in color optical frame; markers can be output in optical frame
        or camera_link frame (Z-up) for RViz ground plane alignment.
        """
        arr = MarkerArray()
        if self._use_optical_frame:
            marker_frame = dets.header.frame_id or self._marker_frame
        else:
            marker_frame = self._marker_frame or dets.header.frame_id

        clear = Marker()
        clear.header.stamp = dets.header.stamp
        clear.header.frame_id = marker_frame
        clear.ns = 'yolo_3d'
        clear.id = 0
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        life = self._marker_lifetime_sec
        life_sec, life_nsec = self._marker_lifetime(life)
        q_id = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        # Colors for visualization
        box_person_color = ColorRGBA(r=0.15, g=0.75, b=0.95, a=0.45)
        box_obstacle_color = ColorRGBA(r=0.95, g=0.60, b=0.15, a=0.55)
        body_color = ColorRGBA(r=0.15, g=0.75, b=0.95, a=0.85)
        head_color = ColorRGBA(r=1.0, g=0.85, b=0.55, a=0.95)
        text_color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)

        for i, det in enumerate(dets.detections):
            class_id = det.id or 'obj'
            score = 0.0
            if det.results:
                class_id = det.results[0].hypothesis.class_id or class_id
                score = float(det.results[0].hypothesis.score)

            xo = float(det.bbox.center.position.x)
            yo = float(det.bbox.center.position.y)
            zo = float(det.bbox.center.position.z)

            sx = float(det.bbox.size.x)
            sy = float(det.bbox.size.y)
            sz = float(det.bbox.size.z)

            is_optical = self._use_optical_frame or (marker_frame == dets.header.frame_id)
            if is_optical:
                px, py, pz = xo, yo, zo
                scale_x, scale_y, scale_z = sx, sy, sz
            else:
                px, py, pz = self._optical_to_link(xo, yo, zo)
                # In optical -> link (REP-103):
                # optical Z (fwd) -> link X (fwd)
                # optical X (right) -> link Y (left)
                # optical Y (down) -> link Z (up)
                scale_x, scale_y, scale_z = sz, sx, sy

            depth_m = zo
            base_id = i * 10 + 1
            is_person = (class_id.lower() == 'person')

            def _base_marker(mid: int, mtype: int) -> Marker:
                m = Marker()
                m.header.stamp = dets.header.stamp
                m.header.frame_id = marker_frame
                m.ns = 'yolo_3d'
                m.id = mid
                m.type = mtype
                m.action = Marker.ADD
                m.pose.orientation = q_id
                m.lifetime.sec = life_sec
                m.lifetime.nanosec = life_nsec
                return m

            # 1. 3D Bounding Box (CUBE) Marker for Foxglove / RViz
            if self._publish_box_markers:
                box = _base_marker(base_id, Marker.CUBE)
                box.pose.position = Point(x=px, y=py, z=pz)
                box.scale = Vector3(
                    x=max(0.05, scale_x),
                    y=max(0.05, scale_y),
                    z=max(0.05, scale_z),
                )
                box.color = box_person_color if is_person else box_obstacle_color
                arr.markers.append(box)

            # 2. Additional Human Figure markers if class is person
            if is_person:
                height = max(sy, sx, 0.4)
                width = max(sx, height * 0.28)
                width = min(width, height * 0.55)

                head_r = max(0.04, height * 0.09)
                head = _base_marker(base_id + 1, Marker.SPHERE)
                head.pose.position = Point(x=px, y=py, z=pz + (0.42 * height if not is_optical else -0.42 * height))
                head.scale = Vector3(x=2.0 * head_r, y=2.0 * head_r, z=2.0 * head_r)
                head.color = head_color
                arr.markers.append(head)

                torso_h = height * 0.38
                torso_d = width * 0.55
                torso = _base_marker(base_id + 2, Marker.CYLINDER)
                torso.pose.position = Point(x=px, y=py, z=pz + (0.08 * height if not is_optical else -0.08 * height))
                torso.scale = Vector3(x=torso_d, y=torso_d, z=torso_h)
                torso.color = body_color
                arr.markers.append(torso)

            # 3. Label Text Marker
            text_z_offset = (0.55 * scale_z) if not is_optical else (-0.55 * scale_z)
            text = _base_marker(base_id + 6, Marker.TEXT_VIEW_FACING)
            text.pose.position = Point(x=px, y=py, z=pz + text_z_offset)
            text.scale = Vector3(x=0.0, y=0.0, z=max(0.08, scale_z * 0.2))
            text.color = text_color
            text.text = f'{class_id} {score:.2f} z={depth_m:.2f}m'
            arr.markers.append(text)

        return arr


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = Yolo3dNode()
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
