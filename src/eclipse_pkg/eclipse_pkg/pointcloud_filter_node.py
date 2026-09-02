#!/usr/bin/env python3
"""ROI + voxel downsample filter for the RealSense point cloud.

Sits between the camera driver and the nav2 costmaps. Keeps only points
inside the robot-frame box the costmaps can actually use, then voxel-
downsamples to costmap resolution. Publishes an XYZ-only cloud in the
original camera depth optical frame so TF handling stays unchanged.

Camera depth optical frame -> base_link (static TF,
t=(0.270, 0.000, 0.050), q=(-0.5, 0.5, -0.5, 0.5)):
    base_x =  cam_z + 0.270
    base_y = -cam_x
    base_z = -cam_y + 0.050
so the robot-frame ROI is a simple axis-aligned box in camera
coordinates (no per-point rotation).

Hot path note (2026-08-10 py-spy on Jetson): ~40% of process samples were in
``np.unique`` during voxel downsample. Replaced with sort + first-of-run
mask (same one-point-per-voxel occupancy for costmaps; which representative
point is kept may differ from ``unique``'s first-original-order rule).
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

_XYZ_OUT_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]

# Structured output avoids np.stack + astype intermediate allocations.
_XYZ_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4')])


def _best_effort_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _voxel_first_indices(
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    inv: float,
) -> np.ndarray:
    """Indices of one representative point per voxel cell.

    Quantization matches the previous ``(coord * inv).astype(int)`` truncate
    toward zero. Keys are packed int64 so a single 1-D sort is enough.
    """
    # int32 quantize first (cheaper), then widen for packing.
    kx = (x * inv).astype(np.int32, copy=False)
    ky = (y * inv).astype(np.int32, copy=False)
    kz = (z * inv).astype(np.int32, copy=False)
    # +32768 keeps common ROI ranges non-negative inside 17-bit fields.
    keys = (
        ((kx.astype(np.int64) + 32768) << 34)
        | ((ky.astype(np.int64) + 32768) << 17)
        | (kz.astype(np.int64) + 32768)
    )
    # sort + first-of-run: typically much faster than np.unique(..., return_index)
    # for large N (py-spy hotspot on Jetson Xavier).
    order = np.argsort(keys, kind='quicksort')
    ks = keys[order]
    n = ks.size
    if n == 1:
        return order
    first = np.empty(n, dtype=bool)
    first[0] = True
    np.not_equal(ks[1:], ks[:-1], out=first[1:])
    return order[first]


class PointCloudFilterNode(Node):
    def __init__(self) -> None:
        super().__init__('pointcloud_filter_node')
        # Robot-frame (base_link) ROI the costmaps care about.
        self.declare_parameter('x_min', -0.30)
        self.declare_parameter('x_max', 5.0)
        self.declare_parameter('y_max', 2.50)
        self.declare_parameter('z_min', -2.0)
        self.declare_parameter('z_max', 2.0)
        self.declare_parameter('voxel_size', 0.05)
        # 0 = no gating; >0 caps processed input frames per second.
        self.declare_parameter('max_rate', 15.0)
        self.declare_parameter(
            'input_topic', '/camera/camera/depth/color/points'
        )
        self.declare_parameter('output_topic', '/camera/points_filtered')

        dv = lambda n: (  # noqa: E731
            self.get_parameter(n).get_parameter_value().double_value
        )
        x_min, x_max = dv('x_min'), dv('x_max')
        y_max = dv('y_max')
        z_min, z_max = dv('z_min'), dv('z_max')
        self._voxel = dv('voxel_size')
        if self._voxel <= 0.0:
            raise ValueError('voxel_size must be > 0')
        self._inv = 1.0 / self._voxel
        rate = dv('max_rate')
        self._min_dt = 1.0 / rate if rate > 0.0 else 0.0
        self._last_t = -1.0

        # Equivalent box in camera depth optical coordinates.
        self._cz_min = max(0.05, x_min - 0.270)
        self._cz_max = x_max - 0.270
        self._cx_max = y_max
        self._cy_min = 0.050 - z_max
        self._cy_max = 0.050 - z_min

        sv = self.get_parameter('input_topic').get_parameter_value().string_value
        ov = self.get_parameter('output_topic').get_parameter_value().string_value

        self._pub = self.create_publisher(PointCloud2, ov, _best_effort_qos(1))
        self._sub = self.create_subscription(
            PointCloud2, sv, self._on_cloud, _best_effort_qos(1)
        )
        self._msgs = 0
        self._in_sum = 0
        self._out_sum = 0
        self.get_logger().info(
            f'pc filter ready: {sv} -> {ov} voxel={self._voxel} '
            f'cam_box cz[{self._cz_min:.2f},{self._cz_max:.2f}] '
            f'cx[{-self._cx_max:.2f},{self._cx_max:.2f}] '
            f'cy[{self._cy_min:.2f},{self._cy_max:.2f}] '
            f'(voxel via sort+first, not np.unique)'
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        n = int(msg.width) * int(msg.height)
        ps = int(msg.point_step)
        if n == 0 or ps < 12 or not msg.data:
            return

        # Frame-rate gate on the input (header stamp based).
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if (
            self._min_dt > 0.0
            and self._last_t >= 0.0
            and (t - self._last_t) < self._min_dt
        ):
            return
        self._last_t = t

        # Fast path: xyz float32 prefix (realsense layout) -> single view,
        # no per-axis copies.
        f = msg.fields
        if (
            len(f) >= 3
            and f[0].name == 'x' and f[0].offset == 0
            and f[1].name == 'y' and f[1].offset == 4
            and f[2].name == 'z' and f[2].offset == 8
            and f[0].datatype == PointField.FLOAT32
        ):
            dt = np.dtype(
                [('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                 ('pad', f'V{ps - 12}')]
            )
            pts = np.frombuffer(msg.data, dtype=dt, count=n)
            x, y, z = pts['x'], pts['y'], pts['z']
        else:
            off = {}
            for fld in f:
                if (
                    fld.name in ('x', 'y', 'z')
                    and fld.datatype == PointField.FLOAT32
                ):
                    off[fld.name] = fld.offset
            if len(off) != 3:
                self.get_logger().warn(
                    'unexpected PointCloud2 fields; dropping', once=True
                )
                return
            raw = np.frombuffer(msg.data, dtype=np.uint8, count=n * ps)
            grid = raw.reshape(n, ps)

            def f32(name: str) -> np.ndarray:
                o = off[name]
                return (
                    np.ascontiguousarray(grid[:, o:o + 4])
                    .view(np.float32)
                    .reshape(n)
                )

            x = f32('x')
            y = f32('y')
            z = f32('z')

        # ROI mask (camera frame). NaN comparisons evaluate False -> dropped.
        m = (
            (z > self._cz_min)
            & (z < self._cz_max)
            & (x > -self._cx_max)
            & (x < self._cx_max)
            & (y > self._cy_min)
            & (y < self._cy_max)
        )
        x, y, z = x[m], y[m], z[m]

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.fields = _XYZ_OUT_FIELDS
        out.is_bigendian = False
        out.point_step = 12
        out.is_dense = True

        if x.size:
            idx = _voxel_first_indices(x, y, z, self._inv)
            n_out = int(idx.size)
            packed = np.empty(n_out, dtype=_XYZ_DTYPE)
            packed['x'] = x[idx]
            packed['y'] = y[idx]
            packed['z'] = z[idx]
            out.width = n_out
            out.row_step = 12 * n_out
            out.data = packed.tobytes()
        else:
            out.width = 0
            out.row_step = 0
            out.data = b''

        self._pub.publish(out)

        self._msgs += 1
        self._in_sum += n
        self._out_sum += int(out.width)
        if self._msgs % 300 == 0:
            self.get_logger().info(
                f'pc filter avg: in={self._in_sum / self._msgs:.0f} '
                f'out={self._out_sum / self._msgs:.0f} pts'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudFilterNode()
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
