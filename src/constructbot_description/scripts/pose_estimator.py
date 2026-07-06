#!/usr/bin/env python3
"""
ConstructBot 3D Pose Estimator — fake detection mode
=====================================================
Simulates a YOLO detection at a hardcoded pixel (u, v).
Reads depth at that pixel from /camera/depth.
Back-projects to 3D using camera intrinsics from /camera/camera_info.
Transforms the 3D point from camera_optical_frame → base frame via TF.
Publishes result as /detected_object_pose (PoseStamped).

To test: watch /detected_object_pose and verify XYZ makes sense
given where the camera is pointing in Isaac Sim.

Swap FAKE_U, FAKE_V to match the pixel center of your target object.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import numpy as np
import struct


# ── Fake detection pixel ──────────────────────────────────────────
# Set these to the pixel center of whatever object you want to pick.
# In Isaac Sim, you can find pixel coords by hovering over the
# /camera/image_raw feed in rqt_image_view and reading the status bar.
FAKE_U = 640   # center of 1280px wide image (x pixel)
FAKE_V = 360   # center of 720px tall image  (y pixel)
# ──────────────────────────────────────────────────────────────────


class PoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimator')

        self.camera_info = None
        self.latest_depth = None

        # Subscribers
        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 self.camera_info_cb, 10)
        self.create_subscription(Image, '/camera/depth',
                                 self.depth_cb, 10)

        # Publisher
        self.pose_pub = self.create_publisher(PoseStamped,
                                              '/detected_object_pose', 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer — estimate at 2hz (enough to verify, not spammy)
        self.create_timer(0.5, self.estimate)

        self.get_logger().info('Pose estimator started.')
        self.get_logger().info(f'Fake detection pixel: u={FAKE_U}, v={FAKE_V}')
        self.get_logger().info('Publishing to /detected_object_pose')

    def camera_info_cb(self, msg):
        self.camera_info = msg

    def depth_cb(self, msg):
        self.latest_depth = msg

    def get_depth_at_pixel(self, u, v):
        """Read depth value (metres) at pixel (u, v) from 32FC1 depth image."""
        msg = self.latest_depth
        if msg is None:
            return None

        # Clamp to image bounds
        u = max(0, min(u, msg.width - 1))
        v = max(0, min(v, msg.height - 1))

        # 32FC1 = 4 bytes per pixel, single float
        byte_offset = (v * msg.width + u) * 4
        if byte_offset + 4 > len(msg.data):
            return None

        depth = struct.unpack_from('f', bytes(msg.data[byte_offset:byte_offset + 4]))[0]

        # NaN or inf = no valid depth at this pixel
        if not np.isfinite(depth) or depth <= 0.0:
            return None

        return float(depth)

    def estimate(self):
        if self.camera_info is None:
            self.get_logger().warn('Waiting for /camera/camera_info...', throttle_duration_sec=3.0)
            return
        if self.latest_depth is None:
            self.get_logger().warn('Waiting for /camera/depth...', throttle_duration_sec=3.0)
            return

        # ── Step 1: get depth at fake detection pixel ──────────────
        D = self.get_depth_at_pixel(FAKE_U, FAKE_V)
        if D is None:
            self.get_logger().warn(
                f'No valid depth at pixel ({FAKE_U}, {FAKE_V}) — '
                'point may be aimed at sky/background. '
                'Try moving an object in front of the camera.',
                throttle_duration_sec=3.0
            )
            return

        # ── Step 2: camera intrinsics from camera_info ─────────────
        K = self.camera_info.k  # 3x3 intrinsic matrix, row-major
        fx = K[0]
        fy = K[4]
        cx = K[2]
        cy = K[5]

        # ── Step 3: back-project pixel → 3D in camera_optical_frame ─
        X = (FAKE_U - cx) * D / fx
        Y = (FAKE_V - cy) * D / fy
        Z = D

        self.get_logger().info(
            f'Pixel ({FAKE_U},{FAKE_V}) depth={D:.3f}m → '
            f'camera frame: ({X:.3f}, {Y:.3f}, {Z:.3f})',
            throttle_duration_sec=1.0
        )

        # ── Step 4: transform camera_optical_frame → base via TF ────
        point_cam = PointStamped()
        point_cam.header.frame_id = 'camera_optical_frame'
        point_cam.header.stamp = self.get_clock().now().to_msg()
        point_cam.point.x = X
        point_cam.point.y = Y
        point_cam.point.z = Z

        try:
            transform = self.tf_buffer.lookup_transform(
                'base',
                'camera_optical_frame',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            point_base = do_transform_point(point_cam, transform)

        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=3.0)
            return

        # ── Step 5: publish as PoseStamped ───────────────────────────
        pose = PoseStamped()
        pose.header.frame_id = 'base'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point_base.point.x
        pose.pose.position.y = point_base.point.y
        pose.pose.position.z = point_base.point.z
        # Orientation not meaningful for a point detection — identity quaternion
        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)

        self.get_logger().info(
            f'Object in base frame: '
            f'x={point_base.point.x:.3f}m  '
            f'y={point_base.point.y:.3f}m  '
            f'z={point_base.point.z:.3f}m',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
