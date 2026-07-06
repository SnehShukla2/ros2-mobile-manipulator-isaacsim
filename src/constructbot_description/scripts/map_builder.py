#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math

class MapBuilder(Node):
    def __init__(self):
        super().__init__("map_builder")
        self.resolution = 0.05
        self.map_size = 400
        self.grid = np.full((self.map_size, self.map_size), -1, dtype=np.int8)
        self.origin_x = -10.0
        self.origin_y = -10.0

        # Robot pose in the fixed world frame, kept up to date from /odom so
        # incoming points (given in the robot-local point cloud frame) can be
        # rotated/translated into that fixed frame before being written to the grid.
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        sensor_qos = QoSProfile(depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.map_pub = self.create_publisher(OccupancyGrid, "/my_map", 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, sensor_qos)
        self.create_subscription(PointCloud2, "/point_cloud", self.cloud_cb, sensor_qos)
        self.create_timer(1.0, self.publish_map) #2.0 instead on 1
        self.get_logger().info("Map Builder started")

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def cloud_cb(self, msg):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        if points.size == 0:
            return

        x = points["x"].astype(np.float64)
        y = points["y"].astype(np.float64)
        z = points["z"].astype(np.float64)

        height_mask = (z >= -0.1) & (z <= 1.5)
        x = x[height_mask]
        y = y[height_mask]

        # Rotate/translate from the robot-local point cloud frame into the
        # fixed world frame using the latest /odom pose, matching the approach
        # in custom_slam_node.py.
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)
        wx = self.robot_x + (x * cos_yaw - y * sin_yaw)
        wy = self.robot_y + (x * sin_yaw + y * cos_yaw)

        gx = ((wx - self.origin_x) / self.resolution).astype(np.int64)
        gy = ((wy - self.origin_y) / self.resolution).astype(np.int64)

        bounds_mask = (gx >= 0) & (gx < self.map_size) & (gy >= 0) & (gy < self.map_size)
        self.grid[gy[bounds_mask], gx[bounds_mask]] = 100

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = self.resolution
        msg.info.width = self.map_size
        msg.info.height = self.map_size
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.flatten().tolist()
        self.map_pub.publish(msg)
        self.get_logger().info("Map published", throttle_duration_sec=5.0)

def main():
    rclpy.init()
    node = MapBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
