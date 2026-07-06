#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class MapBuilder(Node):
    def __init__(self):
        super().__init__("map_builder")
        self.resolution = 0.05
        self.map_size = 400
        self.grid = np.zeros((self.map_size, self.map_size), dtype=np.int8)
        self.origin_x = -10.0
        self.origin_y = -10.0
        sensor_qos = QoSProfile(depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.map_pub = self.create_publisher(OccupancyGrid, "/my_map", 10)
        self.create_subscription(PointCloud2, "/point_cloud", self.cloud_cb, sensor_qos)
        self.create_timer(1.0, self.publish_map) #2.0 instead on 1
        self.get_logger().info("Map Builder started")

    def cloud_cb(self, msg):
        for pt in pc2.read_points(msg, field_names=("x","y","z"), skip_nans=True):
            x, y, z = pt
            if z < -0.1 or z > 1.5:
                continue
            gx = int((x - self.origin_x) / self.resolution)
            gy = int((y - self.origin_y) / self.resolution)
            if 0 <= gx < self.map_size and 0 <= gy < self.map_size:
                self.grid[gy][gx] = 100

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
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
    rclpy.spin(MapBuilder())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
