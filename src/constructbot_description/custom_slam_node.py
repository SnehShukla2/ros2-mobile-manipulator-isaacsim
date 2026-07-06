#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid

class RealCustomSlamNode(Node):
    def __init__(self):
        super().__init__("custom_slam_processor")
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.resolution = 0.05
        self.width = 400
        self.height = 400
        self.origin_x = -10.0
        self.origin_y = -10.0
        self.grid = [-1] * (self.width * self.height)

        # Telemetry counters to diagnose unicast flow issues live
        self.scan_count = 0
        self.odom_count = 0

        # Strict Matching QoS Policy for Isaac Sim FastDDS Bridge
        isaac_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Map topics typically require Transient Local Durability to persist in RViz
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile=isaac_qos)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, qos_profile=isaac_qos)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", qos_profile=map_qos)
        
        self.get_logger().info("================================================================")
        self.get_logger().info("CUSTOM SLAM: OccupancyGrid mapping node fully initialized!")
        self.get_logger().info("Waiting for data over Unicast pipeline...")
        self.get_logger().info("================================================================")

    def odom_callback(self, msg):
        self.odom_count += 1
        if self.odom_count % 20 == 0:
            self.get_logger().info(f"[Telemetry] Received {self.odom_count} /odom messages from Isaac Sim.")
            
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        
        # Guard against zero-division errors during initialization states
        try:
            self.robot_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        except Exception:
            self.robot_yaw = 0.0

    def scan_callback(self, msg):
        self.scan_count += 1
        if self.scan_count % 5 == 0:
            self.get_logger().info(f"[Telemetry] Processing /scan #{self.scan_count} -> Publishing updated /map")

        angle = msg.angle_min
        for r in msg.ranges:
            if r > msg.range_min and r < msg.range_max and not math.isnan(r) and not math.isinf(r):
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)
                wx = self.robot_x + (lx * math.cos(self.robot_yaw) - ly * math.sin(self.robot_yaw))
                wy = self.robot_y + (lx * math.sin(self.robot_yaw) + ly * math.cos(self.robot_yaw))
                mx = int((wx - self.origin_x) / self.resolution)
                my = int((wy - self.origin_y) / self.resolution)
                if 0 <= mx < self.width and 0 <= my < self.height:
                    self.grid[my * self.width + mx] = 100
            angle += msg.angle_increment

        try:
            map_msg = OccupancyGrid()
            map_msg.header.stamp = self.get_clock().now().to_msg()
            map_msg.header.frame_id = "map"
            map_msg.info.resolution = self.resolution
            map_msg.info.width = self.width
            map_msg.info.height = self.height
            map_msg.info.origin.position.x = self.origin_x
            map_msg.info.origin.position.y = self.origin_y
            map_msg.data = self.grid
            self.map_pub.publish(map_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish occupancy grid map: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = RealCustomSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Verify ROS context is still open before triggering generic shutdown sequence
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()