#!/usr/bin/env python3
"""
Reads arm joint angles from TF tree and publishes as /joint_states.
Needed because Isaac Sim publishes TF but not joint_states.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import math

class TFToJointStates(Node):
    def __init__(self):
        super().__init__('tf_to_joint_states')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_joint_states)  # 20hz
        self.get_logger().info('TF to JointStates bridge started.')

    def get_rotation_z(self, transform):
        """Extract rotation around Z axis (yaw) from quaternion."""
        q = transform.rotation
        # yaw from quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_rotation_y(self, transform):
        """Extract rotation around Y axis (pitch) from quaternion."""
        q = transform.rotation
        sinp = 2 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        return math.asin(sinp)

    def publish_joint_states(self):
        try:
            # Get shoulder rotation (Z axis yaw)
            t_shoulder = self.tf_buffer.lookup_transform(
                'base', 'shoulder', rclpy.time.Time())
            shoulder_angle = self.get_rotation_z(t_shoulder.transform)

            # Get elbow rotation (Y axis pitch) relative to shoulder
            t_elbow = self.tf_buffer.lookup_transform(
                'shoulder', 'elbow', rclpy.time.Time())
            elbow_angle = self.get_rotation_y(t_elbow.transform)

            # Get wrist rotation (Y axis pitch) relative to elbow
            t_wrist = self.tf_buffer.lookup_transform(
                'elbow', 'wrist', rclpy.time.Time())
            wrist_angle = self.get_rotation_y(t_wrist.transform)

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = ['shoulder_joint', 'elbow_joint', 'wrist_joint']
            js.position = [shoulder_angle, elbow_angle, wrist_angle]
            js.velocity = [0.0, 0.0, 0.0]
            js.effort = [0.0, 0.0, 0.0]
            self.pub.publish(js)

        except Exception as e:
            pass  # TF not ready yet, skip this tick

def main(args=None):
    rclpy.init(args=args)
    node = TFToJointStates()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
