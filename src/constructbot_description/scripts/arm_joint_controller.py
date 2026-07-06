#!/usr/bin/env python3
"""
ConstructBot Arm Joint Controller - WSL2 Side
=============================================
Subscribes to MoveIt2's JointTrajectory output.
Extracts final joint angles (last waypoint).
Publishes simplified Float64MultiArray to Isaac Sim.

Topic in:  /arm_controller/joint_trajectory  (JointTrajectory)
Topic out: /arm_joint_targets               (Float64MultiArray)

Joint order in published array: [shoulder, elbow, wrist] in DEGREES
(Isaac Sim DriveAPI uses degrees, ROS2 uses radians — conversion here)
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray
import math


class ArmJointController(Node):

    # Map ROS2 joint names to array indices
    JOINT_ORDER = ['shoulder_joint', 'elbow_joint', 'wrist_joint']

    def __init__(self):
        super().__init__('arm_joint_controller')

        self.sub = self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )

        self.pub = self.create_publisher(
            Float64MultiArray,
            '/arm_joint_targets',
            10
        )

        # Current targets — start at home position (all zeros)
        self.current_targets = [0.0, 0.0, 0.0]  # degrees: shoulder, elbow, wrist

        # Publish home position immediately on startup
        self.publish_targets(self.current_targets)

        self.get_logger().info('Arm joint controller ready.')
        self.get_logger().info('Waiting for /arm_controller/joint_trajectory...')

    def trajectory_callback(self, msg: JointTrajectory):
        """
        Receives JointTrajectory from MoveIt2.
        Takes the LAST waypoint (final target position).
        Converts radians → degrees.
        Publishes to /arm_joint_targets.
        """
        if not msg.points:
            self.get_logger().warn('Received empty JointTrajectory, ignoring.')
            return

        # Take the last waypoint — that's the final target pose
        final_point = msg.points[-1]

        if len(final_point.positions) != len(msg.joint_names):
            self.get_logger().warn('Joint names and positions length mismatch, ignoring.')
            return

        # Build name→position map from the message
        joint_positions = dict(zip(msg.joint_names, final_point.positions))

        # Extract in our fixed order, convert radians to degrees
        targets_deg = []
        for joint_name in self.JOINT_ORDER:
            if joint_name in joint_positions:
                deg = math.degrees(joint_positions[joint_name])
                targets_deg.append(deg)
            else:
                # Joint not in this message — keep current value
                idx = self.JOINT_ORDER.index(joint_name)
                targets_deg.append(self.current_targets[idx])
                self.get_logger().warn(
                    f'{joint_name} not in trajectory message, keeping current: '
                    f'{self.current_targets[idx]:.1f} deg'
                )

        self.current_targets = targets_deg
        self.publish_targets(targets_deg)

        self.get_logger().info(
            f'Joint targets → shoulder: {targets_deg[0]:.1f}°  '
            f'elbow: {targets_deg[1]:.1f}°  '
            f'wrist: {targets_deg[2]:.1f}°'
        )

    def publish_targets(self, targets_deg):
        msg = Float64MultiArray()
        msg.data = targets_deg
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmJointController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
