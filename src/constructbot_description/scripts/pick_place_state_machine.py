#!/usr/bin/env python3
"""
ConstructBot Pick-and-Place State Machine
==========================================
Orchestrates: detect -> validate -> pick -> lift -> place -> release -> home
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from builtin_interfaces.msg import Duration
import math
import time


# ── Workspace geometry (confirmed via MoveIt2 IK testing) ──────────
L1 = 0.25
L2 = 0.25
SHOULDER_Z = 0.25

PLACE_POSE = (0.30, 0.0, 0.20)
HOME_JOINTS = (0.0, 0.0, 0.0)

JOINT_ARRIVE_TOLERANCE_DEG = 5.0
STATE_TIMEOUT_SEC = 8.0
LIFT_HEIGHT = 0.10


def is_pose_reachable(x, y, z):
    """Geometric pre-check against known FK-sampled workspace."""
    radius = math.sqrt(x * x + y * y)

    best_dist = float('inf')
    for e_deg in range(0, 151, 3):
        for w_deg in range(-90, 91, 3):
            e = math.radians(e_deg)
            w = math.radians(w_deg)
            fx = L1 * math.sin(e) + L2 * math.sin(e + w)
            fz = L1 * math.cos(e) + L2 * math.cos(e + w)
            dist = math.hypot(fx - radius, fz - z)
            if dist < best_dist:
                best_dist = dist

    return best_dist < 0.03, best_dist


class PickPlaceStateMachine(Node):
    def __init__(self):
        super().__init__('pick_place_state_machine')

        self.state = 'IDLE'
        self.state_entry_time = time.time()

        self.detected_pose = None
        self.pick_joint_targets = None
        self.lift_joint_targets = None
        self.place_joint_targets = None

        self.current_joint_state = {}

        self.create_subscription(PoseStamped, '/detected_object_pose',
                                 self.detection_cb, 10)
        self.create_subscription(JointState, '/joint_states',
                                 self.joint_state_cb, 10)

        self.arm_pub = self.create_publisher(Float64MultiArray,
                                             '/arm_joint_targets', 10)

        self.ik_helper_node = rclpy.create_node('ik_helper_' + str(id(self)))
        self.ik_client = self.ik_helper_node.create_client(GetPositionIK, '/compute_ik')

        self.create_timer(0.5, self.tick)

        self.get_logger().info('Pick-and-place state machine started.')
        self.get_logger().info('Waiting for MoveIt2 IK service...')
        self.ik_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('IK service ready. State: IDLE')

    def detection_cb(self, msg):
        self.detected_pose = msg

    def joint_state_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current_joint_state[name] = math.degrees(pos)

    def set_state(self, new_state):
        self.get_logger().info(f'[STATE] {self.state} -> {new_state}')
        self.state = new_state
        self.state_entry_time = time.time()

    def time_in_state(self):
        return time.time() - self.state_entry_time

    def compute_ik(self, x, y, z, desc=""):
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.avoid_collisions = False
        request.ik_request.ik_link_name = 'wrist'
        request.ik_request.timeout = Duration(sec=3, nanosec=0)

        target = PoseStamped()
        target.header.frame_id = 'base'
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        target.pose.orientation.w = 1.0
        request.ik_request.pose_stamped = target

        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self.ik_helper_node, future, timeout_sec=5.0)

        if not future.result():
            self.get_logger().warn(f'IK timeout for {desc}')
            return False, 0.0, 0.0, 0.0

        if future.result().error_code.val != 1:
            self.get_logger().warn(
                f'IK failed for {desc}: error {future.result().error_code.val}'
            )
            return False, 0.0, 0.0, 0.0

        sol = future.result().solution.joint_state
        angles = {}
        for name, pos in zip(sol.name, sol.position):
            if name in ['shoulder_joint', 'elbow_joint', 'wrist_joint']:
                angles[name] = math.degrees(pos)

        return (True,
                angles.get('shoulder_joint', 0.0),
                angles.get('elbow_joint', 0.0),
                angles.get('wrist_joint', 0.0))

    def publish_arm_targets(self, shoulder, elbow, wrist):
        msg = Float64MultiArray()
        msg.data = [shoulder, elbow, wrist]
        self.arm_pub.publish(msg)
        self.get_logger().info(
            f'Arm target sent: sh={shoulder:.1f} el={elbow:.1f} wr={wrist:.1f}'
        )

    def arm_has_arrived(self, target_shoulder, target_elbow, target_wrist):
        if not self.current_joint_state:
            return False
        cs = self.current_joint_state.get('shoulder_joint', 999)
        ce = self.current_joint_state.get('elbow_joint', 999)
        cw = self.current_joint_state.get('wrist_joint', 999)
        return (abs(cs - target_shoulder) < JOINT_ARRIVE_TOLERANCE_DEG and
                abs(ce - target_elbow) < JOINT_ARRIVE_TOLERANCE_DEG and
                abs(cw - target_wrist) < JOINT_ARRIVE_TOLERANCE_DEG)

    def tick(self):
        if self.state == 'IDLE':
            self.handle_idle()
        elif self.state == 'VALIDATE':
            self.handle_validate()
        elif self.state == 'PLAN_PICK':
            self.handle_plan_pick()
        elif self.state == 'MOVE_PICK':
            self.handle_move_pick()
        elif self.state == 'GRASP':
            self.handle_grasp()
        elif self.state == 'LIFT_PLAN':
            self.handle_lift_plan()
        elif self.state == 'LIFT_MOVE':
            self.handle_lift_move()
        elif self.state == 'PLAN_PLACE':
            self.handle_plan_place()
        elif self.state == 'MOVE_PLACE':
            self.handle_move_place()
        elif self.state == 'RELEASE':
            self.handle_release()
        elif self.state == 'HOME':
            self.handle_home()

    def handle_idle(self):
        if self.detected_pose is not None:
            self.set_state('VALIDATE')

    def handle_validate(self):
        p = self.detected_pose.pose.position
        reachable, dist = is_pose_reachable(p.x, p.y, p.z)

        if reachable:
            self.get_logger().info(
                f'Pose ({p.x:.2f},{p.y:.2f},{p.z:.2f}) is REACHABLE '
                f'(nearest valid point {dist*100:.1f}cm away)'
            )
            self.set_state('PLAN_PICK')
        else:
            self.get_logger().warn(
                f'Pose ({p.x:.2f},{p.y:.2f},{p.z:.2f}) is OUT OF WORKSPACE '
                f'(nearest valid point {dist*100:.1f}cm away) - skipping. '
                f'Move the object closer to the arm.'
            )
            self.detected_pose = None
            self.set_state('IDLE')

    def handle_plan_pick(self):
        p = self.detected_pose.pose.position
        success, sh, el, wr = self.compute_ik(p.x, p.y, p.z, desc="pick pose")

        if success:
            self.pick_joint_targets = (sh, el, wr)
            self.set_state('MOVE_PICK')
        else:
            self.get_logger().warn('IK failed despite passing pre-check - returning to IDLE')
            self.detected_pose = None
            self.set_state('IDLE')

    def handle_move_pick(self):
        if self.time_in_state() < 0.1:
            self.publish_arm_targets(*self.pick_joint_targets)

        if self.arm_has_arrived(*self.pick_joint_targets):
            self.set_state('GRASP')
        elif self.time_in_state() > STATE_TIMEOUT_SEC:
            self.get_logger().warn('Timed out moving to pick pose - proceeding anyway')
            self.set_state('GRASP')

    def handle_grasp(self):
        if self.time_in_state() < 0.1:
            self.get_logger().info('GRASP: suction activated (placeholder - no physical gripper yet)')
        if self.time_in_state() > 1.5:
            self.set_state('LIFT_PLAN')

    def handle_lift_plan(self):
        p = self.detected_pose.pose.position
        lift_z = p.z + LIFT_HEIGHT
        success, sh, el, wr = self.compute_ik(p.x, p.y, lift_z, desc="lift pose")

        if success:
            self.lift_joint_targets = (sh, el, wr)
            self.set_state('LIFT_MOVE')
        else:
            self.get_logger().warn('Lift IK failed - skipping lift, going straight to place')
            self.set_state('PLAN_PLACE')

    def handle_lift_move(self):
        if self.time_in_state() < 0.1:
            self.publish_arm_targets(*self.lift_joint_targets)

        if self.arm_has_arrived(*self.lift_joint_targets):
            self.set_state('PLAN_PLACE')
        elif self.time_in_state() > STATE_TIMEOUT_SEC:
            self.set_state('PLAN_PLACE')

    def handle_plan_place(self):
        x, y, z = PLACE_POSE
        success, sh, el, wr = self.compute_ik(x, y, z, desc="place pose")

        if success:
            self.place_joint_targets = (sh, el, wr)
            self.set_state('MOVE_PLACE')
        else:
            self.get_logger().error('Place IK failed - aborting cycle, returning HOME')
            self.set_state('HOME')

    def handle_move_place(self):
        if self.time_in_state() < 0.1:
            self.publish_arm_targets(*self.place_joint_targets)

        if self.arm_has_arrived(*self.place_joint_targets):
            self.set_state('RELEASE')
        elif self.time_in_state() > STATE_TIMEOUT_SEC:
            self.set_state('RELEASE')

    def handle_release(self):
        if self.time_in_state() < 0.1:
            self.get_logger().info('RELEASE: suction deactivated (placeholder)')
        if self.time_in_state() > 1.5:
            self.set_state('HOME')

    def handle_home(self):
        if self.time_in_state() < 0.1:
            self.publish_arm_targets(*HOME_JOINTS)

        if self.arm_has_arrived(*HOME_JOINTS):
            self.get_logger().info('Cycle complete. Returning to IDLE.')
            self.detected_pose = None
            self.pick_joint_targets = None
            self.lift_joint_targets = None
            self.place_joint_targets = None
            self.set_state('IDLE')
        elif self.time_in_state() > STATE_TIMEOUT_SEC:
            self.detected_pose = None
            self.set_state('IDLE')


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceStateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
