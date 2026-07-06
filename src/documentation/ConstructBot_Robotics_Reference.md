# ConstructBot — Robotics Engineering Reference
*Personal knowledge base: Isaac Sim 4.2 + ROS2 Humble + Vision-Guided Pick and Place*
*Last updated: June 2026*

---

## Table of Contents
1. [Forward & Inverse Kinematics — Concepts](#1-forward--inverse-kinematics--concepts)
2. [How Industry Robots Actually Work](#2-how-industry-robots-actually-work)
3. [ConstructBot Arm — Session Fixes & Lessons](#3-constructbot-arm--session-fixes--lessons)
4. [Vision-Guided Pick and Place — Full Pipeline](#4-vision-guided-pick-and-place--full-pipeline)
5. [TF Tree — What It Is and How to Use It](#5-tf-tree--what-it-is-and-how-to-use-it)
6. [Isaac Sim — Hard-Won Lessons](#6-isaac-sim--hard-won-lessons)
7. [Next Steps Checklist](#7-next-steps-checklist)

---

## 1. Forward & Inverse Kinematics — Concepts

### Forward Kinematics (FK)

**Definition:** Given the current joint angles of a robot arm, compute the pose (position + orientation) of the end effector relative to the arm's base.

**In plain English:** You know where every joint is pointed right now — FK tells you where the tool at the tip ended up in space.

**Key property:** Always solvable. Always has exactly one answer. Just matrix multiplication through the kinematic chain.

**In ROS2/TF:** FK is computed automatically and continuously by the TF tree. Every time a joint moves, Isaac Sim (or a real robot driver) publishes updated `/joint_states`, and TF propagates that through the URDF's kinematic chain to update every frame's pose. You never compute FK manually.

**How to query FK for ConstructBot:**
```bash
# Live end effector pose relative to arm base, updated in real time
ros2 run tf2_ros tf2_echo base suction
```

**Output interpretation (from our session):**
```
- Translation: [0.243, 0.421, 0.220]   # XYZ in meters from base origin
- Rotation RPY (degree) [90.0, 0.026, 60.0]  # roll/pitch/yaw of suction cup
- Matrix: 4x4 homogeneous transform    # full SE(3) transform
```

**The 4x4 homogeneous transform matrix:**
```
[ R  | t ]     R = 3x3 rotation matrix
[ 0  | 1 ]     t = 3x1 translation vector
```
This single matrix encodes both where the end effector is (t) and how it's oriented (R).

---

### Inverse Kinematics (IK)

**Definition:** Given a desired pose (position + orientation) for the end effector, compute the joint angles needed to achieve it.

**In plain English:** You know where you want the tool to go — IK figures out what angles to set every joint to get there.

**Key properties:**
- May have multiple solutions (e.g. elbow-up vs elbow-down)
- May have no solution (target outside the arm's reachable workspace)
- Computationally harder than FK — this is what solvers like KDL, TRAC-IK, and BioIK handle

**In MoveIt2:** You specify a target `PoseStamped` (position + orientation), MoveIt2 runs IK internally, gets joint angles, plans a collision-free trajectory, and executes it. You never call IK directly.

**Example MoveIt2 IK call (Python):**
```python
from geometry_msgs.msg import PoseStamped
import moveit_commander

move_group = moveit_commander.MoveGroupCommander("arm")  # your planning group name

target_pose = PoseStamped()
target_pose.header.frame_id = "base"
target_pose.pose.position.x = 0.30
target_pose.pose.position.y = 0.0
target_pose.pose.position.z = 0.25
target_pose.pose.orientation.x = 1.0   # pointing straight down
target_pose.pose.orientation.y = 0.0
target_pose.pose.orientation.z = 0.0
target_pose.pose.orientation.w = 0.0

move_group.set_pose_target(target_pose)
move_group.go(wait=True)
```

---

### Do you need DH Parameters in industry?

**Almost never from scratch.** Here's the real answer:

| Situation | Do you need DH? |
|---|---|
| Commercial robot (UR, KUKA, Fanuc, ABB) with ROS2 driver | No — URDF already provided by manufacturer |
| Commercial robot, no ROS support, PLC-based | No — robot controller handles IK internally, you send Cartesian targets |
| Custom robot from scratch (like ConstructBot) | Yes — you define the URDF yourself |
| Academic research on kinematics algorithms | Yes |
| Verifying/debugging a manufacturer's model | Sometimes |

**DH (Denavit-Hartenberg) parameters** are just a systematic way to describe the geometry of a kinematic chain (joint axis directions, link lengths, offsets) so a computer can build the FK transform matrices. When you write a URDF, you're essentially providing this information in a more human-readable format. The URDF IS the DH table, just in a different form.

**Bottom line:** If the robot has a URDF and a ROS2 driver, you're done. You work in MoveIt2, not in math.

---

## 2. How Industry Robots Actually Work

### Day-One Workflow at a Company

**Step 1 — Find the ROS2 driver package**
```bash
# Examples
ros2 pkg list | grep kuka       # KUKA
ros2 pkg list | grep ur         # Universal Robots
ros2 pkg list | grep fanuc      # Fanuc
```
These packages include: URDF, MoveIt2 config, joint state publisher, hardware interface.

**Step 2 — Launch the robot**
```bash
# Example for Universal Robots
ros2 launch ur_robot_driver ur10e.launch.py robot_ip:=192.168.1.100
```
This gives you: `/joint_states` publishing, FK via TF, MoveIt2 ready to accept IK goals.

**Step 3 — Verify in RViz2**
Open RViz2, add RobotModel + TF displays, confirm the digital model matches the physical robot. If they match, your FK is correct.

**Step 4 — MoveIt2 motion planning**
```bash
ros2 launch <robot>_moveit_config moveit.launch.py
```
Now you can call `move_group.set_pose_target()` and the arm moves.

**Step 5 — Add your application layer**
Vision, inspection logic, PLC interface — everything on top of the working robot driver.

---

### Camera + Arm + PLC Inspection System

**Architecture:**
```
Camera → Object Detection/Inspection Node → ROS2 Topic (pass/fail + pose)
                                                    ↓
                                          PLC Bridge Node
                                                    ↓
                                          PLC I/O (conveyor, alarm, logger)
                                          
Camera → Pose Estimation → MoveIt2 → Robot moves to inspection pose
```

**PLC integration options:**
- `pycomm3` — for Allen Bradley PLCs (EtherNet/IP)
- `python-snap7` — for Siemens PLCs (S7 protocol)
- `ros2-industrial` — generic industrial robot/PLC bridge
- OPC-UA via `asyncua` Python library — vendor-neutral, increasingly common

**Key insight:** The PLC doesn't do vision math. ROS does vision and decision-making; PLC handles hard real-time I/O (conveyor belts, reject gates, safety interlocks). They communicate via simple signals: "part is good/bad", "move to position X", "cycle complete".

---

### Hand-Eye Calibration (Camera + Arm)

This is the critical step that connects vision to motion. Without it, you know where the object is in the camera's frame, but not in the robot's frame.

**Two configurations:**

1. **Eye-in-hand (camera on end effector):** Camera moves with the arm. Calibration computes transform from camera to flange/suction.
2. **Eye-to-hand (camera fixed in scene):** Camera is static. Calibration computes transform from camera to robot base.

**Calibration process (using `easy_handeye` ROS package):**
1. Mount a calibration target (ArUco marker or checkerboard) in the scene
2. Move the arm to ~15–20 different poses
3. At each pose, record: (a) the arm's end-effector pose from FK, (b) the target's pose from the camera
4. The algorithm solves for the unknown camera-to-robot transform
5. Result: a fixed transform you publish as a static TF frame

**After calibration:** Any object the camera detects can be transformed to robot base frame with one TF lookup. The arm can then go directly to it.

---

## 3. ConstructBot Arm — Session Fixes & Lessons

### Problem 1: Arm floating at world origin

**Root cause:** No joint existed between `arm_mount_link` and `arm/base`. The arm chain (base→shoulder→elbow→wrist→suction) was internally connected but had nothing physically anchoring it to the chassis.

**Symptom:** `arm/base world pos = (0, 0, 0.07)` — identical to its local translate, ignoring parent transforms entirely.

**Fix:** Created a `FixedJoint` between `arm_mount_link` (Body0) and `arm/base` (Body1).

**Critical gotcha:** The UI's auto-pivot calculation was corrupted by `resetXformStack=True` on `arm/base`. The UI computed `localPos0 = (0.60393, -0.7600, -0.04)` — the negative of the chassis world position. This is the same `resetXformStack` bug that broke the Python pivot math earlier.

**Fix for the corrupted pivot:** Manually set `localPos0` to the known correct value from documented geometry:
```python
joint_prim = stage.GetPrimAtPath("/World/constructbot/arm_mount_link/arm/base/FixedJoint")
joint_prim.GetAttribute("physics:localPos0").Set(Gf.Vec3f(0.0, 0.0, 0.07))
```

**MUST do while Stopped** — PhysX rejects joint frame edits during simulation:
```
Warning: Updating joint local poses in articulations is not supported after simulation start
```

---

### Problem 2: V-shape / wrong arm geometry (shoulder/elbow/wrist)

**Root cause:** Same `resetXformStack` issue corrupted the UI-computed pivot offsets for all three revolute joints. Stored `localPos0` values were wrong:

| Joint | Stored (wrong) | Should be |
|---|---|---|
| shoulder_joint | (0, 0, 0.18) | (0, 0, 0.25) |
| elbow_joint | (0.15, 0, -0.1) | (0.15, 0, 0.15) |
| wrist_joint | (0.1, 0, -0.15) | (0.25, 0, 0) |

**Fix:** Manually set all three while Stopped:
```python
sj = stage.GetPrimAtPath("/World/constructbot/arm_mount_link/arm/base/shoulder/shoulder_joint")
sj.GetAttribute("physics:localPos0").Set(Gf.Vec3f(0.0, 0.0, 0.25))

ej = stage.GetPrimAtPath("/World/constructbot/arm_mount_link/arm/base/shoulder/elbow/elbow_joint")
ej.GetAttribute("physics:localPos0").Set(Gf.Vec3f(0.15, 0.0, 0.15))

wj = stage.GetPrimAtPath("/World/constructbot/arm_mount_link/arm/base/shoulder/elbow/wrist/wrist_joint")
wj.GetAttribute("physics:localPos0").Set(Gf.Vec3f(0.25, 0.0, 0.0))
```

---

### Problem 3: suction_joint x-axis sign flip

**Root cause:** Same UI pivot corruption. `localPos0 = (-0.13, 0, -0.055)` instead of `(0.12, 0, -0.055)`.

**Fix:**
```python
sj_path = "/World/constructbot/arm_mount_link/arm/base/shoulder/elbow/wrist/suction/suction_joint"
prim = stage.GetPrimAtPath(sj_path)
prim.GetAttribute("physics:localPos0").Set(Gf.Vec3f(0.12, 0.0, -0.055))
```

---

### Problem 4: Drive targets not responding

**Root cause (initial suspicion):** No `ArticulationRootAPI` on `arm_mount_link` or `arm/base`. Found at `/World/constructbot/base_link` — correct location.

**Actual cause:** First test was done too quickly after setting the target — PhysX position drives ease toward target over time (stiffness 5000, damping 200), they don't teleport. The joint was actually moving but slowly. Confirmed by testing elbow (more time elapsed) and seeing it move correctly.

**Lesson:** Always wait 1–2 seconds after setting a drive target before judging whether it worked. Fast joints need higher stiffness or lower damping.

---

### The resetXformStack Rule (CRITICAL — document this permanently)

`resetXformStack=True` is required on nested rigid bodies for PhysX articulation physics to work correctly. BUT it has two major side effects:

1. **`UsdGeom.XformCache().GetLocalToWorldTransform()` returns wrong values** — it returns raw local transform as if the prim has no parent, because `resetXformStack` tells USD "start fresh here, ignore ancestors."

2. **Isaac Sim's UI joint creation tool gets corrupted pivot values** — because the UI tool uses the same XformCache under the hood to auto-compute joint frames. The UI tool is NOT immune to this bug.

**Consequence:** Never trust auto-computed pivot values when `resetXformStack=True` is present on either joint body. Always set `localPos0`/`localPos1` manually using your documented geometry table.

**`localPos1` is almost always `(0, 0, 0)`** — this is the child body's own attach frame at its local origin. Only `localPos0` (the parent body's attach frame) needs to be set to the actual offset.

---

### Joint Diagnostic Script (use every session to verify arm state)

```python
from pxr import UsdGeom, UsdPhysics, Gf

stage = omni.usd.get_context().get_stage()

# Check world positions of all arm links
links = ["base", "base/shoulder", "base/shoulder/elbow",
         "base/shoulder/elbow/wrist", "base/shoulder/elbow/wrist/suction"]
prefix = "/World/constructbot/arm_mount_link/arm/"

print("=== ARM LINK WORLD POSITIONS ===")
for link in links:
    prim = stage.GetPrimAtPath(prefix + link)
    if not prim.IsValid():
        print(f"{link}: INVALID")
        continue
    xform = UsdGeom.Xformable(prim)
    pos = xform.ComputeLocalToWorldTransform(0).ExtractTranslation()
    print(f"{link}: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")

# Check joint offsets
print("\n=== JOINT localPos0 VALUES ===")
joints = {
    "shoulder_joint": "/World/constructbot/arm_mount_link/arm/base/shoulder/shoulder_joint",
    "elbow_joint": "/World/constructbot/arm_mount_link/arm/base/shoulder/elbow/elbow_joint",
    "wrist_joint": "/World/constructbot/arm_mount_link/arm/base/shoulder/elbow/wrist/wrist_joint",
    "suction_joint": "/World/constructbot/arm_mount_link/arm/base/shoulder/elbow/wrist/suction/suction_joint",
    "FixedJoint": "/World/constructbot/arm_mount_link/arm/base/FixedJoint",
}
expected = {
    "shoulder_joint": (0, 0, 0.25),
    "elbow_joint": (0.15, 0, 0.15),
    "wrist_joint": (0.25, 0, 0),
    "suction_joint": (0.12, 0, -0.055),
    "FixedJoint": (0, 0, 0.07),
}
for name, path in joints.items():
    prim = stage.GetPrimAtPath(path)
    val = prim.GetAttribute("physics:localPos0").Get()
    exp = expected[name]
    ok = all(abs(val[i] - exp[i]) < 0.01 for i in range(3))
    print(f"  {name}: {tuple(round(v,3) for v in val)}  {'✅' if ok else '❌ expected ' + str(exp)}")
```

---

### Drive Test Script

```python
# Run while in Play mode
# Tests all 3 revolute joints by commanding nonzero targets

from pxr import UsdPhysics
import time

stage = omni.usd.get_context().get_stage()

joints = {
    "shoulder": "/World/constructbot/arm_mount_link/arm/base/shoulder/shoulder_joint",
    "elbow":    "/World/constructbot/arm_mount_link/arm/base/shoulder/elbow/elbow_joint",
    "wrist":    "/World/constructbot/arm_mount_link/arm/base/shoulder/elbow/wrist/wrist_joint",
}

def set_joint_target(name, degrees):
    prim = stage.GetPrimAtPath(joints[name])
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    drive.GetTargetPositionAttr().Set(degrees)
    print(f"Set {name} → {degrees}°")

def reset_all():
    for name in joints:
        set_joint_target(name, 0.0)
    print("Reset all joints to 0°")

# Test
set_joint_target("shoulder", 45.0)
set_joint_target("elbow", 30.0)
set_joint_target("wrist", -20.0)
```

---

## 4. Vision-Guided Pick and Place — Full Pipeline

### Mental Model

```
┌─────────────────────────────────────────────────────────┐
│  WHAT WE KNOW          │  HOW WE KNOW IT                │
├────────────────────────┼────────────────────────────────┤
│  End effector pose now │  FK via TF tree (automatic)    │
│  Object position (3D)  │  YOLO + depth + camera intrinsics │
│  Object in robot frame │  TF transform chain            │
│  Target orientation    │  Task knowledge (hardcoded)    │
│  Joint angles to go    │  IK via MoveIt2 (automatic)    │
└─────────────────────────────────────────────────────────┘
```

---

### Stage 1: Object Detection (2D)

**What it does:** Finds the object in the camera image, gives you pixel coordinates (u, v) of its center.

**Tool:** YOLOv8 node subscribed to `/camera/image_raw`

**Output:** Bounding box center pixel `(u, v)`, confidence score, class label

**Known issue for ConstructBot:** COCO-pretrained YOLOv8 misclassifies Isaac Sim's untextured primitives (capsule/cone/cube/cylinder) — sim-to-real domain gap. Fix options:
- Replace primitives with realistic textured USD assets
- Fine-tune YOLO on Isaac Sim synthetic renders
- Use Isaac Sim's built-in semantic segmentation ground truth instead

---

### Stage 2: 3D Pose Estimation (2D pixel → 3D world point)

**What it does:** Converts the 2D pixel detection into a real 3D position.

**Requires:** Depth image from `/camera/depth` (not yet wired — next step to add to master publisher)

**Math (camera backprojection):**
```python
# From camera_info topic:
fx = camera_info.k[0]   # focal length x
fy = camera_info.k[4]   # focal length y
cx = camera_info.k[2]   # principal point x
cy = camera_info.k[5]   # principal point y

# From depth image at detected pixel:
D = depth_image[v, u]   # depth in meters

# 3D point in camera_optical_frame:
X = (u - cx) * D / fx
Y = (v - cy) * D / fy
Z = D
```

**Then transform to robot base frame using TF:**
```python
point_in_camera = PointStamped()
point_in_camera.header.frame_id = "camera_optical_frame"
point_in_camera.point.x = X
point_in_camera.point.y = Y
point_in_camera.point.z = Z

point_in_base = tf_buffer.transform(point_in_camera, "base")
# Now point_in_base.point.x/y/z is object position in arm base frame
```

**The full transform chain TF walks for you:**
```
camera_optical_frame → arch_crossbar → top_plate_link → base_link → arm_mount_link → base
```
TF does all of this in one `lookup_transform` call. You don't compute any of the intermediate steps.

---

### Stage 3: Grasp Pose Computation

**For flat surface suction pick (your case):**

```python
from geometry_msgs.msg import PoseStamped

grasp_pose = PoseStamped()
grasp_pose.header.frame_id = "base"

# Position: above the object
grasp_pose.pose.position.x = object_in_base.point.x
grasp_pose.pose.position.y = object_in_base.point.y
grasp_pose.pose.position.z = object_in_base.point.z + 0.05  # 5cm above

# Orientation: suction cup pointing straight down
# In 'base' frame, "pointing down" = rotating 180° around X axis
grasp_pose.pose.orientation.x = 1.0
grasp_pose.pose.orientation.y = 0.0
grasp_pose.pose.orientation.z = 0.0
grasp_pose.pose.orientation.w = 0.0
```

**Why hardcoded orientation works:** For flat-surface pick-and-place, the approach direction is always the same (straight down). The object's own orientation on the table doesn't matter for a suction cup — suction doesn't need rotational alignment, only positional. This is standard practice in industry for bin-picking flat objects.

**For 6-DOF grasping (future):** You'd use the object's full pose (including orientation) from a 6-DOF pose estimator like FoundationPose, MegaPose, or DOPE, and compute the grasp orientation from the object's geometry and surface normals.

---

### Stage 4: MoveIt2 Motion Planning

**What MoveIt2 does:**
1. Receives your target `PoseStamped`
2. Runs IK to find joint angles
3. Plans a collision-free joint trajectory (series of waypoints)
4. Publishes trajectory on `/joint_trajectory_controller/joint_trajectory`
5. Your arm controller node reads this and executes it

**Setup requirements for ConstructBot:**
- SRDF file defining planning group `arm` (joints: shoulder, elbow, wrist)
- KDL kinematics plugin (default, works for 3-DOF)
- `moveit_config` package with launch files

---

### Stage 5: Arm Joint Controller (ROS2 → Isaac Sim)

**The missing bridge:** MoveIt2 outputs `JointTrajectory` messages. Isaac Sim accepts drive targets via Python `DriveAPI`. This node connects them.

**Node structure:**
```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import omni  # runs inside Isaac Sim's embedded Python

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.sub = self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )
        self.joint_paths = {
            'shoulder_joint': '/World/constructbot/arm_mount_link/arm/base/shoulder/shoulder_joint',
            'elbow_joint':    '/World/constructbot/arm_mount_link/arm/base/shoulder/elbow/elbow_joint',
            'wrist_joint':    '/World/constructbot/arm_mount_link/arm/base/shoulder/elbow/wrist/wrist_joint',
        }

    def trajectory_callback(self, msg):
        from pxr import UsdPhysics
        import math
        stage = omni.usd.get_context().get_stage()

        for i, joint_name in enumerate(msg.joint_names):
            if joint_name in self.joint_paths:
                prim = stage.GetPrimAtPath(self.joint_paths[joint_name])
                drive = UsdPhysics.DriveAPI.Get(prim, "angular")
                # JointTrajectory positions are in radians, DriveAPI wants degrees
                target_deg = math.degrees(msg.points[-1].positions[i])
                drive.GetTargetPositionAttr().Set(target_deg)
```

---

### Stage 6: Pick and Place State Machine

```
States:
  IDLE
    ↓ (object detected + user command)
  DETECT → get object pose in base frame
    ↓
  PRE_GRASP → MoveIt2: move to 10cm above object
    ↓
  GRASP → MoveIt2: lower to object surface
    ↓
  PICK → activate suction (Isaac Sim contact API)
    ↓
  LIFT → MoveIt2: move up
    ↓
  TRANSPORT → Nav2: drive to placement zone (future)
    ↓
  PLACE → MoveIt2: lower to placement surface
    ↓
  RELEASE → deactivate suction
    ↓
  RETURN → MoveIt2: return arm to home pose
    ↓
  IDLE
```

---

## 5. TF Tree — What It Is and How to Use It

### What TF Does

TF (Transform Framework) maintains a database of coordinate frame relationships over time. Every time any part of the robot moves, its frame's relationship to its parent is updated. TF lets any node ask: "where is frame A relative to frame B right now?"

**ConstructBot's TF tree (confirmed working):**
```
world
└── constructbot
    └── base_link (ArticulationRoot)
        ├── base_footprint
        ├── front_left_wheel
        │   └── left_encoder_link
        ├── front_right_wheel
        │   └── right_encoder_link
        ├── rear_left_wheel
        ├── rear_right_wheel
        ├── imu_link
        └── top_plate_link
            ├── arch_left_post
            │   └── arch_crossbar
            │       ├── camera_link
            │       │   └── camera_optical_frame
            │       └── lidar_link
            ├── arch_right_post
            └── arm_mount_link
                └── base (arm root)
                    └── shoulder
                        └── elbow
                            └── wrist
                                └── suction
```

### Key TF Commands

```bash
# View complete tree as PDF
ros2 run tf2_tools view_frames

# Live transform between any two frames
ros2 run tf2_ros tf2_echo <parent_frame> <child_frame>

# Check if a frame exists
ros2 topic echo /tf --once | grep child_frame_id

# All frames currently broadcasting
ros2 run tf2_ros tf2_monitor
```

### TF in Python

```python
import rclpy
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped

rclpy.init()
node = rclpy.create_node('tf_user')
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)

# Wait for TF to populate
rclpy.spin_once(node, timeout_sec=2.0)

# Get transform between any two frames
transform = tf_buffer.lookup_transform(
    'base',                  # target frame
    'camera_optical_frame',  # source frame
    rclpy.time.Time()        # latest available
)

# Transform a point from camera frame to base frame
point_cam = PointStamped()
point_cam.header.frame_id = 'camera_optical_frame'
point_cam.point.x = 0.1
point_cam.point.y = 0.0
point_cam.point.z = 0.5

point_base = tf_buffer.transform(point_cam, 'base')
```

---

### Why All Frames Showed parent='world' Initially

Isaac Sim's `ROS2PublishTransformTree` node publishes TWO sets of transforms for each prim:
1. The proper parent-child relationship (e.g. `arm_mount_link → base`)
2. A redundant world-relative absolute transform (e.g. `world → base`)

Both sets appear in `/tf`. TF2 handles this gracefully — it just has two paths to compute any transform and uses whichever is available. Not a bug, just redundancy. RViz2 may sometimes warn about this but it doesn't affect functionality.

---

## 6. Isaac Sim — Hard-Won Lessons

### Run Order (non-negotiable)

```
1. Set env vars in PowerShell BEFORE launching Isaac Sim:
   $env:FASTRTPS_DEFAULT_PROFILES_FILE = "C:\Users\snehs\.ros\fastdds.xml"
   $env:RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"

2. Launch Isaac Sim

3. Load USD stage: Mobile_robot_with_sensors.usd

4. RTX - Real-Time mode (NOT PathTracing, NOT Interactive — sensors don't render)

5. Press PLAY

6. Wait 3 seconds

7. Run constructbot_master_publisher_FINAL.py EXACTLY ONCE in Script Editor

8. In WSL2: ros2 topic list → should show all 9 topics
```

### Master Publisher Script Location

Save at: `C:\Users\snehs\OneDrive\Desktop\constructbot_master_publisher_FINAL.py`

(Keep alongside `Mobile_robot_with_sensors.usd` on Desktop)

### The One-Run-Per-Session Rule

**Never run the master publisher script twice in one Play session.** It deletes and rebuilds all 5 OmniGraphs. Running it twice creates duplicate nodes. If something breaks: Stop → Play → wait 3 sec → run once.

### Physics Joint Edits: Stopped Only

You CANNOT edit `physics:localPos0`, `physics:localPos1` on articulation joints while simulation is running:
```
Warning: Updating joint local poses in articulations is not supported after simulation start
```
Always Stop first, make joint frame edits, then Play again.

### rep.create.render_product() is BROKEN in Isaac Sim 4.2

Never use:
```python
render_product = rep.create.render_product(camera_prim, resolution)  # BROKEN
```

Always use `IsaacCreateRenderProduct` as an **OmniGraph node** inside the graph:
```python
og.Controller.Keys.CREATE_NODES: [
    ("CreateRenderProd", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
]
```

### FastDDS Config: All Three IPs Required

`~/.ros/fastdds.xml` must include all three addresses or local ROS2 nodes can't see each other:
```xml
<udpv4 address="127.0.0.1"/>        <!-- localhost -->
<udpv4 address="172.27.112.1"/>     <!-- Windows host IP -->
<udpv4 address="172.27.123.2"/>     <!-- WSL2 IP (verify each session) -->
```

Verify WSL2 IP each session: `ip addr show eth0`

### Current ALL_ROBOT_PRIMS List (23 frames, arm included)

```python
ALL_ROBOT_PRIMS = [
    "/World/constructbot",
    "/World/constructbot/base_link",
    "/World/constructbot/base_footprint",
    "/World/constructbot/imu_link",
    "/World/constructbot/top_plate_link",
    "/World/constructbot/arch_left_post",
    "/World/constructbot/arch_right_post",
    "/World/constructbot/arch_crossbar",
    "/World/constructbot/arm_mount_link",
    "/World/constructbot/lidar_link",
    "/World/constructbot/camera_link",
    "/World/constructbot/camera_optical_frame",
    "/World/constructbot/front_left_wheel",
    "/World/constructbot/front_right_wheel",
    "/World/constructbot/rear_left_wheel",
    "/World/constructbot/rear_right_wheel",
    "/World/constructbot/left_encoder_link",
    "/World/constructbot/right_encoder_link",
    "/World/constructbot/arm_mount_link/arm/base",
    "/World/constructbot/arm_mount_link/arm/base/shoulder",
    "/World/constructbot/arm_mount_link/arm/base/shoulder/elbow",
    "/World/constructbot/arm_mount_link/arm/base/shoulder/elbow/wrist",
    "/World/constructbot/arm_mount_link/arm/base/shoulder/elbow/wrist/suction",
]
```

### Arm Geometry Reference

| Link | Local offset from parent | Joint | Axis | Limits |
|---|---|---|---|---|
| base | (0, 0, 0.07) from arm_mount_link | FixedJoint | — | — |
| shoulder | (0, 0, 0.25) from base | shoulder_joint | Z (yaw) | ±180° |
| elbow | (0.15, 0, 0.15) from shoulder | elbow_joint | Y (pitch) | 0–150° |
| wrist | (0.25, 0, 0) from elbow | wrist_joint | Y (pitch) | ±90° |
| suction | (0.12, 0, -0.055) from wrist | suction_joint | Fixed | — |

Suction has an additional `xformOp:rotateXYZ = (90, 0, 0)` — 90° pre-rotation baked in.

### Key Prim Paths

```
Robot root:     /World/constructbot
Base link:      /World/constructbot/base_link
Arm mount:      /World/constructbot/arm_mount_link
Arm base:       /World/constructbot/arm_mount_link/arm/base
Shoulder joint: /World/constructbot/arm_mount_link/arm/base/shoulder/shoulder_joint
Elbow joint:    /World/constructbot/arm_mount_link/arm/base/shoulder/elbow/elbow_joint
Wrist joint:    /World/constructbot/arm_mount_link/arm/base/shoulder/elbow/wrist/wrist_joint
Suction joint:  /World/constructbot/arm_mount_link/arm/base/shoulder/elbow/wrist/suction/suction_joint
Fixed joint:    /World/constructbot/arm_mount_link/arm/base/FixedJoint
arm_mount_joint:/World/constructbot/top_plate_link/arm_mount_joint
LiDAR:          /World/constructbot/lidar_link/Rotating
Camera:         /World/constructbot/camera_link/Camera
IMU:            /World/constructbot/imu_link/Imu_Sensor
```

---

## 7. Next Steps Checklist

### Immediate (unblocked now)

- [ ] **Wire depth camera** — add `/camera/depth` to master publisher script
  - Same pattern as RGB camera in CameraGraph
  - Change `type: "rgb"` to `type: "depth"` in a second `ROS2CameraHelper` node
  - Needed for Stage 2 (3D pose estimation)

- [ ] **ROS2 arm joint controller node** — bridge MoveIt2 → Isaac Sim DriveAPI
  - Subscribes to `/arm_controller/joint_trajectory` (`JointTrajectory` msg)
  - Writes `targetPosition` to DriveAPI for each joint
  - Converts radians (ROS2 convention) to degrees (Isaac Sim DriveAPI convention)

### After those two

- [ ] **MoveIt2 setup** — SRDF + kinematics config for 3-joint planning group
- [ ] **Fix YOLO accuracy** — swap primitives for labeled USD assets, or use Isaac semantic segmentation
- [ ] **3D pose estimation node** — camera backprojection + TF transform to base frame
- [ ] **Pick and place state machine** — orchestrate the full pipeline
- [ ] **SLAM re-verification** — was working but fragile; revisit once arm work stable
- [ ] **Eye-in-hand camera mount** — add camera to suction link with known offset
- [ ] **Nav2 integration** — for full navigate → detect → pick → place cycle

### Verified Working (don't re-debug these)

- [x] FastDDS unicast bridging Windows ↔ WSL2
- [x] All 9 sensor topics publishing (clock, odom, tf, scan, point_cloud, image_raw, camera_info, imu)
- [x] Full TF tree: 23 frames including all 5 arm links
- [x] 3-DOF arm articulation with working position drives (shoulder ±180°, elbow 0–150°, wrist ±90°)
- [x] Correct arm geometry (all joint pivot offsets verified against documented values)
- [x] FK confirmed: `ros2 run tf2_ros tf2_echo base suction` gives live transform at 60hz

---

## 8. MoveIt2 Setup — Lessons & Current Config

### What Was Built

A `constructbot_moveit_config` ROS2 package at `~/isaac_mobile_robot/src/constructbot_moveit_config/` containing:

- `config/constructbot.srdf` — planning group definition
- `config/kinematics.yaml` — KDL solver config
- `config/joint_limits.yaml` — velocity limits
- `config/controllers.yaml` — controller definition (not yet wired to execution)
- `launch/moveit.launch.py` — launches move_group + RViz2

### Hard-Won MoveIt2 Lessons

**KDL cannot solve IK when the tip link is connected by a fixed joint.**
Even if the SRDF says `tip_link: suction` and `suction` is in the kinematic chain, KDL will always return error `-31` (NO_IK_SOLUTION) because there are zero DOF at a fixed joint — KDL has nothing to vary. Always set tip_link to the last *revolute* joint's child link. For ConstructBot: `tip_link: wrist` (not `suction`).

**Don't run robot_state_publisher alongside Isaac Sim.**
Isaac Sim's `ROS2PublishTransformTree` already publishes all TF frames. Adding `robot_state_publisher` creates a second publisher for the same frames, causing `TF_OLD_DATA` floods because the two publishers use different time references (sim-time vs wall-time). Only use `robot_state_publisher` when NOT using Isaac Sim's TF publisher.

**Isaac Sim does not publish `/joint_states`.**
MoveIt2's KDL solver needs `/joint_states` to know the current robot configuration (seed state for IK). Isaac Sim only publishes `/tf`. Fix: run the `tf_to_joint_states.py` bridge node which reads arm joint angles from TF transforms and republishes them as `/joint_states`.

**KDL default timeout is too short for some poses.**
Default `kinematics_solver_timeout: 0.05` causes `-31` failures for poses that ARE reachable but take longer to solve. Increase to `3.0` seconds for reliable results.

**SRDF group definition — use `<chain>` not `<joint>` list.**
Listing joints individually doesn't give KDL enough context about the chain direction. Use:
```xml
<group name="arm">
  <chain base_link="base" tip_link="wrist"/>
</group>
```

### Current Working Config

**constructbot.srdf:**
```xml
<?xml version="1.0" ?>
<robot name="constructbot">
  <group name="arm">
    <chain base_link="base" tip_link="wrist"/>
  </group>
  <group name="gripper">
    <link name="suction"/>
  </group>
  <end_effector name="suction_end_effector" parent_link="wrist" group="gripper"/>
  <group_state name="home" group="arm">
    <joint name="shoulder_joint" value="0"/>
    <joint name="elbow_joint" value="0"/>
    <joint name="wrist_joint" value="0"/>
  </group_state>
  <group_state name="ready" group="arm">
    <joint name="shoulder_joint" value="0"/>
    <joint name="elbow_joint" value="0.785"/>
    <joint name="wrist_joint" value="-0.785"/>
  </group_state>
  <disable_collisions link1="base" link2="shoulder" reason="Adjacent"/>
  <disable_collisions link1="shoulder" link2="elbow" reason="Adjacent"/>
  <disable_collisions link1="elbow" link2="wrist" reason="Adjacent"/>
  <disable_collisions link1="wrist" link2="suction" reason="Adjacent"/>
  <disable_collisions link1="base" link2="arm_mount_link" reason="Adjacent"/>
  <disable_collisions link1="arm_mount_link" link2="top_plate_link" reason="Adjacent"/>
</robot>
```

**kinematics.yaml:**
```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 3.0
  kinematics_solver_attempts: 10
  tip_link: wrist
  root_link: base
```

### How to Launch MoveIt2 (Every Session)

Run in this order, each in a separate terminal:

```bash
# Terminal 1 — Isaac Sim must already be in Play with master publisher running

# Terminal 2 — MoveIt2
cd ~/isaac_mobile_robot && source install/setup.bash
ros2 launch constructbot_moveit_config moveit.launch.py

# Terminal 3 — Joint states bridge (Isaac Sim TF → /joint_states for MoveIt2)
cd ~/isaac_mobile_robot && source install/setup.bash
python3 src/constructbot_description/scripts/tf_to_joint_states.py
```

### Confirmed Reachable Wrist Poses (in `base` frame)

| Wrist target | Joint angles | Notes |
|---|---|---|
| (0.40, 0.0, 0.40) | sh=0° el=0° wr=0° | Home pose, arm fully extended |
| (0.30, 0.0, 0.20) | sh=0° el=53° wr=-53° | Pick pose, arm bent down |

Workspace is limited by: elbow `0–150°` (one direction only), wrist `±90°`, shoulder `±180°`. KDL IK solver used (CHOMP planning pipeline).

### FK Results at Home Position (all joints = 0°)

```
shoulder: (0.000, 0.000, 0.250)   from base
elbow:    (0.150, 0.000, 0.400)   from base
wrist:    (0.400, 0.000, 0.400)   from base
suction:  (0.520, 0.000, 0.345)   from base
```

### TF to Joint States Bridge

File: `~/isaac_mobile_robot/src/constructbot_description/scripts/tf_to_joint_states.py`

Reads arm joint angles from TF (`base→shoulder`, `shoulder→elbow`, `elbow→wrist`) and publishes to `/joint_states` at 20hz. Required for MoveIt2 IK to work since Isaac Sim doesn't publish `/joint_states`.

---

## 9. Updated Next Steps Checklist

### Verified Working ✅
- FastDDS unicast bridging Windows ↔ WSL2
- All 9 sensor topics (clock, odom, tf, scan, point_cloud, image_raw, depth, camera_info, imu)
- Full TF tree: 23 frames including all 5 arm links
- 3-DOF arm articulation with working position drives
- Correct arm geometry (all joint pivot offsets verified)
- FK confirmed via `tf2_echo base suction` at 60hz
- Depth camera: `/camera/depth` at 1280×720 32FC1
- Arm joint controller: WSL2 publishes → Isaac Sim DriveAPI executes
- MoveIt2: IK working for reachable poses, FK verified

### Immediate Next Steps
- [ ] Fix odom flood — add static TF `odom → world` to silence RViz/MoveIt2 warnings
- [ ] 3D pose estimation node — YOLO + depth + camera intrinsics → object XYZ in `base` frame
- [ ] Pick and place state machine — orchestrate detect → IK → execute → pick → place

### Later
- [ ] Fix YOLO accuracy — use Isaac Sim semantic segmentation instead of COCO weights
- [ ] Eye-in-hand camera mount on suction link
- [ ] Nav2 integration for full autonomous cycle
- [ ] SLAM re-verification
