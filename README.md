# ConstructBot — ROS2 Mobile Manipulator in Isaac Sim

A simulated mobile manipulator platform built on **ROS2 Humble** and **NVIDIA Isaac Sim 4.2**:
a 4-wheel differential-drive base with a 3-DOF arm, running a full sense → plan → act pipeline —
LiDAR/vision-based mapping, camera-guided object detection, MoveIt2 motion planning, and a
pick-and-place task executor — entirely in simulation, with the simulator and the ROS2 stack
running on two different operating systems talking over the network in real time.

## Why this project is interesting

Most "ROS2 + robot arm" tutorials assume the simulator and the ROS2 graph live in the same Linux
process space. This project doesn't have that luxury: Isaac Sim only runs well on Windows/native
Linux with a GPU, while the ROS2 tooling (MoveIt2, slam_toolbox, tf2) is easiest to develop against
in a Linux environment. So the two halves of the system run on **separate machines** — Isaac Sim
on Windows, the full ROS2 graph in WSL2 — and are bridged live over DDS. Getting that bridge
reliable, and then building a working perception → planning → actuation loop on top of it, is the
core engineering problem this repo solves.

## System architecture

```
┌─────────────────────────────┐        FastDDS / RTPS         ┌──────────────────────────────────┐
│         Windows host        │      (unicast, 3 peers)       │              WSL2                │
│                              │ <────────────────────────────>│                                   │
│   Isaac Sim 4.2              │   /tf  /odom  /scan            │   ROS2 Humble node graph          │
│   - USD robot stage          │   /camera/image_raw            │   - MoveIt2 (motion planning)     │
│   - PhysX articulation       │   /camera/depth                │   - Custom SLAM / slam_toolbox     │
│   - OmniGraph ROS2 bridge    │   /camera/camera_info          │   - Pose estimation node           │
│     nodes ("master           │   /imu   /clock                │   - Pick-and-place state machine   │
│     publisher" script)       │ <────────────────────────────  │   - TF → joint_states bridge       │
│   - PhysX DriveAPI           │   /arm_joint_targets           │                                    │
│     (joint position drives)  │                                │                                    │
└─────────────────────────────┘                                └──────────────────────────────────┘
```

Isaac Sim publishes all sensor and TF data outward; ROS2 nodes in WSL2 consume it, plan against it,
and publish joint targets back, which a script inside Isaac Sim applies to each joint's PhysX drive.
Neither side ever touches the other's process — everything crosses the machine boundary as DDS
traffic over a FastDDS profile listing the loopback address, the Windows host IP, and the WSL2 IP
(the WSL2 side changes every session and has to be re-verified).

This unicast setup exists because WSL2's virtual network adapter doesn't reliably pass multicast
traffic between Windows and the WSL2 VM, so ROS2's default DDS discovery — which depends on
multicast — can't find nodes across that boundary. `fastdds.xml` works around this by explicitly
listing each side's IP as a unicast peer, so nodes connect directly instead of relying on multicast
broadcast. Both Windows and WSL2 need their own copy of this file with correct peer IPs pointing at
each other, and since the WSL2 IP changes every session, it has to be re-verified each time.

One deliberate design constraint this exposed: **Isaac Sim publishes `/tf` but never
`/joint_states`.** Since MoveIt2's IK solver needs a joint-state seed to work from, a small bridge
node recovers the arm's joint angles by walking the TF chain (`base→shoulder→shoulder→elbow→
elbow→wrist`) and republishing them as `/joint_states` at 20 Hz — turning a one-line ROS2
convenience most tutorials take for granted into an explicit piece of the pipeline.

## Pick-and-place pipeline

The task pipeline is a straight line from pixels to motion, with every stage as its own node:

1. **Pose estimation** — a detection pixel `(u, v)` (currently a fixed placeholder, standing in
   for a YOLO/segmentation front-end) is back-projected into 3D using the depth image and the
   camera's intrinsic matrix, then transformed from `camera_optical_frame` into the arm's `base`
   frame via a single `tf2` lookup. Output: `/detected_object_pose`.
2. **Reachability pre-check** — before ever calling IK, the target is checked against a
   brute-force sampled map of the arm's actual 2-link workspace, so unreachable detections are
   rejected cheaply instead of burning planning time.
3. **Motion planning** — MoveIt2's `/compute_ik` service (KDL plugin) solves for joint angles
   directly against the `arm` planning group (`shoulder → elbow → wrist`), sidestepping full
   trajectory planning for the fast reactive moves this task needs.
4. **Arm control** — resulting joint targets are converted from radians to degrees and published
   to `/arm_joint_targets`, which the Isaac Sim side applies to each joint's PhysX position drive.
5. **Task orchestration** — a pick-and-place state machine (`pick_place_state_machine.py`) drives
   the whole sequence — `IDLE → VALIDATE → PLAN_PICK → MOVE_PICK → GRASP → LIFT_PLAN → LIFT_MOVE
   → PLAN_PLACE → MOVE_PLACE → RELEASE → HOME` — polling joint-state feedback to confirm arrival
   at each stage before advancing, with per-state timeouts so the cycle can't hang indefinitely.

## Custom SLAM

Alongside the standard `slam_toolbox` mapping backend, this repo includes a mapping node written
from scratch (`custom_slam_node.py`): it subscribes directly to `/odom` and `/scan`, projects each
LiDAR return into the world frame using the robot's current pose estimate, and rasterizes hits
into an occupancy grid it publishes on `/map` — without leaning on any existing SLAM library for
the grid construction itself. It's a deliberately simpler algorithm than `slam_toolbox`'s
Cartographer-style pipeline (no loop closure, no pose-graph optimization), used both as a
from-first-principles exercise in how occupancy-grid mapping actually works and as a lightweight
fallback when the full `slam_toolbox` stack isn't needed.

Scan-matching odometry is provided separately by a ROS2 port of `scan_tools`'
`laser_scan_matcher`, built against a vendored copy of the `csm` (Canonical Scan Matcher) C library.

## Getting Started / How to Run

### Prerequisites

- Isaac Sim 4.2 installed on Windows (`C:\isaacsim_4_2`), with the ConstructBot USD stage loaded
- ROS2 Humble installed in WSL2, with this workspace built (`colcon build --symlink-install`)
- A `fastdds.xml` unicast profile present on **both** Windows (`C:\Users\snehs\.ros\fastdds.xml`)
  and WSL2 (`~/.ros/fastdds.xml`), each listing the other side's current IP as a peer (see
  [System architecture](#system-architecture) for why this is required)

### 1. Launch Isaac Sim (Windows)

Set the DDS environment variables once (PowerShell), then close and reopen PowerShell so they take
effect:

```powershell
[System.Environment]::SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", "C:\Users\snehs\.ros\fastdds.xml", "User")
[System.Environment]::SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", "User")
```

In the new PowerShell window, launch Isaac Sim:

```powershell
& "C:\isaacsim_4_2\isaac-sim.selector.bat"
```

### 2. Set up the WSL2 environment (once per terminal)

Every new WSL2 terminal used for this project needs:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
```

### 3. Start the simulation

In Isaac Sim, press **▶ Play**.

### 4. SLAM mapping workflow

1. **Fix Isaac Sim timestamps** (only needed once — persists if the USD stage is saved): in the
   Isaac Sim Script Editor, press **Stop**, then **Play** again.
2. **Run the map builder** (single WSL2 terminal):
   ```bash
   cd ~/isaac_mobile_robot && source install/setup.bash
   python3 src/constructbot_description/scripts/map_builder.py
   ```
3. **Open RViz2** to view the map:
   ```bash
   rviz2
   ```

### 5. Pick-and-place / object detection sequence

Computes the object's pose from base → object via inverse kinematics, then commands the arm to
grasp it. Run each of the following in its own WSL2 terminal, in order:

**Terminal 1 — MoveIt2:**
```bash
cd ~/isaac_mobile_robot && source install/setup.bash
ros2 launch constructbot_moveit_config moveit.launch.py
```

**Terminal 2 — TF → joint_states bridge:**
```bash
cd ~/isaac_mobile_robot && source install/setup.bash
python3 src/constructbot_description/scripts/tf_to_joint_states.py
```

**Terminal 3 — pose estimator:**
```bash
cd ~/isaac_mobile_robot && source install/setup.bash
python3 src/constructbot_description/scripts/pose_estimator.py
```

**Terminal 4 — verify:**
```bash
sleep 3 && ros2 topic list | grep detected
ros2 topic echo /detected_object_pose --once
```

## Current status

**Working end-to-end:**
- Cross-machine FastDDS bridge (Windows Isaac Sim ↔ WSL2 ROS2), all sensor topics live
- Full 23-frame TF tree, including the 5-link arm chain, verified against forward kinematics
- 3-DOF arm articulation with working PhysX position drives (shoulder ±180°, elbow 0–150°, wrist ±90°)
- MoveIt2 IK solving correctly for the arm's reachable workspace (KDL, `tip_link: wrist`)
- Depth-based 3D pose estimation from camera pixel → robot base frame
- Occupancy-grid mapping via both `slam_toolbox` and the custom mapping node

**Known issue — `MOVE_PICK` arm-arrival timeout:**
The pick-and-place state machine confirms arrival at each waypoint by comparing live
`/joint_states` feedback against the commanded target within a tolerance, but the `MOVE_PICK`
state occasionally exceeds its timeout window before the arm settles on target and the state
machine advances anyway rather than waiting or retrying. Root cause under investigation — the
Isaac Sim PhysX position drives ease toward a target over time rather than snapping to it, and the
current timeout/tolerance combination doesn't always give slower approach trajectories enough
margin. Next steps: tune per-joint drive stiffness/damping vs. the state timeout, and consider a
velocity-based settling check instead of a fixed wall-clock deadline.

**Not yet implemented:**
- Real object detection (pose estimation currently uses a fixed placeholder pixel, not a live
  vision model)
- Physical suction/gripper actuation (grasp/release are logged placeholders)
- Nav2 integration for combined navigate + pick + place cycles

## Repository layout

| Package | Description |
|---|---|
| `constructbot_description` | URDF, RViz display launch files, SLAM config, and all Python pipeline nodes (pose estimation, arm control, state machine, custom SLAM, TF→joint_states bridge) |
| `constructbot_moveit_config` | MoveIt2 SRDF, kinematics, joint limits, and controller config for the 3-DOF arm planning group |
| `ros2_laser_scan_matcher` | Scan-matching odometry node (C++), ROS2 port of `scan_tools` |
| `csm` | Vendored Canonical Scan Matcher C library, build dependency of the above |

## Tech stack

ROS2 Humble · NVIDIA Isaac Sim 4.2 · MoveIt2 (KDL) · slam_toolbox · tf2 · FastDDS · PhysX ·
Python & C++ · colcon
