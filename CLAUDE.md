# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`isaac_mobile_robot` is a ROS2 Humble colcon workspace for **ConstructBot**, a custom 4-wheel mobile
robot with a 3-DOF arm, simulated in **NVIDIA Isaac Sim 4.2 running on Windows**, bridged to
**ROS2 nodes running in WSL2** over FastDDS. There is no physical robot — Isaac Sim is the only
"driver." This is not a git repository.

The most valuable file in this repo is `src/documentation/ConstructBot_Robotics_Reference.md` — a
running log of hard-won lessons (TF quirks, Isaac Sim gotchas, MoveIt2 IK setup, joint pivot fixes).
**Read it before debugging anything related to the arm, TF, or the Isaac Sim bridge** — most
"weird" bugs in this system have already been diagnosed there.

## Cross-machine architecture (read this first)

Isaac Sim (Windows) and ROS2 (WSL2) are two separate DDS participants connected via FastDDS
unicast, configured in `~/.ros/fastdds.xml` with three addresses: localhost, the Windows host IP,
and the WSL2 IP (the WSL2 IP changes per session — verify with `ip addr show eth0`). Env vars
`FASTRTPS_DEFAULT_PROFILES_FILE` and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` must be set on the
Windows side before Isaac Sim launches.

Isaac Sim publishes `/tf`, `/odom`, `/scan`, `/camera/*`, `/imu` etc. via OmniGraph nodes and a
"master publisher" script run once per Play session inside Isaac Sim's Script Editor (not part of
this repo — lives on the Windows Desktop alongside the USD stage). **Isaac Sim does NOT publish
`/joint_states`** — only `/tf`. Consequently:
- Do not run `robot_state_publisher` alongside Isaac Sim — it double-publishes the same TF frames
  with a different time base and floods `TF_OLD_DATA` warnings. `robot_state_publisher` is only
  used in the pure-RViz display launch files (no Isaac Sim), not with the sim.
- `scripts/tf_to_joint_states.py` bridges TF → `/joint_states` (20 Hz) so MoveIt2's KDL solver has
  a seed state to work from.

Arm targets flow **out** of ROS2 into Isaac Sim in the opposite direction: MoveIt2 plans a
`JointTrajectory` → `scripts/arm_joint_controller.py` takes the last waypoint, converts
radians→degrees, republishes as `/arm_joint_targets` (`Float64MultiArray`) → a script inside Isaac
Sim writes that to each joint's PhysX `DriveAPI` target. Isaac Sim's position drives ease toward a
target over time (they don't teleport); wait 1-2s before judging whether a move worked.

## TF tree

```
world → constructbot → base_link (ArticulationRoot)
  ├── base_footprint, front/rear left/right wheels (+ encoder links), imu_link
  └── top_plate_link
        ├── arch_left_post / arch_right_post → arch_crossbar → camera_link → camera_optical_frame
        │                                                    → lidar_link
        └── arm_mount_link → base (arm root) → shoulder → elbow → wrist → suction
```
The arm chain (`base→shoulder→elbow→wrist→suction`) mirrors the URDF in
`src/constructbot_description/urdf/snehbot.urdf`. `shoulder_joint`/`elbow_joint`/`wrist_joint` are
the only revolute joints; everything else in the URDF is fixed. **MoveIt2's KDL solver cannot
solve IK with a fixed-joint tip link** — `tip_link` must be `wrist` (the last revolute joint's
child), not `suction`; see `config/kinematics.yaml` and `config/constructbot.srdf`.

## Packages (`src/`)

- **`constructbot_description`** (ament_python) — URDF (`urdf/snehbot.urdf`), RViz-only display
  launch files, SLAM param config, and the Python bridge/application scripts:
  - `scripts/tf_to_joint_states.py` — TF → `/joint_states` bridge (see above).
  - `scripts/arm_joint_controller.py` — MoveIt2 `JointTrajectory` → `/arm_joint_targets` for Isaac Sim.
  - `scripts/pose_estimator.py` — fake-detection pixel → depth backprojection → TF transform →
    `/detected_object_pose`. Camera intrinsics come from `/camera/camera_info`; depth is read
    directly from the raw bytes of a 32FC1 `/camera/depth` image.
  - `scripts/pick_place_state_machine.py` — orchestrates the pick/place cycle as an explicit state
    machine (`IDLE → VALIDATE → PLAN_PICK → MOVE_PICK → GRASP → LIFT_PLAN → LIFT_MOVE →
    PLAN_PLACE → MOVE_PLACE → RELEASE → HOME`), calling MoveIt2's `/compute_ik` service directly
    rather than going through `move_group`'s planning pipeline. Reachability is pre-checked with a
    brute-force 2-link FK sweep (`is_pose_reachable`) before calling IK.
  - `custom_slam_node.py` — a from-scratch occupancy-grid mapper (not slam_toolbox) that
    subscribes to `/odom` + `/scan` and publishes `/map` directly; used as an alternative/fallback
    to `slam_toolbox` (see `config/slam_toolbox_params.yaml` and root `my_slam_params.yaml`).
- **`constructbot_moveit_config`** (ament_cmake, config-only — no C++ built) — SRDF, kinematics,
  joint limits, and controller config for MoveIt2's 3-DOF `arm` planning group. No hardware
  interface plugin is wired up yet (`controllers.yaml` defines a `joint_trajectory_controller` but
  execution goes through the `arm_joint_controller.py` bridge above, not `ros2_control`).
- **`ros2_laser_scan_matcher`** (ament_cmake, C++) — scan-matching odometry node, ported from
  `scan_tools`'s `laser_scan_matcher`; publishes `odom→base_link` TF and optionally `/odom`. Links
  against `csm` (built as a sibling package — see below).
- **`csm`** — third-party C-based scan-matching library (`libcsm`), a build dependency of
  `ros2_laser_scan_matcher`. Vendored, not modified; don't "fix" its style/warnings.

## Build & test

Standard colcon workspace:
```bash
cd ~/isaac_mobile_robot
colcon build --symlink-install      # symlink-install so Python script edits take effect without rebuilding
source install/setup.bash
```
Build a single package: `colcon build --packages-select constructbot_description`.

`constructbot_description` and `constructbot_moveit_config`'s only tests are the ament boilerplate
(`test_copyright.py`, `test_flake8.py`, `test_pep257.py`) — there is no application-level test
suite. Run via colcon:
```bash
colcon test --packages-select constructbot_description
colcon test-result --verbose
```

## Running the system (every session, in order)

1. **Windows side**: set FastDDS env vars, launch Isaac Sim, load the USD stage, switch to
   **RTX - Real-Time** render mode (not PathTracing/Interactive — sensors won't render otherwise),
   press Play, wait ~3s, then run the master publisher script **exactly once** from the Script
   Editor (it rebuilds 5 OmniGraphs — running it twice creates duplicate nodes; if something
   breaks, Stop → Play → wait → rerun once).
2. **WSL2, terminal 1** — verify the bridge: `ros2 topic list` should show all sensor topics
   (`/tf`, `/odom`, `/scan`, `/camera/image_raw`, `/camera/depth`, `/camera/camera_info`, `/imu`, `/clock`).
3. **WSL2, terminal 2** — MoveIt2: `ros2 launch constructbot_moveit_config moveit.launch.py`
4. **WSL2, terminal 3** — joint-states bridge (required before MoveIt2 IK will work):
   `python3 src/constructbot_description/scripts/tf_to_joint_states.py`
5. Bring up whichever application nodes you need on top:
   `arm_joint_controller.py`, `pose_estimator.py`, `pick_place_state_machine.py`, or
   `custom_slam_node.py`.

For pure RViz visualization without Isaac Sim, use
`ros2 launch constructbot_description display.launch.py` (runs `robot_state_publisher` +
`joint_state_publisher` + static TF stand-ins + RViz2) or `rsp_sim.launch.py` (robot_state_publisher
only, `use_sim_time:=true`, meant to run alongside a sim clock source).

## Physics/joint editing constraints (Isaac Sim side)

Never edit `physics:localPos0`/`localPos1` on articulation joints while simulation is running —
PhysX silently rejects it. Always Stop, edit, then Play again. Also: `resetXformStack=True`
(required on nested rigid bodies for correct PhysX articulation behavior) breaks both
`UsdGeom.XformCache().GetLocalToWorldTransform()` and Isaac Sim's UI joint-creation auto-pivot
tool — never trust auto-computed joint pivots when it's present; set `localPos0` manually from the
documented geometry table in the reference doc.
