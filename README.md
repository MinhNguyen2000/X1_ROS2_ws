# X1 ROS2 Workspace

This repository contains the full software stack for the **ROSMaster X1** mobile robot, integrating real-time face detection, deep reinforcement learning (DRL) navigation, sensor fusion, and hardware bringup within a **ROS2 Humble** framework.

The robot is controlled by a **Jetson Nano 4GB** (running Ubuntu 22.04 inside a Docker container) and an **STM32 board** as the low-level hardware microcontroller. All development and deployment is done inside Docker containers — one on the development machine and one on the robot side.

---

## System Architecture

High-level overview of the components in the X1_ROS2_ws workspace (as of **July 2025**):

![Current components/functionalities in the X1_ROS2_ws workspace](https://github.com/MinhNguyen2000/X1_ROS2_ws/blob/main/images/X1_ROS2_ws_system.png)

---

## Hardware & Infrastructure

| Component | Details |
|---|---|
| Compute | Jetson Nano 4GB |
| Low-level MCU | STM32 (connected via USB → symlink `/dev/myserial`) |
| LiDAR | YDLiDAR (connected via USB → symlink `/dev/ydlidar`) |
| Camera | Orbbec RGB-D Camera |
| Drive | DC wheel drive with encoder feedback |
| OS | Ubuntu 18.04 host → Ubuntu 22.04 Docker container |

### LiDAR Scan Properties (real hardware)
```
angle_min:       -π  (-3.1416 rad)
angle_max:        π  ( 3.1416 rad)
angle_increment:  0.01853 rad (~340 rays per scan)
range_min:        0.10 m
range_max:        8.00 m
```

> The physical LiDAR reports ~340 points rather than the 360 used during training. The **min-pooling strategy** (grouping rays into 18 sectors) handles this discrepancy gracefully.

---

## Package Overview

### `x1_bringup`

Handles real robot hardware bringup — sensors, low-level hardware, and odometry fusion.

**Launch files:**
- `robot_launch.py` — Main bringup launch. Starts:
  - YDLiDAR driver (`ydlidar_ros2_driver`)
  - Orbbec RGB-D camera (`orbbec_camera` composable node)
  - Low-level driver (`Mcnamu_driver`) for IMU, wheel encoders, and motor actuation
  - Odometry pipeline via `odom_launch.py`
- `odom_launch.py` — Launches the full odometry stack:
  - `rf2o_laser_odometry` — LiDAR scan matching for odometry
  - `covariance_filter_node` — Applies covariance matrices for EKF inputs; gates RF2O reset artifacts with a displacement threshold (>0.1 m) and a 3-second startup delay
  - `ekf_node` (from `robot_localization`) — Fuses wheel odometry, LiDAR odometry, and IMU data. Remaps `odometry/filtered` → `/odom`
- `visualize.launch.py` — Launched on a workstation in the same ROS network to remotely visualize real robot sensor data

**Executables:**
- `Mcnamu_driver.py` — Low-level hardware node. Subscribes to `/cmd_vel` (`TwistStamped`); publishes `/imu/data_raw`, `/imu/mag`, `/vel_raw`, `joint_states`, `voltage`, `edition`

**EKF Sensor Fusion Rules:**
- Wheel odometry → fuse `vx`, `vy`, and `vyaw`
- LiDAR odometry (RF2O) → fuse `x`, `y`, `yaw` with `odom1_differential: true`
- IMU → fuse `vyaw` and `ay`

---

### `x1_description`

Contains URDF and XACRO files defining the robot's physical structure and Gazebo simulation model.

**Launch files:**
- `display.launch.py` — Visualize robot structure in RViz
- `launch_sim.launch.py` — Full Gazebo simulation with differential drive controller, simulated LiDAR and camera sensors, SLAM via `slam_toolbox`, Nav2 costmap, and `twist_mux` for joystick/Nav2 priority blending
- `online_async_launch.py` — Launch `slam_toolbox` for online asynchronous SLAM or localization

---

### `x1_control`

Handles robot teleoperation via joystick or keyboard.

**Executables:**
- `x1_joy.py` — Maps joystick axes/buttons to velocity commands, buzzer, and RGB light control

**Launch files:**
- `x1_joy.launch.py` — Joystick control for Gazebo simulation
- `x1_keyboard.launch.py` — Keyboard-based velocity control

---

### `x1_drl_interfaces`

Defines the custom ROS2 action interface used by the DRL navigation stack.

**Action: `NavigateToGoal`**
- **Goal:** `target_pose` (`PoseStamped`, in the fixed `odom` frame), `goal_tolerance` (`float32`, meters)
- **Result:** `success` (`bool`), `message` (`string`), `total_distance` (`float32`, meters)
- **Feedback:** `distance_to_goal` (`float32`), `elapsed_time` (`float32`), `current_pose` (`Pose`)

---

### `x1_drl_policy`

Contains the DRL inference node and face-following coordinator.

**Executables:**
- `policy_node.py` — Action server (`navigate_to_goal`). Subscribes to `/odom` and `/scan`, publishes to `/cmd_vel`. Runs a 50 Hz control loop with goal tolerance checking, obstacle abort, and timeout handling. Uses `MultiThreadedExecutor` + `ReentrantCallbackGroup`.
- `face_follower_node.py` — Subscribes to `/face_pose` (`PoseStamped` in `camera_color_optical_frame`); transforms to the `odom` frame via TF2; projects onto the ground plane; sends `NavigateToGoal` action requests to `policy_node`. Implements goal debouncing by minimum displacement (default 0.20 m) and minimum time between goals (default 1.0 s).

#### Policy Architecture

The TD3 policy was trained in **MuJoCo** using **Stable Baselines3**, then exported for runtime deployment by saving actor weights as plain `.pt` PyTorch files and normalization statistics as `.npz` files — eliminating the SB3 runtime dependency entirely.

**Observation space (27-dimensional):**

| Index | Component | Description |
|---|---|---|
| 0–2 | `dx`, `dy`, `dgoal` | Displacement and distance to goal in the fixed `odom` frame |
| 3–4 | `cos θ`, `sin θ` | Robot heading in the `odom` frame |
| 5–6 | `cos φ`, `sin φ` | Relative bearing from robot heading to goal direction |
| 7–8 | `vx`, `vyaw` | Linear and angular velocities in the moving robot frame |
| 9–26 | LiDAR sectors | 18 min-pooled LiDAR groups covering 360° |

**Action space (2-dimensional):** `[vx ∈ [0, 1], vyaw ∈ [-1, 1]]` — scaled at runtime by `max_lin_vel` and `max_angular_vel`.

**LiDAR processing:**
- Raw scan is re-indexed from `(-π, π)` to `(0, 2π)` by rolling by the zero-angle index
- The array is flipped (`np.flip`) to correct for the clockwise scan direction mismatch between MuJoCo (clockwise) and ROS `LaserScan` (counter-clockwise)
- WiFi antenna interference (readings ≤ 0.20 m) is replaced with `range_max` to preserve sector integrity
- Rays are min-pooled into 18 sectors

**Policies are stored at:**
```
x1_drl_policy/policies/<MODEL_NAME>/
├── actor_weights.pt
└── norm_stats.npz
```

---

### `x1_visual`

Handles real-time face detection and 3D face pose estimation.

**Executables:**
- `face_detection_node.py` — Runs a YOLOv5 face detection model via **ONNXRuntime** with **TensorRT** execution provider (fp16, engine cache enabled). For each frame:
  1. Preprocesses to (1, 3, H, W) float32
  2. Runs ONNX inference
  3. Decodes bounding boxes with NMS
  4. Selects primary face by a weighted score of bounding box area and confidence
  5. Estimates 3D face position using the **monocular pinhole camera model** and an assumed face width (0.14 m)
  6. Applies exponential smoothing (`α = 0.9`) to the estimated pose
  7. Publishes: `/face_crop` (`Image`), `/face_detection` (`Detection2DArray`), `/face_pose` (`PoseStamped` in `camera_color_optical_frame`)

**Models stored at:**
```
x1_visual/models/face_detection/<MODEL_NAME>.onnx
```

---

### External / Cloned Packages

| Package | Purpose |
|---|---|
| `orbbec_camera` | Driver for Orbbec RGB-D camera |
| `ydlidar_ros2_driver` | Driver for YDLiDAR sensor |
| `rf2o_laser_odometry` | LiDAR scan matching odometry |
| `rosmaster_driver_install` | Low-level serial interface to STM32 (IMU, encoders, motors) |
| `robot_localization` | EKF sensor fusion (`ekf_node`) |

---

## DRL Training (Simulation Side)

Training is performed on the development machine using MuJoCo + Stable Baselines3 TD3. This was done in a different project => [ROS_DRL_Navigation](https://github.com/MinhNguyen2000/ROS2_DRL_Navigation)

**Key files:**
- `nav2d.py` — Gymnasium environment wrapping MuJoCo. Defines the full observation space, action space, reward shaping, curriculum learning hooks, and episode termination conditions.
- `model_creation.py` — `MakeEnv` class that programmatically constructs the MuJoCo `MjSpec` with arena walls, agent, simulated LiDAR (rangefinder sensors), goal, and randomized obstacles.
- `gym_TD3.py` — Training script using `SubprocVecEnv` for multi-core parallel rollouts. Implements a `CurriculumCallback` that advances agent and goal randomization bounds based on a rolling success rate window.

**Sim-to-real key considerations:**
- MuJoCo LiDAR scans are clockwise; ROS `LaserScan` is counter-clockwise — the raw array must be flipped before sector pooling
- The action space in MuJoCo required converting local frame velocities to global `(x, y)` slider velocities; the policy learns to output local frame commands
- The physical LiDAR has ~340 rays vs. 360 in simulation — handled by min-pooling into 18 fixed sectors

---

## Current Testing Pipeline (Real Robot)

Launch order on the robot:

```bash
# 1. Hardware bringup (LiDAR, camera, driver, EKF)
ros2 launch x1_bringup robot_launch.py

# 2. DRL policy action server
ros2 run x1_drl_policy drl_policy_node

# 3. Face detection and pose estimation
ros2 run x1_visual face_detection_node

# 4. Face follower (sends goals to policy node)
ros2 run x1_drl_policy face_follower_node
```

The face follower node transforms detected face poses from `camera_color_optical_frame` → `odom` frame via TF2, then dispatches `NavigateToGoal` action requests to the policy node. The policy runs at 50 Hz, reading from `/odom` and `/scan`, and publishing `TwistStamped` velocity commands to `/cmd_vel`.

---

## Known Issues & Remaining TODOs

### Active Issues

- **EKF yaw drift when stationary** — Caused by Gazebo IMU angular velocity bias. Mitigations:
  - Remove IMU `vyaw` from EKF fusion entirely when deploying on real hardware, or
  - Increase the IMU yaw covariance in the covariance filter node when the robot is stationary (wheel encoders report no change)

### TODOs

- [x] Validate and fix the DRL policy directional bias on the physical robot
- [ ] Tune EKF covariance parameters for real hardware (tighten LiDAR x/y covariance to `1e-4`)
- [ ] Resolve IMU angular velocity bias causing yaw drift when stationary
- [x] End-to-end validation of full face-following pipeline on physical robot
- [ ] Benchmark sim-to-real gap in LiDAR characteristics (ray count, range noise, reflectivity)
- [x] Physical camera fully integrated into the robot (mounting + calibration)
- [x] SLAM using `slam_toolbox` on the physical robot

---

## Repository Structure

```
X1_ROS2_ws/
├── src/
│   ├── x1_bringup/              # Hardware bringup launch files, EKF config, driver node
│   │   ├── launch/
│   │   │   ├── robot_launch.py
│   │   │   └── odom_launch.py
│   │   └── x1_bringup/
│   │       └── Mcnamu_driver.py
│   ├── x1_description/          # URDF, XACRO, Gazebo simulation
│   ├── x1_control/              # Joystick and keyboard teleoperation
│   ├── x1_drl_interfaces/       # NavigateToGoal action definition
│   ├── x1_drl_policy/           # DRL policy node and face follower
│   │   ├── x1_drl_policy/
│   │   │   ├── policy_node.py
│   │   │   └── face_follower_node.py
│   │   └── policies/
│   │       └── <MODEL_NAME>/
│   │           ├── actor_weights.pt
│   │           └── norm_stats.npz
│   ├── x1_visual/               # Face detection and pose estimation
│   │   ├── x1_visual/
│   │   │   └── face_detection_node.py
│   │   └── models/
│   │       └── face_detection/
│   │           └── <MODEL_NAME>.onnx
│   ├── orbbec_camera/           # Orbbec camera driver (cloned)
│   ├── ydlidar_ros2_driver/     # YDLiDAR driver (cloned)
│   ├── rf2o_laser_odometry/     # LiDAR scan matching (cloned)
│   └── rosmaster_driver_install/ # STM32 serial interface (cloned)
├── training/                    # DRL training scripts (dev machine only)
│   ├── nav2d.py
│   ├── model_creation.py
│   └── gym_TD3.py
└── images/
    └── X1_ROS2_ws_system.png
```
