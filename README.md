# Aerial Robotics Sim ROS2

An end-to-end **ROS 2**, **PX4 Autopilot**, and **Gazebo** simulation workspace (`drone_ws`) built for developing, testing, and validating autonomous quadrotor perception, state estimation, and offboard flight control pipelines.

## Overview

This repository houses custom ROS 2 nodes, launch configurations, simulation assets, and utility scripts designed for Software-In-The-Loop (SITL) and Hardware-In-The-Loop (HITL) aerial robotics workflows. It bridges high-level computer vision and trajectory/attitude control algorithms in ROS 2 with PX4 flight dynamics via Micro XRCE-DDS, enabling rapid prototyping before physical deployment on Pixhawk-based UAV hardware.

### Core Capabilities
- **Offboard Flight Control:** Custom ROS 2 nodes for autonomous arming, mode switching, setpoint streaming, and closed-loop attitude/position control.
- **Vision & Perception:** Real-time camera stream processing, marker detection, and 3D relative pose estimation in simulated environments.
- **Simulation & Dynamics:** Gazebo world/vehicle configurations and control utilities for testing quadrotor flight behavior under repeatable conditions.

---

## Root Directory Scripts

The root directory houses standalone Python scripts used for vision diagnostics, hardware bring-up (Intel RealSense, Dynamixel, Arduino servos), trajectory simulation, and PX4 Gazebo asset generation.

### Vision & ArUco Diagnostics
- **`aruco_diagnostic.py`**
  * **Purpose:** Interactive ROS 2 diagnostic node (`aruco_diagnostic`) for calibrating ArUco marker dimensions and verifying 3D world-frame pose estimation during simulated PX4 flight.
  * **Details:** Subscribes to the simulated IMX214 RGB camera (`/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image`) and PX4 local odometry (`/fmu/out/vehicle_local_position`). Detects `DICT_6X6_250` markers and allows live toggling between four candidate marker physical sizes (`0.15m`, `0.5m`, `1.0m`, `1.5m`) via keyboard keys (`1`–`4`) to evaluate distance and world-position error against a target coordinate `[3.0, 3.0, 0.0]`.
- **`aruco_gz_sim_diagnostics.py`**
  * **Purpose:** Pre-flight verification node (`camera_diagnostics`) to validate the `ros_gz_bridge` image pipeline and `DICT_6X6_250` ArUco detection before running full autonomy stacks.
  * **Details:** Subscribes to `/realsense/color/image_raw`, overlays 3D coordinate axes (`cv2.drawFrameAxes`) and relative translation vectors (`X, Y, Z`), and logs frame rates and detection percentages every 2 seconds.
- **`generate_aruco.py`**
  * **Purpose:** Utility script to generate high-resolution `DICT_6X6_250` ArUco marker PNGs with a 20% white border and overwrite PX4's default Gazebo marker texture.
  * **Details:** Generates sample markers (`ID 0–4`) in `./aruco_test_markers/` and optionally backs up and replaces `$PX4_AUTOPILOT/Tools/simulation/gz/models/aruco_marker/materials/textures/aruco_marker_0.png`.

### RealSense & Hardware Bring-Up
- **`realsense_stream_test.py`**
  * **Purpose:** Minimal `pyrealsense2` sanity check to verify physical Intel RealSense depth streaming.
  * **Details:** Reads the depth frame at the center pixel (`width // 2, height // 2`) and prints the distance in meters to the console.
- **`realsense_cv_test.py`**
  * **Purpose:** Standalone OpenCV RGB streaming benchmark (`640x480 @ 30 FPS`) for the Intel RealSense camera with a live FPS counter overlay.
- **`realsense_aruco_detection.py`**
  * **Purpose:** Real-time ArUco (`DICT_4X4_50`) pose estimation and pitch-control prototyping script using live camera intrinsics from `pyrealsense2`.
  * **Details:** Computes relative marker coordinates `(x, y, z)`, calculates vertical angular error `θ = atan2(y, z)`, and outputs a PD pitch command (`kp = 0.15`, `kd = 0.15`) overlaid on the live RGB feed.

### Gimbal / Tracker & Servo Utilities
- **`aruco_tracker.py`**
  * **Purpose:** Single-axis (pan) visual servoing prototype for tracking a `DICT_4X4_50` ArUco marker using a webcam and an Arduino-controlled servo over serial (`/dev/ttyACM0`, `9600` baud).
  * **Details:** Computes horizontal pixel error from the frame center, applies a PID controller (`KP = 0.1`, `KI = 0.0`, `KD = 0.8`) with a 7-degree deadband, and streams target angles (`0°–180°`) to the microcontroller.
- **`test_servo.py`**
  * **Purpose:** Basic serial communication check (`9600` baud on `/dev/ttyACM0`) that sweeps an Arduino-connected hobby servo across `0°`, `180°`, and `90°`.
- **`dynamixel_servo_id_setter.py`**
  * **Purpose:** Interactive configuration tool using `dynamixel_sdk` (Protocol 2.0, `57600` baud on `/dev/ttyUSB0`) for setting up a 2-DOF pan-tilt Dynamixel gimbal.
  * **Details:** Scans IDs `0–10`, disables torque to reassign servo IDs (e.g., `ID 1` for pan, `ID 2` for tilt), and configures Operating Mode `3` (Position Control).
- **`two_axis_obj_det.py`**
  * **Purpose:** Real-time 2-DOF pan-tilt visual servoing controller for tracking a `DICT_4X4_50` ArUco marker (`0.05 m`) using a monocular webcam (`640x480`) and Dynamixel XL330 servos (`ID 1` pan, `ID 2` tilt) over `/dev/ttyUSB0` (`Protocol 2.0`, `57600` baud).
  * **Details:**
    * **Perception & Filtering:** Estimates 3D marker translation `(tx, ty, tz)`, smooths raw pose estimates via an `ExponentialFilter` (`alpha = 0.3`), and computes optical-axis angular errors `pan_error = atan2(tx, tz)` and `tilt_error = atan2(ty, tz)`.
    * **Dynamic Homing & Safety Bounds:** Captures live encoder ticks on boot as `PAN_CENTER` and `TILT_CENTER`, enforcing software travel limits of `±1024` ticks (`±90°`) around the initial pose.
    * **Trajectory & Velocity Control (`Operating Mode = 1`):** Generates `0.8s` S-curve setpoints via `CubicTrajectoryGenerator` when angular error exceeds `0.09 rad` (with a `0.5s` cooldown), tracking them with `TrajectoryController` (`kp = 0.2`, `kd = 0.035`, `velocity_gain = 0.8`) and proportional holding damping outside active trajectories. Converts `rad/s` commands to XL330 velocity register units (`0.229 RPM/unit`, clipped to `[-500, 500]`), while also retaining `PDController` and position-mode (`Operating Mode = 3`) interfaces in `DynamixelController`.

### Trajectory Planning & Control Verification
- **`trajectory_test.py`**
  * **Purpose:** Standalone mathematical simulation script (`numpy`, `matplotlib`) to verify minimum-jerk trajectory generation and closed-loop PD tracking before deploying in ROS 2 / Gazebo.
  * **Details:** Implements `QuinticPolynomial` (5th-order) and `CubicPolynomial` (3rd-order) trajectory generators, simulates a discrete-time 3D double-integrator plant under a PD control law (`a_cmd = a_r - Kd·ė - Kp·e`) with Gaussian disturbance noise, and exports verification plots (`trajectory_profiles.png`, `closed_loop_simulation.png`, `quintic_vs_cubic.png`).

---

## 2-DOF Pan-Tilt ArUco Visual Servoing

Real-time monocular visual servoing on a 2-DOF Dynamixel pan-tilt gimbal to track a `4×4` ArUco marker. This module implements two complete control architectures—starting with a filtered **Position-Mode PD Controller** and advancing to a **Minimum-Jerk Quintic Trajectory Tracker** with feedforward velocity and PD feedback derived from manipulator joint-space dynamics.

### Hardware & System Architecture

- **Actuators:** 2× Dynamixel **XL330-M077-T** (Model `1190`) in a 3-pin TTL daisy chain
  * **Pan Axis:** ID `1`
  * **Tilt Axis:** ID `2`
- **Interface & Power:** Robotis **U2D2** USB-to-TTL converter + U2D2 Power Hub powered by a **5V regulated supply** (`Protocol 2.0`, `57600` baud over `/dev/ttyUSB0`)
- **Vision Sensor:** Monocular USB camera (`640×480`) tracking a `50 mm` (`0.05 m`) marker from `cv2.aruco.DICT_4X4_50`

#### Visual Error Formulation
Using the marker's 3D translation vector **t** = `[tx, ty, tz]ᵀ` estimated via `cv2.aruco.estimatePoseSingleMarkers`, optical-axis alignment errors (in radians) are computed as:

```text
θ_pan  = atan2(tx, tz)
θ_tilt = atan2(ty, tz)
```

### Control Architectures

#### 1. Position-Control PD Tracker (`Operating Mode = 3`)
A `20 Hz` discrete position-control loop designed to eliminate high-frequency pose jitter and startup alignment offsets:
- **Dynamic Startup Homing:** Reads live encoder ticks on initialization as `PAN_CENTER` and `TILT_CENTER`, enforcing software travel limits of `±1024` ticks (`±90°`) around the initial pose regardless of horn mounting offset.
- **Commanded-Position Integration:** Tracks internal `commanded_pan` and `commanded_tilt` states in radians and maps deltas to encoder resolution (`4096 / 2π ≈ 651.9 ticks/rad`), avoiding readback jitter from polling `ADDR_PRESENT_POSITION` every cycle.
- **Two-Stage EMA Signal Conditioning:**
  1. **Pose Filtering (`α = 0.2`):** Smooths raw `tx, ty, tz` estimates prior to computing `atan2` angular errors.
  2. **Derivative Low-Pass Filtering (`α_d = 0.3`):** Filters the error signal prior to numerical differentiation (`ė`) to prevent derivative kick and noise amplification without introducing integral windup.

| Axis | `Kp` | `Kd` | Output Limit (`rad`) | Notes |
| :--- | :---: | :---: | :---: | :--- |
| **Pan (ID 1)** | `0.40` | `0.035` | `±0.40` | Symmetric horizontal tracking |
| **Tilt (ID 2)** | `0.40` | `0.035` | `±0.50` | Higher authority for gravity compensation |

#### 2. Quintic Trajectory & Feedforward-PD Velocity Controller (`Operating Mode = 1`)
To achieve smooth, jerk-free motion with zero velocity and zero acceleration at trajectory endpoints, the gimbal operates in **Velocity Control Mode** (`ADDR_GOAL_VELOCITY = 104`).

**Minimum-Jerk Quintic Polynomial:**
Enforcing six boundary conditions over duration `T` (`q(0) = q₀`, `q(T) = q_f`, `q̇(0) = q̇(T) = 0`, `q̈(0) = q̈(T) = 0`) yields a 5th-order polynomial in normalized time `τ = (t - t₀) / T ∈ [0, 1]` with `Δq = q_f - q₀`:

```text
q_d(τ) = q₀ + Δq · (10τ³ - 15τ⁴ + 6τ⁵)
q̇_d(τ) = (Δq / T)  · (30τ² - 60τ³ + 30τ⁴)
q̈_d(τ) = (Δq / T²) · (60τ  - 180τ² + 120τ³)
```

**Joint-Space Dynamics & Velocity Control Law:**
Starting from the 2-DOF rigid-body manipulator dynamics:

```text
M(q)q̈ + h(q, q̇) = τ
```

Computed-torque control specifies `τ = M(q)(q̈_d + Kp·e + Kd·ė) + h(q, q̇)`. Because the XL330's internal velocity loop regulates motor current and rejects low-speed inertial/gravity disturbances (`M(q) ≈ I`, `h(q, q̇)` compensated internally), the outer visual-servoing loop commands desired joint velocities directly via feedforward plus PD feedback:

```text
q̇_cmd = q̇_d + Kp · (q_d - q) + Kd · (q̇_d - q̇)
```

Velocity commands (`rad/s`) are converted to XL330 register units (`0.229 RPM/unit`) and saturated within safe bounds:

```text
u_vel = clip( int((q̇_cmd · (60 / 2π)) / 0.229), -500, 500 )
```

### Gimbal Sub-Module Structure

```text
├── two_axis_obj_det.py           # 2-DOF pan-tilt ArUco visual servoing controller (Velocity & Position modes)
├── dynamixel_servo_id_setter.py  # Interactive Dynamixel ID & operating mode configuration tool
└── README.md
```

### Setup & Usage (Pan-Tilt Gimbal)

#### Prerequisites
- Python 3.8+
- `dynamixel-sdk`
- `opencv-contrib-python`
- `numpy`

```bash
pip install dynamixel-sdk opencv-contrib-python numpy
```

#### Hardware Permissions (Linux)
Grant serial port access to the U2D2 interface before running:
```bash
sudo usermod -aG dialout $USER
# Or temporarily for the current session:
sudo chmod 666 /dev/ttyUSB0
```

#### Running the Tracker
1. Verify the 5V supply is connected to the U2D2 Power Hub and both XL330 servos (IDs `1` and `2`) are powered.
2. Print a `50 mm` `DICT_4X4_50` ArUco marker (e.g., Marker ID `0`).
3. Launch the 2-DOF pan-tilt tracker:

```bash
python3 two_axis_obj_det.py
```

Press `q` in the OpenCV preview window to safely zero velocities, disable torque, and close the serial port.

## ROS 2 Package: `fly`

The `fly` package is the core ROS 2 flight-control, perception, and autonomous mission package inside `drone_ws`[cite: 20, 21]. It bridges high-level computer vision, state-space optimal control, and polynomial trajectory algorithms with the **PX4 Autopilot** flight stack via Micro XRCE-DDS (`/fmu/in/*` and `/fmu/out/*` topics using `px4_msgs`), progressing from basic offboard heartbeats and keyboard teleoperation to LQR hover stabilization and closed-loop ArUco visual servoing[cite: 20, 21, 26, 27].

### 1. Foundational Offboard & Teleoperation Nodes
- **`offboard.py` (`offboard`)**[cite: 29]
  * **Purpose:** Minimal PX4 offboard heartbeat publisher for verifying Micro XRCE-DDS link connectivity[cite: 29].
  * **Details:** Publishes `OffboardControlMode` messages (enabling position and velocity control flags) to `fmu/in/offboard_control_mode` at `2 Hz` (`0.5 s` timer)[cite: 29].
- **`all_nodes.py` (`offboard_control_takeoff_and_land`)**[cite: 20]
  * **Purpose:** Basic single-setpoint offboard takeoff and auto-land flight lifecycle node[cite: 20].
  * **Details:** Configured with PX4-compatible QoS (`BEST_EFFORT`, `TRANSIENT_LOCAL`) at `10 Hz` to stream `OffboardControlMode` heartbeats and `TrajectorySetpoint` commands (`[10.0, 10.0, -5.0]` NED, `yaw = 1.57079 rad`), engage Offboard mode, arm the vehicle, and trigger `VEHICLE_CMD_NAV_LAND` once the `-5.0 m` target altitude is reached[cite: 20].
- **`mult_waypoints.py` (`offboard_control_takeoff_and_land`)**[cite: 28]
  * **Purpose:** Sequential multi-waypoint offboard navigation node[cite: 28].
  * **Details:** Guides the quadrotor across `[(2.0, 2.0, -2.0), (7.0, 5.0, -3.0), (10.0, 10.0, -5.0)]` in NED coordinates, advancing through waypoints using a `0.3 m` Euclidean proximity threshold before initiating landing[cite: 28].
- **`keyboard_control.py` (`keyboard_offboard_control`)**[cite: 26]
  * **Purpose:** Interactive terminal-based keyboard teleoperation node for manual offboard flight testing[cite: 26].
  * **Details:** Captures raw `termios`/`tty` keystrokes (`W/S` for vertical Z, `A/D` for lateral Y, arrow keys for forward/backward X in `0.5 m` steps, and `Space` to engage Offboard mode and toggle arm/disarm after 10 setpoints) while spinning ROS 2 in a background daemon thread at `10 Hz`[cite: 26].

### 2. Optimal Control & Vision Bring-Up Nodes
- **`lqr.py` (`lqr_hover_controller`)**[cite: 27]
  * **Purpose:** State-space Linear Quadratic Regulator (LQR) controller for hover stabilization[cite: 27].
  * **Details:** Models quadrotor translational dynamics as a 6-state double integrator (`[x, y, z, vx, vy, vz]`), solves the Continuous-Time Algebraic Riccati Equation (`scipy.linalg.solve_continuous_are`) with `Q = diag(10, 10, 15, 1, 1, 2)` and `R = I₃` (with a fallback PD gain matrix), and integrates optimal accelerations into velocity-bounded setpoints (`max_velocity = 2.0 m/s`) to stabilize hover at `[0.0, 0.0, -5.0]`[cite: 27].
- **`cam.py` (`image_subscriber`)**[cite: 22]
  * **Purpose:** Lightweight OpenCV perception test node for contour and centroid extraction on the simulated IMX214 camera feed (`/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image`)[cite: 22].
  * **Details:** Converts incoming frames to grayscale, applies a `3×3` Gaussian blur, runs Canny edge detection (`100, 200`) and binary thresholding, and draws image-moment centroids (`m10/m00`, `m01/m00`) on detected contours[cite: 22].
- **`hover.py` (`manual_hover`)**[cite: 25]
  * **Purpose:** Calibration flight node for positioning the drone over a ground marker to verify camera intrinsics and detection stability[cite: 25].
  * **Details:** Commands a steady hover at NED `(3.0, 3.0, -3.0)` above a ground marker at `(3, 3, 0)` while displaying a live OpenCV feed with `DICT_6X6_250` ArUco overlays and real-time Euclidean position error[cite: 25].
- **`gen_aruco.py`**[cite: 24]
  * **Purpose:** Standalone utility script for generating `1000×1000 px` `DICT_6X6_250` ArUco marker PNGs (`ID 0–4`) with a 10% white quiet-zone border in `./aruco_markers/` for Gazebo simulation assets[cite: 24].

### 3. Autonomous Trajectory & ArUco Visual Servoing Nodes
- **`aruco.py` (`aruco_marker_tracker`)**[cite: 21]
  * **Purpose:** Full autonomous search-and-track mission node combining minimum-jerk waypoint navigation, `360°` visual scanning, and PID visual servoing[cite: 21].
  * **Details:**
    * **Finite State Machine:** Transitions across `ARMING` → `TAKEOFF` → `WAYPOINT_SEARCH` → `SCANNING_360` → `MARKER_TRACKING` → `HOVERING` at `20 Hz`[cite: 21].
    * **Trajectory & Scan:** Traverses search waypoints via 3D `QuinticTrajectory` polynomials and executes `360°` yaw scans (`0.5 rad/s`) at each waypoint until a marker is detected[cite: 21].
    * **Visual Servoing:** Subscribes to `/realsense/color/image_raw`, estimates 3D pose of `0.2 m` `DICT_6X6_250` markers, and drives a 3-axis `PIDController` (`kp = 0.3, ki = 0.06, kd = 0.1`) with EMA velocity smoothing (`alpha = 0.2`) to lock and hover `2.0 m` in front of the target[cite: 21].
- **`drone_aruco_trajectory.py` (`drone_pd_controller`)**[cite: 23]
  * **Purpose:** High-rate (`50 Hz`) joint-space acceleration controller for minimum-jerk approach and station-keeping in front of a target ArUco marker[cite: 23].
  * **Details:** Plans 3D `QuinticPolynomial` (or `CubicPolynomial`) trajectories in PX4 NED coordinates across `INIT` → `TAKEOFF` → `GOTO_MARKER` → `HOVER_MARKER` phases and computes desired accelerations via `a_cmd = a_r - Kd·ė - Kp·e` (`KP = [2.0, 2.0, 3.0]`, `KD = [2.8, 2.8, 3.5]`) streamed directly to PX4 acceleration setpoints to hold `0.5 m` in front of the marker at NED `[4.5, 0.0, -2.5]`[cite: 23].
  * - **`trajectorysp.py` (`trajectorysp`)**
  * **Purpose:** Minimal `TrajectorySetpoint` publisher for testing position command streaming to PX4[cite: 31].
  * **Details:** Publishes fixed NED position setpoints (`[10.0, 10.0, -10.0]`, `yaw = 0.0`) to `/fmu/in/trajectory_setpoint` at `2 Hz` (`0.5 s` timer)[cite: 31].
- **`vehiclecmd.py` (`vehiclecmd`)**
  * **Purpose:** Standalone `VehicleCommand` publisher for testing mode switching and vehicle arming over Micro XRCE-DDS[cite: 32].
  * **Details:** Sends a single-shot sequence of `VEHICLE_CMD_DO_SET_MODE` (`param1 = 1.0`, `param2 = 6.0` for Offboard mode) followed by `VEHICLE_CMD_COMPONENT_ARM_DISARM` (`param1 = 1.0`) to `/fmu/in/vehicle_command`[cite: 32].
- **`pid.py` (`pid_node`)**
  * **Purpose:** Low-level `50 Hz` vertical-axis PID controller regulating altitude directly via PX4 thrust setpoints[cite: 30].
  * **Details:** Enables `thrust_and_torque` in `OffboardControlMode`, waits for `/fmu/out/vehicle_local_position` telemetry before engaging Offboard mode (`1.0 s`) and arming (`2.0 s`), and computes vertical thrust commands (`kp = 0.8`, `ki = 0.05`, `kd = 0.1`, clamped to `[0.0, 1.0]`) published to `/fmu/in/vehicle_thrust_setpoint` to track a `-5.0 m` NED target altitude[cite: 30].
