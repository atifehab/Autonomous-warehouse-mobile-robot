
# 🤖 Autonomous Mobile Robot (ROS 2 Jazzy)

This repository contains the full software stack for an **Autonomous Mobile Robot (AMR)** using **ROS 2 Jazzy**, designed for real hardware (not simulation).  
It supports **SLAM**, **localization**, and **fully autonomous navigation** using **Nav2**.

---

## ✨ Features

- 🗺️ LiDAR-based SLAM using `slam_toolbox`
- 📍 Localization using AMCL
- 🚀 Autonomous navigation using Nav2
- 🧭 Sensor fusion (IMU + wheel odometry) with EKF
- 🧱 Obstacle avoidance
- 🔌 ESP32 + micro-ROS motor control
- 🛠️ Real robot tested

---

## 🧩 Hardware Used

- RPLIDAR A1
- ESP32 (micro-ROS)
- DC Motors with Cytron motor driver
- MPU6050 IMU
- Differential drive robot

---

## 🗂️ Repository Structure

```
robot/
 ├── config/
 │   ├── slam.yaml
 │   ├── amcl.yaml
 │   ├── nav2_params.yaml
 │   ├── ekf.yaml
 │
 ├── launch/
 │   ├── localization.launch.py
 │   ├── navigation.launch.py
 │
 ├── urdf/
 │   └── robot.urdf
 │
awmr_autonomous/
 ├── auto_mapper.py
 ├── cmd_vel_to_motor.py
 ├── safety_node.py
```

---

## 🛠️ Requirements

- Ubuntu 22.04
- ROS 2 Jazzy
- Python 3.10+

Install required packages:
```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup                  ros-jazzy-slam-toolbox ros-jazzy-robot-localization
```

---

## 🗺️ Mapping (SLAM)

Run SLAM to create a map:

```bash
ros2 launch slam_toolbox online_async_launch.py \
slam_params_file:=~/ros2_jazzy/src/robot/config/slam.yaml
```

Save the map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_jazzy/maps/robot_map
```

---

## 📍 Localization (AMCL)

Run localization using a saved map:

```bash
ros2 launch nav2_bringup localization_launch.py \
map:=~/ros2_jazzy/maps/robot_map.yaml \
params_file:=~/ros2_jazzy/src/robot/config/nav2_params.yaml
```

⚠️ Do **NOT** run `slam_toolbox` while using AMCL.

---

## 🚀 Autonomous Navigation

Start Nav2 navigation:

```bash
ros2 launch nav2_bringup navigation_launch.py \
params_file:=~/ros2_jazzy/src/robot/config/nav2_params.yaml
```

In **RViz**:
1. Set **Fixed Frame** → `map`
2. Use **2D Pose Estimate**
3. Send navigation goals

---

## 🧭 Sensor Fusion (EKF)

- IMU (MPU6050) + wheel odometry
- Publishes stable `/odom`
- Improves localization accuracy

EKF config located in:
```
robot/config/ekf.yaml
```

---

## 📊 Notes

- Initial pose must be set in RViz when using AMCL
- LiDAR frame name must match URDF
- EKF must run before Nav2
- Motor speed calibration is done mathematically (no trial & error)

