![image](https://github.com/user-attachments/assets/16924cee-3290-427a-b5f3-cafb4270333f)

# SLAMBot – Cloud-Controlled Robot (ROS 2 Humble)

This repository contains the complete ROS 2 Humble-based codebase for **SLAMBot**, a mobile robot designed to perform SLAM and remote navigation as part of a cloud-managed fleet system.

> 🛠️ Status: IN DEVELOPMENT

---

## 🧭 Overview

SLAMBot is a ground robot capable of mapping and navigating unknown environments using onboard sensors and SLAM techniques. The robot connects to a cloud-based control system, allowing for remote fleet-level management and monitoring.

---

## 📁 Repository Structure

| Folder                | Description                                                  |
|-----------------------|--------------------------------------------------------------|
| `slambot_controller`  | ROS 2 node for handling velocity commands and control logic  |
| `slambot_description` | URDF/xacro models and RViz configuration for the robot       |
| `slambot_mapping`     | SLAM configuration and launch files (e.g., slam_toolbox)     |
| `slambot_firmware`    | Low-level motor/firmware communication layer (e.g., ESP32)   |

---

## 🚀 Features

- Real-time SLAM using `slam_toolbox`
- Velocity command processing via `/cmd_vel`
- Cloud-based fleet integration (MQTT/REST coming soon)
- Accurate robot visualization in RViz
- Modular structure for scalable multi-robot control

---

## 🔧 Requirements

- ROS 2 Humble (Ubuntu 22.04)
- Lidar (e.g., RPLidar A1, LDS-01)
- Differential drive setup
- ESP32 or custom motor controller
- Internet/Wi-Fi for cloud connectivity

---

## 🛠️ Build and Run

```bash
# Clone into ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/saieswaramurali/slambot.git

# Build the workspace
cd ~/ros2_ws
colcon build

# Source the workspace
source install/setup.bash
