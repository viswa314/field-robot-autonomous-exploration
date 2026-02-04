# Field Robot: Autonomous Exploration & SLAM

[![ROS 2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Iron%20%7C%20Rolling-blue)](https://docs.ros.org/en/humble/index.html)
[![License](https://img.shields.io/badge/License-Apache%202.0-orange.svg)](https://opensource.org/licenses/Apache-2.0)

This repository features a comprehensive ROS 2 workspace for a field robot designed for autonomous exploration and environment mapping.

## 📺 Demonstrations

### 1. Autonomous Exploration (Task 3)
<video src="media/Task%203.webm" controls="controls" style="max-width: 100%;">
</video>

### 2. SLAM & Mapping (Task 2)
<video src="media/Task%202.webm" controls="controls" style="max-width: 100%;">
</video>

### 3. Simulation Environment (Task 1)
<video src="media/Task%201.webm" controls="controls" style="max-width: 100%;">
</video>

## 🚀 Key Features
- **Autonomous Exploration**: Implements a frontier-based exploration algorithm.
- **SLAM**: Integrated with `slam_toolbox`.
- **Robust Navigation**: Utilizes `Nav2` with fine-tuned DWB planners.

## 🛠️ Prerequisites & Installation
Ensure you have ROS 2 Humble installed on Ubuntu 22.04.

### 1. Install Dependencies
```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs \
                 ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox
```

### 2. Build the Workspace
```bash
mkdir -p field_robot_ws/src
cd field_robot_ws
# Copy the contents of this repo into src/
colcon build --symlink-install
source install/setup.bash
```

## 🎮 Usage
To launch the full autonomous exploration pipeline:
```bash
./workspace/run_task_3.sh
```

## 📊 System Architecture
The system follows a modular ROS 2 architecture:
- `field_robot_description`: URDF/Xacro models.
- `field_robot_gazebo`: Gazebo plugins.
- `field_robot_navigation`: Nav2 parameters and controllers.

## 📬 Contact
**Gandamalla Viswa** - [viswag001@gmail.com](mailto:viswag001@gmail.com)
