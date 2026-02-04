# Assignment 01: Field Robotics - Autonomous Exploration

## Description
This project implements an autonomous exploration task for a field robot in a Gazebo simulation environment. The robot uses SLAM to map the environment and a navigation stack to explore autonomously.

## Prerequisites
- **ROS 2** (Humble/Iron/Rolling)
- `gazebo_ros_pkgs`
- `navigation2`
- `slam_toolbox`

## Installation
Ensure you have the necessary dependencies installed:
```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
```

## Setup & Compilation
1. Navigate to the workspace directory:
   ```bash
   cd workspace
   ```
2. Build the workspace:
   ```bash
   colcon build
   ```
3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage
To run the autonomous exploration task (Task 3), execute the provided script in the `workspace` directory:

```bash
./run_task_3.sh
```

This script will automatically:
1. Launch the Gazebo simulation with the robot.
2. Start SLAM and the Navigation2 stack.
3. specific exploration controller node to drive the robot.

## Author
**Gandamalla Viswa**
viswag001@gmail.com
