#!/bin/bash

# Script to automate Task 3: Autonomous Exploration
# This script runs the necessary commands in separate background processes.

# Ensure the workspace is sourced
WORKSPACE_DIR="/home/viswa/ASSIGNMENT/assignment_01_field_robotics/workspace"
cd $WORKSPACE_DIR

if [ -f install/setup.bash ]; then
    source install/setup.bash
else
    echo "Error: install/setup.bash not found. Please build your workspace first using 'colcon build'."
    exit 1
fi

echo "Starting Terminal 1: Gazebo (spawn_robot_gazebo.launch.py)..."
ros2 launch field_robot_gazebo spawn_robot_gazebo.launch.py &
GAZEBO_PID=$!

sleep 10

echo "Starting Terminal 2: SLAM + Nav2 (slam_exploration.launch.py)..."
ros2 launch field_robot_navigation slam_exploration.launch.py &
SLAM_PID=$!

echo "Waiting 15 seconds for system to stabilize..."
sleep 15

echo "Starting Terminal 3: Exploration Controller (exploration_controller.py)..."
ros2 run field_robot_navigation exploration_controller.py

# Keep script running while processes are active
wait $GAZEBO_PID $SLAM_PID
