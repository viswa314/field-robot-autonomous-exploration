cmake_minimum_required(VERSION 3.8)
project(field_robot_navigation)

# Find dependencies
find_package(ament_cmake REQUIRED)
# find_package(nav2_common REQUIRED)
find_package(rclpy REQUIRED)

# Install directories
install(DIRECTORY
  config
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/circular_trajectory.py
  scripts/exploration_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
