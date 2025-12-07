cmake_minimum_required(VERSION 3.8)
project(robot_dog_description)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)

install(
  DIRECTORY urdf meshes config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/gait_trajectory.py
    scripts/gait_patterns.py
    scripts/base_mover.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
