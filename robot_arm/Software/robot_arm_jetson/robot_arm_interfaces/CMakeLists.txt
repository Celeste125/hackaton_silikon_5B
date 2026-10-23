cmake_minimum_required(VERSION 3.8)
project(robot_arm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Gripper.action"
  "action/MoveDistance.action"
  DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
