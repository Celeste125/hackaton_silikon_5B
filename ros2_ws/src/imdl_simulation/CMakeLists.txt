cmake_minimum_required(VERSION 3.8)
project(imdl_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS
    scripts/arena_marker_publisher.py
    scripts/scan_self_filter.py
    scripts/simulation_readiness.py
    scripts/static_tf_relay.py
    scripts/task_matrix_runner.py
    scripts/ultrasonic_range_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
