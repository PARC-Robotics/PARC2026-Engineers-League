cmake_minimum_required(VERSION 3.8)
project(parc_robot_bringup)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)
find_package(pluginlib REQUIRED)

install(
  DIRECTORY config launch models rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/load_task_params.py
  scripts/depth_converter.py
  scripts/scan_filter.py
  scripts/scan_merger.py
  scripts/respeaker_sim.py
  # scripts/get_robot_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
