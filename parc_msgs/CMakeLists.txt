cmake_minimum_required(VERSION 3.8)
project(parc_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TagOrder.msg"
  "msg/StartSignal.msg"
  "msg/TagVisit.msg"
  "msg/Finish.msg"
  "msg/RunStatus.msg"
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
