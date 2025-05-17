cmake_minimum_required(VERSION 3.5)
project(ception_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # msg
  "msg/IMUEuler.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs)

ament_package()


