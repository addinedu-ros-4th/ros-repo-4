cmake_minimum_required(VERSION 3.5)
project(turtles_service_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ArucoNavigateTo.srv"
  "srv/NavToPose.srv"
  "msg/ActionClientResult.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()