cmake_minimum_required(VERSION 3.5)
project(turtles_service)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NavigateTo.srv"
)

ament_python_install_package(${PROJECT_NAME})

ament_package()
