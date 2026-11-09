cmake_minimum_required(VERSION 3.10)
project(slop_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Grip.srv"
  "srv/Move.srv"
)

ament_package()
