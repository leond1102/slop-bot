cmake_minimum_required(VERSION 3.10)
project(slop_navigation NONE)

find_package(ament_cmake REQUIRED)

# Launch-Dateien und Parameter (slam_toolbox, später Nav2)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
