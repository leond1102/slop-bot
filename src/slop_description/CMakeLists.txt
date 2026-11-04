cmake_minimum_required(VERSION 3.10)
project(slop_description NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
