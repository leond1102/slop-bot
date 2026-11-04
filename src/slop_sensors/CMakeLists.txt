cmake_minimum_required(VERSION 3.10)
project(slop_sensors NONE)

find_package(ament_cmake REQUIRED)

# Nur Launch-Dateien und Treiber-Configs (Livox Mid-360s, RealSense D435i)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
