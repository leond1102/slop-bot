cmake_minimum_required(VERSION 3.10)
project(slop_sensors CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Launch-Dateien und Treiber-Configs (Livox Mid-360s, RealSense D435i)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# livox_to_cloud: Livox-CustomMsg → PointCloud2 (für pointcloud_to_laserscan und Costmaps)
add_executable(livox_to_cloud src/livox_to_cloud.cpp)
target_compile_options(livox_to_cloud PRIVATE -Wall -Wextra -O2)
ament_target_dependencies(livox_to_cloud rclcpp sensor_msgs livox_ros_driver2)

# depth_obstacles: Hindernisse aus dem Tiefenbild der RealSense (für die Nav2-Costmap)
add_executable(depth_obstacles src/depth_obstacles.cpp)
target_compile_options(depth_obstacles PRIVATE -Wall -Wextra -O2)
ament_target_dependencies(depth_obstacles rclcpp sensor_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS livox_to_cloud depth_obstacles DESTINATION lib/${PROJECT_NAME})

ament_package()
