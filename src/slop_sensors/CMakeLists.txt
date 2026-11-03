cmake_minimum_required(VERSION 3.10)
project(slop_sensors CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# C++-Komponente: IMU drosseln (für mount_tf), läuft im Container sensor_helpers
add_library(imu_throttle SHARED src/imu_throttle.cpp)
ament_target_dependencies(imu_throttle rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(imu_throttle PLUGIN "slop_sensors::ImuThrottle" EXECUTABLE imu_throttle_node)
install(TARGETS imu_throttle ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

# Python-Modul slop_sensors
ament_python_install_package(${PROJECT_NAME})

# Startskripte (ros2 run slop_sensors <name>)
install(PROGRAMS
  scripts/mount_tf
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_gravity test/test_gravity.py)
endif()

ament_package()
