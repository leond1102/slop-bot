cmake_minimum_required(VERSION 3.10)
project(slop_sensors NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

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
