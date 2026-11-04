cmake_minimum_required(VERSION 3.10)
project(slop_teleop NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python-Modul slop_teleop
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Startskripte (ros2 run slop_teleop <name>)
install(PROGRAMS
  scripts/keyboard_teleop
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_keys test/test_keys.py)
endif()

ament_package()
