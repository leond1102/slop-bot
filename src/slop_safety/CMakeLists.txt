cmake_minimum_required(VERSION 3.8)
project(slop_safety NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python-Modul slop_safety
ament_python_install_package(${PROJECT_NAME})

# Startskripte (ros2 run slop_safety <name>)
install(PROGRAMS
  scripts/remote_guard
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_remote_guard test/test_remote_guard.py)
endif()

ament_package()
