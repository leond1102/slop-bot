cmake_minimum_required(VERSION 3.8)
project(slop_base NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python-Modul slop_base
ament_python_install_package(${PROJECT_NAME})

# Startskripte (ros2 run slop_base <name>)
install(PROGRAMS
  scripts/diff_drive
  scripts/with_netcaps
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_kinematics test/test_kinematics.py)
  ament_add_pytest_test(test_cl57e_bus test/test_cl57e_bus.py)
endif()

ament_package()
