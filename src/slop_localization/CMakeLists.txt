cmake_minimum_required(VERSION 3.10)
project(slop_localization NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python-Modul slop_localization
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Startskripte (ros2 run slop_localization <name>)
install(PROGRAMS
  scripts/imu_adapter
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_imu test/test_imu.py)
endif()

ament_package()
