cmake_minimum_required(VERSION 3.10)
project(slop_calibration NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python-Modul slop_calibration
ament_python_install_package(${PROJECT_NAME})

# Startskripte (ros2 run slop_calibration <name>)
install(PROGRAMS
  scripts/calibrate
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_store test/test_store.py)
  ament_add_pytest_test(test_still test/test_still.py)
endif()

ament_package()
