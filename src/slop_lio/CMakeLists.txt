cmake_minimum_required(VERSION 3.10)
project(slop_lio NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python-Modul slop_lio
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Startskripte (ros2 run slop_lio <name>)
install(PROGRAMS
  scripts/lio_bridge
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_lio_math test/test_lio_math.py)
  ament_add_pytest_test(test_crop test/test_crop.py)
endif()

ament_package()
