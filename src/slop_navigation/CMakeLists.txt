cmake_minimum_required(VERSION 3.10)
project(slop_navigation NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python-Modul slop_navigation (Relokalisierung)
ament_python_install_package(${PROJECT_NAME})

# Launch-Dateien und Parameter (slam_toolbox, AMCL, später Nav2)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Startskripte (ros2 run slop_navigation <name>)
install(PROGRAMS scripts/relocalize DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_relocalize test/test_relocalize.py)
endif()

ament_package()
