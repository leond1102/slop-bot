cmake_minimum_required(VERSION 3.10)
project(slop_navigation NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python-Modul slop_navigation (Relokalisierung, Erfahrungskarte)
ament_python_install_package(${PROJECT_NAME})

# Launch-Dateien und Parameter (slam_toolbox, AMCL, später Nav2)
install(DIRECTORY config launch behavior_trees DESTINATION share/${PROJECT_NAME})

# Startskripte (ros2 run slop_navigation <name>)
install(PROGRAMS scripts/relocalize scripts/experience scripts/experience_export scripts/unknown_obstacles scripts/experience_transfer scripts/extend_start DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_relocalize test/test_relocalize.py)
  ament_add_pytest_test(test_experience test/test_experience.py)
  ament_add_pytest_test(test_safety test/test_safety.py)
  ament_add_pytest_test(test_map_extend test/test_map_extend.py)
endif()

ament_package()
