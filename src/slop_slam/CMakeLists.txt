cmake_minimum_required(VERSION 3.10)
project(slop_slam NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python-Modul slop_slam
ament_python_install_package(${PROJECT_NAME})

# Startskripte (ros2 run slop_slam <name>)
install(PROGRAMS
  scripts/lio_tf
  scripts/mount_calib
  scripts/lio_map
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_transforms test/test_transforms.py)
  ament_add_pytest_test(test_calib_math test/test_calib_math.py)
endif()

ament_package()
