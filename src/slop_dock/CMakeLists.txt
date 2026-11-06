cmake_minimum_required(VERSION 3.10)
project(slop_dock NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python-Modul slop_dock (Marker-Erkennung, Regelung, Dock-Datei)
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Startskripte (ros2 run slop_dock <name>)
install(PROGRAMS scripts/dock DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_dock test/test_dock.py)
endif()

ament_package()
