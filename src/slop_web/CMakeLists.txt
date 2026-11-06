cmake_minimum_required(VERSION 3.10)
project(slop_web NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python-Modul slop_web (Webserver, Prozesse, Fahren)
ament_python_install_package(${PROJECT_NAME})

# Konfiguration, Webseite, systemd-Vorlage
install(DIRECTORY config www systemd DESTINATION share/${PROJECT_NAME})

# Startskript (ros2 run slop_web web)
install(PROGRAMS scripts/web DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_web test/test_web.py)
endif()

ament_package()
