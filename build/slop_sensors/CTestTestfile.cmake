# CMake generated Testfile for 
# Source directory: /home/jetson/slop-bot_ws/src/slop_sensors
# Build directory: /home/jetson/slop-bot_ws/build/slop_sensors
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_gravity "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/jetson/slop-bot_ws/build/slop_sensors/test_results/slop_sensors/test_gravity.xunit.xml" "--package-name" "slop_sensors" "--output-file" "/home/jetson/slop-bot_ws/build/slop_sensors/ament_cmake_pytest/test_gravity.txt" "--command" "/usr/bin/python3" "-u" "-m" "pytest" "/home/jetson/slop-bot_ws/src/slop_sensors/test/test_gravity.py" "-o" "cache_dir=/home/jetson/slop-bot_ws/build/slop_sensors/ament_cmake_pytest/test_gravity/.cache" "--junit-xml=/home/jetson/slop-bot_ws/build/slop_sensors/test_results/slop_sensors/test_gravity.xunit.xml" "--junit-prefix=slop_sensors")
set_tests_properties(test_gravity PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/jetson/slop-bot_ws/build/slop_sensors" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;169;ament_add_test;/home/jetson/slop-bot_ws/src/slop_sensors/CMakeLists.txt;19;ament_add_pytest_test;/home/jetson/slop-bot_ws/src/slop_sensors/CMakeLists.txt;0;")
