# CMake generated Testfile for 
# Source directory: /home/jetson/slop-bot_ws/src/slop_safety
# Build directory: /home/jetson/slop-bot_ws/build/slop_safety
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_remote_guard "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/jetson/slop-bot_ws/build/slop_safety/test_results/slop_safety/test_remote_guard.xunit.xml" "--package-name" "slop_safety" "--output-file" "/home/jetson/slop-bot_ws/build/slop_safety/ament_cmake_pytest/test_remote_guard.txt" "--command" "/usr/bin/python3" "-u" "-m" "pytest" "/home/jetson/slop-bot_ws/src/slop_safety/test/test_remote_guard.py" "-o" "cache_dir=/home/jetson/slop-bot_ws/build/slop_safety/ament_cmake_pytest/test_remote_guard/.cache" "--junit-xml=/home/jetson/slop-bot_ws/build/slop_safety/test_results/slop_safety/test_remote_guard.xunit.xml" "--junit-prefix=slop_safety")
set_tests_properties(test_remote_guard PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/jetson/slop-bot_ws/build/slop_safety" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;169;ament_add_test;/home/jetson/slop-bot_ws/src/slop_safety/CMakeLists.txt;19;ament_add_pytest_test;/home/jetson/slop-bot_ws/src/slop_safety/CMakeLists.txt;0;")
