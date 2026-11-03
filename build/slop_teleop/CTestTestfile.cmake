# CMake generated Testfile for 
# Source directory: /home/jetson/slop-bot_ws/src/slop_teleop
# Build directory: /home/jetson/slop-bot_ws/build/slop_teleop
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_keys "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/jetson/slop-bot_ws/build/slop_teleop/test_results/slop_teleop/test_keys.xunit.xml" "--package-name" "slop_teleop" "--output-file" "/home/jetson/slop-bot_ws/build/slop_teleop/ament_cmake_pytest/test_keys.txt" "--command" "/usr/bin/python3" "-u" "-m" "pytest" "/home/jetson/slop-bot_ws/src/slop_teleop/test/test_keys.py" "-o" "cache_dir=/home/jetson/slop-bot_ws/build/slop_teleop/ament_cmake_pytest/test_keys/.cache" "--junit-xml=/home/jetson/slop-bot_ws/build/slop_teleop/test_results/slop_teleop/test_keys.xunit.xml" "--junit-prefix=slop_teleop")
set_tests_properties(test_keys PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/jetson/slop-bot_ws/build/slop_teleop" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;169;ament_add_test;/home/jetson/slop-bot_ws/src/slop_teleop/CMakeLists.txt;19;ament_add_pytest_test;/home/jetson/slop-bot_ws/src/slop_teleop/CMakeLists.txt;0;")
