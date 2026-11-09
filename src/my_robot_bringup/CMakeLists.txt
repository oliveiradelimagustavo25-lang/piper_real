cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(robot_configuration
    ${CMAKE_CURRENT_SOURCE_DIR}/../../test/test_robot_configuration.py TIMEOUT 60
    ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs)
  ament_add_pytest_test(robot_launch
    ${CMAKE_CURRENT_SOURCE_DIR}/../../test/test_robot_launch.py TIMEOUT 120)
  ament_add_pytest_test(handeye
    ${CMAKE_CURRENT_SOURCE_DIR}/../../test/test_handeye.py TIMEOUT 90)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
