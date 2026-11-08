cmake_minimum_required(VERSION 3.5)
project(vidu_ros2_wrapper)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Vidu_SDK_Api REQUIRED)
find_package(vidu_ros2_interfaces REQUIRED)

# Internal SDK-to-ROS conversion/calibration helpers, shared with offline tests.
add_library(vidu_conversions STATIC src/conversions.cpp)
target_include_directories(vidu_conversions PUBLIC include ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(vidu_conversions rclcpp sensor_msgs camera_info_manager)
target_link_libraries(vidu_conversions Vidu_SDK_Api::GenTLwrapper ${OpenCV_LIBS})

add_executable(${PROJECT_NAME}_node src/vidu_ros2_wrapper.cpp)
target_link_libraries(${PROJECT_NAME}_node vidu_conversions)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp rcl_interfaces cv_bridge std_msgs sensor_msgs vidu_ros2_interfaces)
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(WORKSPACE_TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../test)
  ament_add_gtest_executable(conversion_probe ${WORKSPACE_TEST_DIR}/native/test_conversions.cpp)
  target_link_libraries(conversion_probe vidu_conversions)
  ament_add_pytest_test(conversions ${WORKSPACE_TEST_DIR}/test_conversions.py TIMEOUT 40
    ENV VIDU_CONVERSION_PROBE=$<TARGET_FILE:conversion_probe>
      ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs)
endif()

ament_package()
