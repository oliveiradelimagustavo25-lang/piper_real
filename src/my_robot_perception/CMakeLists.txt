cmake_minimum_required(VERSION 3.8)
project(my_robot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(my_robot_interface REQUIRED)
find_package(vidu_ros2_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(color_detector src/color_detector.cpp)
target_compile_features(color_detector PRIVATE cxx_std_17)
ament_target_dependencies(color_detector rclcpp rcl_interfaces sensor_msgs cv_bridge)
target_include_directories(color_detector PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(color_detector ${OpenCV_LIBS})

add_executable(aruco_detector src/aruco_detector.cpp)
ament_target_dependencies(aruco_detector
  rclcpp rcl_interfaces sensor_msgs geometry_msgs cv_bridge tf2 tf2_ros)
target_include_directories(aruco_detector PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(aruco_detector ${OpenCV_LIBS})

add_executable(depth_localizer src/depth_localizer.cpp)
target_compile_features(depth_localizer PRIVATE cxx_std_17)
ament_target_dependencies(depth_localizer
  rclcpp rcl_interfaces sensor_msgs geometry_msgs my_robot_interface
  vidu_ros2_interfaces cv_bridge message_filters tf2 tf2_ros tf2_geometry_msgs)
target_include_directories(depth_localizer PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(depth_localizer ${OpenCV_LIBS})

install(TARGETS color_detector aruco_detector depth_localizer
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(visual_pipeline
    ${CMAKE_CURRENT_SOURCE_DIR}/../../test/test_visual_pipeline.py TIMEOUT 90
    ENV ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs)
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
