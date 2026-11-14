cmake_minimum_required(VERSION 3.8)
project(my_robot_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(my_robot_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(motion_controller SHARED src/motion_controller.cpp)
target_compile_features(motion_controller PUBLIC cxx_std_17)
target_include_directories(motion_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(motion_controller
  rclcpp rclcpp_action geometry_msgs moveit_msgs moveit_ros_planning_interface tf2 my_robot_interface)

add_library(pick_place_scene SHARED src/pick_place_scene.cpp)
target_compile_features(pick_place_scene PUBLIC cxx_std_17)
target_include_directories(pick_place_scene PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pick_place_scene
  rclcpp geometry_msgs moveit_msgs moveit_ros_planning_interface my_robot_interface
  shape_msgs moveit_core tf2 tf2_ros tf2_geometry_msgs)

add_executable(pick_place_executor src/pick_place_executor.cpp)
target_link_libraries(pick_place_executor motion_controller pick_place_scene)
ament_target_dependencies(pick_place_executor
  rclcpp rclcpp_action my_robot_interface geometry_msgs std_srvs std_msgs tf2 tf2_ros tf2_geometry_msgs)
install(TARGETS pick_place_executor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS motion_controller EXPORT export_motion_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_motion_controller HAS_LIBRARY_TARGET)
install(TARGETS pick_place_scene EXPORT export_pick_place_scene
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_targets(export_pick_place_scene HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action geometry_msgs moveit_msgs moveit_ros_planning_interface
  my_robot_interface shape_msgs moveit_core tf2 tf2_ros tf2_geometry_msgs)

if(BUILD_TESTING)
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
