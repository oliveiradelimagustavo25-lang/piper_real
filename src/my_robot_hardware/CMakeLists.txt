cmake_minimum_required(VERSION 3.8)
project(my_robot_hardware)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

add_library(agx_topic_system SHARED src/agx_topic_system.cpp)
target_compile_features(agx_topic_system PUBLIC cxx_std_17)
target_compile_options(agx_topic_system PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(agx_topic_system hardware_interface pluginlib rclcpp sensor_msgs
  controller_manager_msgs)
pluginlib_export_plugin_description_file(hardware_interface hardware_plugins.xml)
install(TARGETS agx_topic_system LIBRARY DESTINATION lib)
ament_package()
