cmake_minimum_required(VERSION 3.8)
project(my_robot_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# 生成消息与 Action 的 C++ / Python 接口。
rosidl_generate_interfaces( ${PROJECT_NAME}
    "msg/Robotinterface.msg"
    "msg/DetectedObject.msg"
    "msg/DetectedObjectArray.msg"
    "action/PickPlace.action"
    "action/MoveToNamed.action"
    "action/SortObjects.action"
    DEPENDENCIES geometry_msgs std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

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
