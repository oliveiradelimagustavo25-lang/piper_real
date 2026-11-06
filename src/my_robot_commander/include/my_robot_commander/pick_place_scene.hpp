#ifndef MY_ROBOT_COMMANDER__PICK_PLACE_SCENE_HPP_
#define MY_ROBOT_COMMANDER__PICK_PLACE_SCENE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <my_robot_interface/msg/detected_object_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

namespace my_robot_commander
{

// 同步场景接口，调用时需有 executor 处理 node 的 ROS 回调。
// node 和 tf 必须比本实例活得更久；场景操作由同一线程顺序执行。
// 视觉订阅应只缓存观测，在运动前调用 sync()，不在搬运中改写物块位置。
class PickPlaceScene
{
public:
  PickPlaceScene(
    const rclcpp::Node::SharedPtr & node, tf2_ros::Buffer & tf,
    const std::string & frame, const std::string & tool_link);

  PickPlaceScene(const PickPlaceScene &) = delete;
  PickPlaceScene & operator=(const PickPlaceScene &) = delete;

  void sync(const my_robot_interface::msg::DetectedObjectArray::ConstSharedPtr & observations);
  void prepare(
    const std::string & id, const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Vector3 & dimensions);
  void attach();
  void release();
  void restoreTouch();
  bool holding() const {return !held_id_.empty();}

private:
  moveit_msgs::msg::AllowedCollisionMatrix collisionMatrix();
  void apply(const moveit_msgs::msg::PlanningScene & scene);
  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer & tf_;
  std::string frame_, tool_link_, held_id_, target_id_;
  std::vector<std::string> touch_links_;
  std::map<std::string, int> previous_touch_;
  moveit_msgs::msg::CollisionObject target_;
  moveit::planning_interface::PlanningSceneInterface scene_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_;
};

}  // namespace my_robot_commander

#endif  // MY_ROBOT_COMMANDER__PICK_PLACE_SCENE_HPP_
