#include "my_robot_commander/pick_place_scene.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <moveit/collision_detection/collision_matrix.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace my_robot_commander
{
namespace
{
template<typename T>
T parameter(const rclcpp::Node::SharedPtr & node, const std::string & name, T fallback)
{
  return node->has_parameter(name) ? node->get_parameter(name).get_value<T>() :
         node->declare_parameter<T>(name, fallback);
}

moveit_msgs::msg::CollisionObject box(
  const std::string & id,
  const geometry_msgs::msg::PoseStamped & pose, const std::vector<double> & size)
{
  if (size.size() != 3 || !std::all_of(
      size.begin(), size.end(),
      [](double v) {return std::isfinite(v) && v > 0;}))
  {
    throw std::invalid_argument("Collision box dimensions must be three positive finite values");
  }
  moveit_msgs::msg::CollisionObject object;
  object.id = id;
  object.header = pose.header;
  object.pose = pose.pose;
  object.operation = object.ADD;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.assign(size.begin(), size.end());
  object.primitives.push_back(primitive);
  // The primitive pose is relative to CollisionObject.pose in current MoveIt messages.
  geometry_msgs::msg::Pose primitive_pose;
  primitive_pose.orientation.w = 1.0;
  object.primitive_poses.push_back(primitive_pose);
  return object;
}
moveit_msgs::msg::PlanningScene diff()
{
  moveit_msgs::msg::PlanningScene result;
  result.is_diff = true;
  result.robot_state.is_diff = true;
  return result;
}
}  // namespace

PickPlaceScene::PickPlaceScene(
  const rclcpp::Node::SharedPtr & node, tf2_ros::Buffer & tf,
  const std::string & frame, const std::string & tool_link)
: node_(node), tf_(tf), frame_(frame), tool_link_(tool_link)
{
  touch_links_ = parameter<std::vector<std::string>>(node_, "touch_links", {"link7", "link8"});
  get_scene_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
  auto update = diff();
  collision_detection::AllowedCollisionMatrix matrix(collisionMatrix());
  for (const auto & name : parameter<std::vector<std::string>>(node_, "scene_boxes", {"ground"})) {
    const auto prefix = "scene." + name;
    const auto size = parameter<std::vector<double>>(node_, prefix + ".size", {2.0, 2.0, 0.02});
    const auto xyz = parameter<std::vector<double>>(node_, prefix + ".position", {0.0, 0.0, -0.01});
    const auto rpy = parameter<std::vector<double>>(node_, prefix + ".rpy", {0.0, 0.0, 0.0});
    if (xyz.size() != 3 || rpy.size() != 3) {
      throw std::invalid_argument("Collision box position/rpy must have three values");
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_;
    pose.pose.position.x = xyz[0]; pose.pose.position.y = xyz[1]; pose.pose.position.z = xyz[2];
    tf2::Quaternion q;
    q.setRPY(rpy[0], rpy[1], rpy[2]);
    pose.pose.orientation = tf2::toMsg(q);
    const auto id = "environment/" + name;
    update.world.collision_objects.push_back(box(id, pose, size));
    // 安装底座允许与支撑面接触，其他连杆仍做碰撞检查。
    if (name == "ground") {matrix.setEntry(id, "base_link", true);}
  }
  matrix.getMessage(update.allowed_collision_matrix);
  apply(update);
}

moveit_msgs::msg::AllowedCollisionMatrix PickPlaceScene::collisionMatrix()
{
  using namespace std::chrono_literals;
  if (!get_scene_->wait_for_service(5s)) {
    throw std::runtime_error("MoveIt planning scene service unavailable");
  }
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components =
    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto result = get_scene_->async_send_request(request);
  if (result.wait_for(5s) != std::future_status::ready) {
    get_scene_->remove_pending_request(result);
    throw std::runtime_error("Planning scene query timed out");
  }
  return result.get()->scene.allowed_collision_matrix;
}

void PickPlaceScene::apply(const moveit_msgs::msg::PlanningScene & update)
{
  if (!scene_.applyPlanningScene(update)) {
    throw std::runtime_error("MoveIt rejected the planning scene update");
  }
}

void PickPlaceScene::sync(
  const my_robot_interface::msg::DetectedObjectArray::ConstSharedPtr & observations)
{
  if (!observations) {return;}
  const auto stamp = rclcpp::Time(observations->header.stamp, node_->get_clock()->get_clock_type());
  const double age = (node_->now() - stamp).seconds();
  if (stamp.nanoseconds() != 0 && (age < -0.1 || age > 2.0)) {return;}
  auto update = diff();
  for (const auto & object : observations->objects) {
    if (object.object_id == held_id_) {continue;}
    geometry_msgs::msg::PoseStamped pose;
    pose.header = observations->header;
    pose.pose = object.pose;
    if (pose.header.frame_id != frame_) {
      pose = tf_.transform(pose, frame_, tf2::durationFromSec(0.1));
    }
    update.world.collision_objects.push_back(
      box(
        object.object_id, pose,
        {object.dimensions.x, object.dimensions.y, object.dimensions.z}));
  }
  if (!update.world.collision_objects.empty()) {apply(update);}
}

void PickPlaceScene::prepare(
  const std::string & id,
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Vector3 & dimensions)
{
  target_id_ = id;
  target_ = box(id, pose, {dimensions.x, dimensions.y, dimensions.z});
  auto update = diff();
  update.world.collision_objects.push_back(target_);
  collision_detection::AllowedCollisionMatrix matrix(collisionMatrix());
  previous_touch_.clear();
  for (const auto & link : touch_links_) {
    collision_detection::AllowedCollision::Type type;
    previous_touch_[link] = matrix.getEntry(id, link, type) ?
      static_cast<int>(type == collision_detection::AllowedCollision::ALWAYS) : -1;
    matrix.setEntry(id, link, true);
  }
  matrix.getMessage(update.allowed_collision_matrix);
  apply(update);
}

void PickPlaceScene::attach()
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = target_.header;
  pose.header.stamp = builtin_interfaces::msg::Time();
  pose.pose = target_.pose;
  pose = tf_.transform(pose, tool_link_, tf2::durationFromSec(0.1));
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = tool_link_;
  attached.touch_links = touch_links_;
  attached.object = target_;
  attached.object.header = pose.header;
  attached.object.pose = pose.pose;
  auto update = diff();
  // MoveIt 在附着 ADD 时自动从世界移除同名物体。
  update.robot_state.attached_collision_objects.push_back(attached);
  apply(update);
  held_id_ = target_id_;
}

void PickPlaceScene::release()
{
  const auto attached = scene_.getAttachedObjects({held_id_});
  auto object = attached.at(held_id_).object;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = object.header;
  pose.header.stamp = builtin_interfaces::msg::Time();
  pose.pose = object.pose;
  pose = tf_.transform(pose, frame_, tf2::durationFromSec(0.1));
  object.header = pose.header;
  object.pose = pose.pose;
  object.operation = object.ADD;
  auto update = diff();
  auto remove = attached.at(held_id_);
  remove.object.operation = remove.object.REMOVE;
  update.robot_state.attached_collision_objects.push_back(remove);
  update.world.collision_objects.push_back(object);
  apply(update);
  held_id_.clear();
}

void PickPlaceScene::restoreTouch()
{
  if (previous_touch_.empty()) {return;}
  auto update = diff();
  collision_detection::AllowedCollisionMatrix matrix(collisionMatrix());
  for (const auto & entry : previous_touch_) {
    if (entry.second < 0) {matrix.removeEntry(target_id_, entry.first);} else {
      matrix.setEntry(target_id_, entry.first, entry.second != 0);
    }
  }
  matrix.getMessage(update.allowed_collision_matrix);
  apply(update);
  previous_touch_.clear();
}

}  // namespace my_robot_commander
