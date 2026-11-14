#include "my_robot_commander/pick_place_scene.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <my_robot_interface/failure_policy.hpp>

#include <moveit/collision_detection/collision_matrix.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace policy = my_robot_interface::failure;

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
  const auto & p = pose.pose.position;
  const auto & q = pose.pose.orientation;
  if (pose.header.frame_id.empty() || !std::isfinite(p.x) || !std::isfinite(p.y) ||
    !std::isfinite(p.z) || !std::isfinite(q.x) || !std::isfinite(q.y) ||
    !std::isfinite(q.z) || !std::isfinite(q.w) ||
    std::abs(std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w)) - 1.0) > 1e-3)
  {
    throw std::invalid_argument("Collision box requires a finite pose and unit quaternion");
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

void validateObjectId(const std::string & id)
{
  if (id.empty() || id.rfind("environment/", 0) == 0) {
    throw std::invalid_argument("Perceived/target object ID is empty or uses reserved environment/ prefix");
  }
}
}  // namespace

PickPlaceScene::PickPlaceScene(
  const rclcpp::Node::SharedPtr & node, tf2_ros::Buffer & tf,
  const std::string & frame, const std::string & tool_link)
: node_(node), tf_(tf), frame_(frame), tool_link_(tool_link), scene_(node->get_namespace())
{
  touch_links_ = parameter<std::vector<std::string>>(
    node_, "touch_links", {"gripper_link1", "gripper_link2"});
  get_scene_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
  auto update = diff();
  collision_detection::AllowedCollisionMatrix matrix(collisionMatrix());
  for (const auto & name : parameter<std::vector<std::string>>(node_, "scene_boxes", {"ground"})) {
    const auto prefix = "scene." + name;
    const auto size = parameter<std::vector<double>>(node_, prefix + ".size", {2.0, 2.0, 0.02});
    const auto xyz = parameter<std::vector<double>>(node_, prefix + ".position", {0.0, 0.0, -0.01});
    const auto rpy = parameter<std::vector<double>>(node_, prefix + ".rpy", {0.0, 0.0, 0.0});
    if (xyz.size() != 3 || rpy.size() != 3 ||
      !std::all_of(xyz.begin(), xyz.end(), [](double v) {return std::isfinite(v);}) ||
      !std::all_of(rpy.begin(), rpy.end(), [](double v) {return std::isfinite(v);}))
    {
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
    throw policy::Error(policy::makeFailure(
      policy::Failure::DEPENDENCY_UNAVAILABLE,
      policy::Failure::SCENE, "MoveIt planning scene service unavailable"));
  }
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components =
    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto result = get_scene_->async_send_request(request);
  if (result.wait_for(5s) != std::future_status::ready) {
    get_scene_->remove_pending_request(result);
    throw policy::Error(policy::makeFailure(
      policy::Failure::DEPENDENCY_UNAVAILABLE,
      policy::Failure::SCENE, "Planning scene query timed out"));
  }
  return result.get()->scene.allowed_collision_matrix;
}

void PickPlaceScene::apply(const moveit_msgs::msg::PlanningScene & update)
{
  if (!scene_.applyPlanningScene(update)) {
    throw policy::Error(policy::makeFailure(
      policy::Failure::SCENE_INCONSISTENT,
      policy::Failure::SCENE, "MoveIt planning scene update was not confirmed"));
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
  geometry_msgs::msg::TransformStamped transform;
  if (observations->header.frame_id != frame_) {
    transform = tf_.lookupTransform(frame_, observations->header.frame_id, stamp,
      tf2::durationFromSec(0.1));
  }
  std::set<std::string> observed;
  for (const auto & object : observations->objects) {
    try {
      validateObjectId(object.object_id);
      if (object.object_id == held_id_) {continue;}
      geometry_msgs::msg::PoseStamped pose;
      pose.header = observations->header;
      pose.pose = object.pose;
      if (pose.header.frame_id != frame_) {
        geometry_msgs::msg::PoseStamped converted;
        tf2::doTransform(pose, converted, transform);
        pose = converted;
      }
      update.world.collision_objects.push_back(
        box(
          object.object_id, pose,
          {object.dimensions.x, object.dimensions.y, object.dimensions.z}));
      observed.insert(object.object_id);
    } catch (const std::invalid_argument & error) {
      // Keep any previous obstacle; one malformed observation cannot replace it.
      RCLCPP_WARN(node_->get_logger(), "Ignoring invalid scene object '%s': %s",
        object.object_id.c_str(), error.what());
    }
  }
  if (!update.world.collision_objects.empty()) {
    apply(update);
    observed_ids_.insert(observed.begin(), observed.end());
  }
}

void PickPlaceScene::prepare(
  const std::string & id,
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Vector3 & dimensions)
{
  validateObjectId(id);
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
  observed_ids_.insert(id);
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
  held_id_ = target_id_;  // Retain the ID even if the update result is lost.
  apply(update);
}

void PickPlaceScene::release()
{
  const auto attached = scene_.getAttachedObjects({held_id_});
  const auto found = attached.find(held_id_);
  if (found == attached.end()) {
    throw policy::Error(policy::makeFailure(
      policy::Failure::SCENE_INCONSISTENT, policy::Failure::SCENE,
      "Expected attached object is missing: " + held_id_));
  }
  auto object = found->second.object;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = object.header;
  pose.header.stamp = builtin_interfaces::msg::Time();
  pose.pose = object.pose;
  pose = tf_.transform(pose, frame_, tf2::durationFromSec(0.1));
  object.header = pose.header;
  object.pose = pose.pose;
  object.operation = object.ADD;
  auto update = diff();
  auto remove = found->second;
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

std::size_t PickPlaceScene::clearObserved()
{
  if (holding() || !previous_touch_.empty()) {
    throw std::runtime_error("Cannot clear objects while a grasp or collision recovery is pending");
  }
  auto update = diff();
  for (const auto & id : observed_ids_) {
    moveit_msgs::msg::CollisionObject object;
    object.id = id;
    object.header.frame_id = frame_;
    object.operation = object.REMOVE;
    update.world.collision_objects.push_back(object);
  }
  if (!observed_ids_.empty()) {apply(update);}
  const auto count = observed_ids_.size();
  observed_ids_.clear();
  return count;
}

}  // namespace my_robot_commander
