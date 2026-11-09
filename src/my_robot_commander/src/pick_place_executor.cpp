#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <my_robot_interface/action/pick_place.hpp>
#include <my_robot_interface/action/move_to_named.hpp>
#include <my_robot_interface/msg/detected_object_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "my_robot_commander/motion_controller.hpp"
#include "my_robot_commander/pick_place_scene.hpp"

namespace
{
template<typename T>
T parameter(const rclcpp::Node::SharedPtr & node, const std::string & name, T fallback)
{
  return node->has_parameter(name) ? node->get_parameter(name).get_value<T>() :
         node->declare_parameter<T>(name, fallback);
}

tf2::Vector3 vector3(const std::vector<double> & values)
{
  if (values.size() != 3) {
    throw std::invalid_argument("Expected an xyz or rpy parameter with 3 values");
  }
  return {values[0], values[1], values[2]};
}

// 输入是夹持中心，MoveIt 目标是 link6 原点，两者相差一个旋转后的 TCP 偏移。
geometry_msgs::msg::PoseStamped toolPose(
  const tf2::Vector3 & tcp, const tf2::Quaternion & rotation,
  const tf2::Vector3 & tool_to_tcp, const std::string & frame)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = frame;
  const auto origin = tcp - tf2::quatRotate(rotation, tool_to_tcp);
  result.pose.position.x = origin.x();
  result.pose.position.y = origin.y();
  result.pose.position.z = origin.z();
  result.pose.orientation = tf2::toMsg(rotation);
  return result;
}

// 盒子在夹爪局部 Y（闭合方向）上的投影宽度。
double graspWidth(
  const geometry_msgs::msg::Pose & object,
  const geometry_msgs::msg::Vector3 & dimensions, const tf2::Quaternion & tool_rotation)
{
  tf2::Quaternion object_rotation;
  tf2::fromMsg(object.orientation, object_rotation);
  object_rotation.normalize();
  const tf2::Matrix3x3 relative(tool_rotation.inverse() * object_rotation);
  return std::abs(relative[1][0]) * dimensions.x +
         std::abs(relative[1][1]) * dimensions.y +
         std::abs(relative[1][2]) * dimensions.z;
}
}  // namespace

class PickPlaceExecutor
{
  using Action = my_robot_interface::action::PickPlace;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using NamedAction = my_robot_interface::action::MoveToNamed;
  using NamedHandle = rclcpp_action::ServerGoalHandle<NamedAction>;
  using Motion = my_robot_commander::MotionController;
  using Scene = my_robot_commander::PickPlaceScene;

public:
  explicit PickPlaceExecutor(const rclcpp::Node::SharedPtr & node)
  : node_(node), tf_buffer_(node->get_clock()), tf_listener_(tf_buffer_)
  {
    Motion::Options options;
    options.arm_group = parameter(node_, "arm_group", options.arm_group);
    options.gripper_group = parameter(node_, "gripper_group", options.gripper_group);
    options.gripper_width_joint = parameter(
      node_, "gripper_width_joint", options.gripper_width_joint);
    frame_ = options.reference_frame = parameter(node_, "reference_frame", options.reference_frame);
    options.end_effector_link = parameter<std::string>(node_, "end_effector_link", "link6");
    options.velocity_scaling = parameter(node_, "velocity_scaling", options.velocity_scaling);
    options.acceleration_scaling = parameter(
      node_, "acceleration_scaling",
      options.acceleration_scaling);
    options.planning_time = parameter(node_, "planning_time", options.planning_time);
    options.cartesian_step = parameter(node_, "cartesian_step", options.cartesian_step);
    tool_to_tcp_ = vector3(parameter<std::vector<double>>(node_, "tool_to_tcp", {0, 0, 0.132}));
    offset_ = vector3(parameter<std::vector<double>>(node_, "grasp_offset", {0, 0, 0}));
    const auto rpy = vector3(
      parameter<std::vector<double>>(
        node_, "grasp_rpy", {0, 3.141592653589793, 0}));
    tool_rotation_.setRPY(rpy.x(), rpy.y(), rpy.z());
    approach_ = parameter(node_, "approach_distance", 0.04);
    lift_ = parameter(node_, "lift_distance", 0.06);
    transport_z_ = parameter(node_, "transport_height", 0.08);
    clearance_ = parameter(node_, "release_clearance", 0.002);
    retreat_ = parameter(node_, "retreat_distance", 0.04);
    open_width_ = parameter(node_, "gripper_open_width", 0.095);
    pad_thickness_ = parameter(node_, "gripper_pad_thickness", 0.0);
    compression_ = parameter(node_, "gripper_compression", 0.002);
    max_age_ = parameter(node_, "max_target_age", 5.0);
    tf_timeout_ = parameter(node_, "tf_timeout", 0.3);
    if (!std::isfinite(open_width_) || !std::isfinite(pad_thickness_) ||
      !std::isfinite(compression_) || pad_thickness_ < 0 ||
      open_width_ <= 2 * pad_thickness_ || compression_ < 0 ||
      approach_ <= 0 || lift_ <= 0 || retreat_ <= 0 || clearance_ < 0)
    {
      throw std::invalid_argument("Invalid gripper configuration or approach/lift/retreat distances");
    }
    // Wait for the dependency when the independent layers start concurrently.
    // Robot descriptions are received from the control stack's latched topics.
    auto planning_scene = node_->create_client<moveit_msgs::srv::GetPlanningScene>(
      "get_planning_scene");
    while (!planning_scene->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error("Shutdown while waiting for MoveIt");
      }
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Waiting for control stack: get_planning_scene");
    }
    motion_ = std::make_unique<Motion>(node_, options);
    const auto width_limits = motion_->gripperWidthLimits();
    if (open_width_ < width_limits.first || open_width_ > width_limits.second) {
      throw std::invalid_argument("gripper_open_width is outside the loaded robot model limits");
    }
    RCLCPP_INFO(
      node_->get_logger(), "Gripper width limits [%.3f, %.3f] m; opening %.3f m",
      width_limits.first, width_limits.second, open_width_);
    scene_ = std::make_unique<Scene>(node_, tf_buffer_, frame_, options.end_effector_link);
    objects_sub_ = node_->create_subscription<my_robot_interface::msg::DetectedObjectArray>(
      parameter<std::string>(node_, "objects_topic", "/perception/objects"), 10,
      [this](my_robot_interface::msg::DetectedObjectArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(observations_mutex_);
        observations_ = msg;
      });

    const auto name = parameter<std::string>(node_, "action_name", "/manipulation/pick_place");
    server_ = rclcpp_action::create_server<Action>(
      node_, name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) {
        if (!validGoal(*goal) || holding_ || busy_.exchange(true)) {
          RCLCPP_WARN(node_->get_logger(), "Rejected invalid/busy goal or object still held");
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) {
        // 当前运动调用为同步执行：接受取消，在当前动作结束后停止后续动作。
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> handle) {
        if (worker_.joinable()) {
          worker_.join();
        }
        worker_ = std::thread([this, handle]() {execute(handle);});
      });
    RCLCPP_INFO(node_->get_logger(), "PickPlace executor ready: %s", name.c_str());
    named_server_ = rclcpp_action::create_server<NamedAction>(
      node_,
      parameter<std::string>(node_, "move_to_named_action", "/manipulation/move_to_named"),
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const NamedAction::Goal> goal) {
        if (goal->target.empty() || busy_.exchange(true)) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<NamedHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<NamedHandle> handle) {
        if (worker_.joinable()) {worker_.join();}
        worker_ = std::thread([this, handle]() {executeNamed(handle);});
      });

    // If an action stops with a carried object, release it explicitly after recovery.
    recovery_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    release_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/manipulation/release_held_object",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (busy_.exchange(true)) {
          response->message = "Executor is busy";
          return;
        }
        try {
          if (!motion_->moveGripperToWidth(open_width_)) {
            throw std::runtime_error("Opening gripper failed");
          }
          if (scene_->holding()) {scene_->release();}
          holding_ = false;
          scene_->restoreTouch();
          response->success = true;
          response->message = "Gripper opened; planning-scene attachment released";
        } catch (const std::exception & error) {
          response->message = error.what();
        }
        busy_ = false;
      }, rmw_qos_profile_services_default, recovery_group_);
  }

  ~PickPlaceExecutor()
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  // Cache perception callbacks; apply a consistent snapshot before each motion.
  void syncScene()
  {
    my_robot_interface::msg::DetectedObjectArray::ConstSharedPtr observations;
    {
      std::lock_guard<std::mutex> lock(observations_mutex_);
      observations = observations_;
    }
    scene_->sync(observations);
  }

  void executeNamed(const std::shared_ptr<NamedHandle> & handle)
  {
    auto result = std::make_shared<NamedAction::Result>();
    try {
      syncScene();
      if (!handle->is_canceling()) {
        result->success = motion_->moveToNamedTarget(handle->get_goal()->target);
      }
      result->message = result->success ? "Named target reached" : "Named motion failed";
    } catch (const std::exception & error) {
      result->message = error.what();
    }
    if (rclcpp::ok()) {
      if (handle->is_canceling()) {
        result->success = false;
        result->message = "Canceled after current named motion";
        handle->canceled(result);
      } else if (result->success) {
        handle->succeed(result);
      } else {
        handle->abort(result);
      }
    }
    busy_ = false;
  }

  bool validGoal(const Action::Goal & goal) const
  {
    const auto pose_valid = [](const geometry_msgs::msg::PoseStamped & p) {
        const auto & v = p.pose.position;
        const auto & q = p.pose.orientation;
        return !p.header.frame_id.empty() && std::isfinite(v.x) && std::isfinite(v.y) &&
               std::isfinite(v.z) && std::isfinite(q.x) && std::isfinite(q.y) &&
               std::isfinite(q.z) && std::isfinite(q.w) &&
               std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w)) > 1e-9;
      };
    const auto & d = goal.object_dimensions;
    return !goal.object_id.empty() && pose_valid(goal.object_pose) && pose_valid(goal.place_pose) &&
           std::isfinite(d.x) && d.x > 0 && std::isfinite(d.y) && d.y > 0 &&
           std::isfinite(d.z) && d.z > 0;
  }

  geometry_msgs::msg::PoseStamped inBase(const geometry_msgs::msg::PoseStamped & pose)
  {
    auto result = pose.header.frame_id == frame_ ? pose :
      tf_buffer_.transform(pose, frame_, tf2::durationFromSec(tf_timeout_));
    tf2::Quaternion q;
    tf2::fromMsg(result.pose.orientation, q);
    q.normalize();
    result.pose.orientation = tf2::toMsg(q);
    return result;
  }

  bool interrupted(const std::shared_ptr<GoalHandle> & handle) const
  {
    return !rclcpp::ok() || handle->is_canceling();
  }

  void execute(const std::shared_ptr<GoalHandle> & handle)
  {
    auto result = std::make_shared<Action::Result>();
    auto feedback = std::make_shared<Action::Feedback>();
    const auto stage = [&](uint8_t value, const std::string & message) {
        feedback->stage = value;
        feedback->message = message;
        if (rclcpp::ok()) {
          handle->publish_feedback(feedback);
        }
      };
    const auto step = [&](const std::string & name, const std::function<bool()> & operation) {
        if (interrupted(handle)) {
          throw std::runtime_error("Canceled before " + name);
        }
        if (!operation()) {
          throw std::runtime_error(name + " failed");
        }
      };
    const auto move = [&](const tf2::Vector3 & tcp, bool cartesian) {
        return motion_->moveToPose(toolPose(tcp, tool_rotation_, tool_to_tcp_, frame_), cartesian);
      };
    const auto grip = [&](double width) {
        return motion_->moveGripperToWidth(width);
      };

    try {
      stage(Action::Feedback::PREPARING, "Preparing object and tool targets");
      const auto goal = handle->get_goal();
      const auto stamp = rclcpp::Time(
        goal->object_pose.header.stamp,
        node_->get_clock()->get_clock_type());
      if (stamp.nanoseconds() != 0 && max_age_ > 0) {
        const double age = (node_->now() - stamp).seconds();
        if (age < -0.1 || age > max_age_) {
          throw std::runtime_error("Object observation is stale or uses a different clock");
        }
      }
      const auto object = inBase(goal->object_pose);
      const auto place = inBase(goal->place_pose);
      const double width = graspWidth(object.pose, goal->object_dimensions, tool_rotation_);
      if (width + 2 * pad_thickness_ >= open_width_) {
        throw std::runtime_error("Object is wider than the clear opening between gripper pads");
      }
      const tf2::Vector3 pick = tf2::Vector3(
        object.pose.position.x,
        object.pose.position.y, object.pose.position.z) + offset_;
      const tf2::Vector3 put = tf2::Vector3(
        place.pose.position.x,
        place.pose.position.y, place.pose.position.z + clearance_) + offset_;
      // 固定工具朝向下，搬运不会改变物块姿态。
      tf2::Quaternion object_q, place_q;
      tf2::fromMsg(object.pose.orientation, object_q);
      tf2::fromMsg(place.pose.orientation, place_q);
      if (object_q.angleShortestPath(place_q) > 0.01) {
        throw std::runtime_error("This executor requires matching pick/place object orientation");
      }
      const auto pre_pick = pick + tf2::Vector3(0, 0, approach_);
      const double safe_z = std::max({transport_z_, pick.z() + lift_, put.z() + retreat_});
      const tf2::Vector3 lifted(pick.x(), pick.y(), safe_z);
      const tf2::Vector3 pre_place(put.x(), put.y(), safe_z);

      syncScene();
      scene_->prepare(goal->object_id, object, goal->object_dimensions);

      stage(Action::Feedback::PICKING, "Opening, approaching and gripping " + goal->object_id);
      step("Open gripper", [&]() {return grip(open_width_);});
      step("Move to pre-grasp", [&]() {return move(pre_pick, false);});
      step("Descend to grasp", [&]() {return move(pick, true);});
      step(
        "Close gripper", [&]() {
          // The MoveIt attachment is a planning model, not proof of a physical grasp.
          scene_->attach();
          holding_ = true;
          return grip(2 * pad_thickness_ + std::max(0.0, width - compression_));
        });

      stage(Action::Feedback::TRANSFERRING, "Lifting and transferring");
      step("Lift object", [&]() {return move(lifted, true);});
      step("Transfer object", [&]() {return move(pre_place, false);});

      stage(Action::Feedback::PLACING, "Lowering, releasing and retreating");
      step("Descend to place", [&]() {return move(put, true);});
      step(
        "Release object", [&]() {
          if (!grip(open_width_)) {return false;}
          scene_->release();
          holding_ = false;
          return true;
        });
      step("Retreat", [&]() {return move(put + tf2::Vector3(0, 0, retreat_), true);});
      result->success = true;
      result->message = "Motion sequence completed; object placement needs visual verification";
    } catch (const std::exception & error) {
      result->message = error.what();
      RCLCPP_WARN(node_->get_logger(), "%s", error.what());
    }
    try {
      scene_->restoreTouch();
    } catch (const std::exception & error) {
      result->success = false;
      result->message += "; restoring collision checks failed: " + std::string(error.what());
    }
    if (rclcpp::ok()) {
      if (handle->is_canceling()) {
        result->success = false;
        result->message = "Canceled after current motion; gripper state retained";
        handle->canceled(result);
      } else if (result->success) {
        handle->succeed(result);
      } else {
        handle->abort(result);
      }
    }
    busy_ = false;
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<Motion> motion_;
  std::unique_ptr<Scene> scene_;
  std::mutex observations_mutex_;
  my_robot_interface::msg::DetectedObjectArray::ConstSharedPtr observations_;
  rclcpp::Subscription<my_robot_interface::msg::DetectedObjectArray>::SharedPtr objects_sub_;
  rclcpp::CallbackGroup::SharedPtr recovery_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_service_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp_action::Server<NamedAction>::SharedPtr named_server_;
  std::thread worker_;
  std::atomic<bool> busy_{false};
  std::atomic<bool> holding_{false};
  std::string frame_;
  tf2::Vector3 tool_to_tcp_, offset_;
  tf2::Quaternion tool_rotation_;
  double approach_, lift_, transport_z_, clearance_, retreat_;
  double open_width_, pad_thickness_, compression_, max_age_, tf_timeout_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "pick_place_executor",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread spin_thread([&]() {executor.spin();});
  int result = 0;
  try {
    PickPlaceExecutor server(node);
    spin_thread.join();
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "%s", error.what());
      result = 1;
    }
  }
  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return result;
}
