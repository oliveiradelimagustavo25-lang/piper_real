#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
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
#include <my_robot_interface/failure_policy.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "my_robot_commander/motion_controller.hpp"
#include "my_robot_commander/pick_place_scene.hpp"

namespace policy = my_robot_interface::failure;

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
    if (values.size() != 3 || !std::all_of(values.begin(), values.end(),
      [](double v) {return std::isfinite(v);}))
    {
      throw std::invalid_argument("Expected an xyz or rpy parameter with 3 finite values");
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
  using Failure = policy::Failure;
  using FailureError = policy::Error;
  // Attached is a planning state, not a physical grasp-confirmation signal.
  enum class GraspState {Empty, NeedsRecovery, Attached, Released};

public:
  explicit PickPlaceExecutor(const rclcpp::Node::SharedPtr & node)
  : node_(node), tf_buffer_(node->get_clock())
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
    options.execution_timeout = parameter(node_, "execution_timeout", options.execution_timeout);
    options.cancel_timeout = parameter(node_, "cancel_timeout", options.cancel_timeout);
    options.interrupted = [this]() {return cancel_requested_.load();};
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
    compression_ = parameter(node_, "gripper_compression", 0.004);
    max_age_ = parameter(node_, "max_target_age", 5.0);
    tf_timeout_ = parameter(node_, "tf_timeout", 0.3);
    const std::vector<double> distances{approach_, lift_, transport_z_, clearance_, retreat_,
      max_age_, tf_timeout_};
    if (!std::all_of(distances.begin(), distances.end(), [](double v) {return std::isfinite(v);}) ||
      max_age_ <= 0 || tf_timeout_ < 0 || frame_.empty() ||
      !std::isfinite(open_width_) || !std::isfinite(pad_thickness_) ||
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
    const auto startup_timeout = parameter(node_, "startup_timeout", 60.0);
    if (!std::isfinite(startup_timeout) || startup_timeout <= 0) {
      throw std::invalid_argument("startup_timeout must be positive and finite");
    }
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(startup_timeout);
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> descriptions;
    for (const std::string name : {"robot_description", "robot_description_semantic"}) {
      if (parameter<std::string>(node_, name, "").empty()) {
        descriptions.push_back(node_->create_subscription<std_msgs::msg::String>(
          name, rclcpp::QoS(1).transient_local(),
          [node = node_, name](std_msgs::msg::String::ConstSharedPtr message) {
            if (!message->data.empty()) {node->set_parameter(rclcpp::Parameter(name, message->data));}
          }));
      }
    }
    while (!planning_scene->wait_for_service(std::chrono::seconds(1)) ||
      node_->get_parameter("robot_description").as_string().empty() ||
      node_->get_parameter("robot_description_semantic").as_string().empty())
    {
      if (!rclcpp::ok()) {
        throw std::runtime_error("Shutdown while waiting for MoveIt");
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("Timed out waiting for MoveIt scene/model; start the control layer");
      }
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Waiting for control stack: planning scene and robot model");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    options.startup_timeout = std::max(0.001,
      std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count());
    motion_ = std::make_unique<Motion>(node_, options);
    motion_->validateLinks({options.end_effector_link});
    const auto touch_links = parameter<std::vector<std::string>>(
      node_, "touch_links", {"gripper_link1", "gripper_link2"});
    if (touch_links.empty()) {throw std::invalid_argument("touch_links must not be empty");}
    motion_->validateLinks(touch_links);
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
        const int64_t stamp = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000 +
          msg->header.stamp.nanosec;
        if (observations_after_ns_ != 0 && stamp <= observations_after_ns_) {return;}
        observations_ = msg;
      });

    const auto name = parameter<std::string>(node_, "action_name", "/manipulation/pick_place");
    server_ = rclcpp_action::create_server<Action>(
      node_, name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) {
        if (busy_ || !scene_ready_ || !validGoal(*goal) || grasp_state_ != GraspState::Empty ||
          !motion_->executionStateKnown() || busy_.exchange(true))
        {
          RCLCPP_WARN(node_->get_logger(), "Rejected invalid/busy goal or pending grasp recovery");
          return rclcpp_action::GoalResponse::REJECT;
        }
        cancel_requested_ = false;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle>) {
        cancel_requested_ = true;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> handle) {
        if (worker_.joinable()) {
          worker_.join();
        }
        worker_ = std::thread([this, handle]() {execute(handle);});
      });
    named_server_ = rclcpp_action::create_server<NamedAction>(
      node_,
      parameter<std::string>(node_, "move_to_named_action", "/manipulation/move_to_named"),
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const NamedAction::Goal> goal) {
        if (busy_ || !scene_ready_ || goal->target.empty() || grasp_state_ == GraspState::NeedsRecovery ||
          grasp_state_ == GraspState::Released || !motion_->executionStateKnown() || busy_.exchange(true))
        {
          return rclcpp_action::GoalResponse::REJECT;
        }
        cancel_requested_ = false;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<NamedHandle>) {
        cancel_requested_ = true;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
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
        cancel_requested_ = false;
        try {
          if (!motion_->executionStateKnown()) {
            throw std::runtime_error("Motion state unknown; verify the controller before gripper recovery");
          }
          if (grasp_state_ != GraspState::Released) {
            grasp_state_ = GraspState::NeedsRecovery;
            if (!motion_->moveGripperToWidth(open_width_)) {
              throw std::runtime_error("Opening gripper failed; explicit recovery still required");
            }
            grasp_state_ = GraspState::Released;
          }
          if (scene_->holding()) {scene_->release();}
          scene_->restoreTouch();
          grasp_state_ = GraspState::Empty;
          scene_ready_ = true;
          response->success = true;
          response->message = "Gripper opened; planning-scene attachment released";
        } catch (const std::exception & error) {
          response->message = error.what();
        }
        busy_ = false;
      }, rmw_qos_profile_services_default, recovery_group_);
    clear_scene_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "/manipulation/clear_observed_objects",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (busy_.exchange(true)) {
          response->message = "Executor is busy";
          return;
        }
        try {
          if (grasp_state_ != GraspState::Empty || !motion_->executionStateKnown()) {
            throw std::runtime_error("Grasp recovery is pending; objects cannot be cleared");
          }
          const auto count = scene_->clearObserved();
          {
            std::lock_guard<std::mutex> lock(observations_mutex_);
            observations_.reset();
            observations_after_ns_ = node_->now().nanoseconds();
          }
          response->success = true;
          response->message = "Removed " + std::to_string(count) +
            " managed objects; fixed environment retained; new observations may add objects again";
        } catch (const std::exception & error) {
          response->message = error.what();
        }
        busy_ = false;
      }, rmw_qos_profile_services_default, recovery_group_);
    // Create the listener last: failed startup validation must not wait for a
    // TF thread while the ROS context is still active during stack unwinding.
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
    RCLCPP_INFO(node_->get_logger(), "PickPlace executor ready: %s", name.c_str());
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

  Failure snapshot(Failure failure, uint8_t stage, const std::string & id = "")
  {
    failure.stage = stage;
    failure.object_id = id;
    if (failure.origin != Failure::MOTION) {
      failure.command_state = motion_->executionStateKnown() ? Failure::NOT_SENT : Failure::UNKNOWN;
    }
    const auto grasp = grasp_state_.load();
    failure.payload_state = (grasp == GraspState::Empty || grasp == GraspState::Released) ?
      Failure::CLEAR : Failure::MAY_HOLD;
    failure.scene_state = scene_ready_ ? Failure::CONSISTENT : Failure::UNCERTAIN;
    return failure;
  }

  void executeNamed(const std::shared_ptr<NamedHandle> & handle)
  {
    auto result = std::make_shared<NamedAction::Result>();
    try {
      syncScene();
      if (!handle->is_canceling()) {
        result->success = motion_->moveToNamedTarget(handle->get_goal()->target);
        result->failure = motion_->lastFailure();
      } else {
        result->failure = policy::makeFailure(
          Failure::EXECUTION_FAILED, Failure::EXECUTOR, "Canceled before named motion");
      }
    } catch (const FailureError & error) {
      result->failure = error.failure;
    } catch (const tf2::TransformException & error) {
      result->failure = policy::makeFailure(Failure::TF_UNAVAILABLE, Failure::SCENE, error.what());
    } catch (const std::exception & error) {
      result->failure = policy::makeFailure(Failure::UNCLASSIFIED, Failure::EXECUTOR, error.what());
    }
    if (result->failure.code == Failure::SCENE_INCONSISTENT) {scene_ready_ = false;}
    result->failure = snapshot(result->failure, Failure::PLAN);
    result->message = result->success ? "Named target reached" : result->failure.detail;
    if (rclcpp::ok()) {
      if (handle->is_canceling()) {
        result->success = false;
        if (result->failure.code == Failure::NONE) {
          result->failure.code = Failure::EXECUTION_FAILED;
          result->failure.detail = "Named motion cancellation completed";
          result->message = result->failure.detail;
        }
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
    uint8_t current_stage = Failure::VALIDATE;
    auto feedback = std::make_shared<Action::Feedback>();
    const auto stage = [&](uint8_t value, const std::string & message) {
        feedback->stage = value;
        feedback->message = message;
        RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
        if (rclcpp::ok()) {
          handle->publish_feedback(feedback);
        }
      };
    const auto step = [&](const std::string & name, uint8_t value,
        const std::function<bool()> & operation) {
        current_stage = value;
        if (interrupted(handle)) {
          throw FailureError(policy::makeFailure(
            Failure::EXECUTION_FAILED, Failure::EXECUTOR, "Canceled before " + name));
        }
        stage(feedback->stage, name);
        if (!operation()) {
          auto failure = motion_->lastFailure();
          failure.detail = name + " failed: " + failure.detail;
          throw FailureError(failure);
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
      // Transform at the observation time, then execute the fixed base-frame snapshot.
      const auto goal = handle->get_goal();
      const auto stamp = rclcpp::Time(
        goal->object_pose.header.stamp,
        node_->get_clock()->get_clock_type());
      if (stamp.nanoseconds() != 0 && max_age_ > 0) {
        const double age = (node_->now() - stamp).seconds();
        if (age < -0.1 || age > max_age_) {
          auto failure = policy::makeFailure(Failure::OBSERVATION_STALE,
            Failure::EXECUTOR, "Object observation is stale or uses a different clock");
          failure.target_specific = true;
          throw FailureError(failure);
        }
      }
      const auto object = inBase(goal->object_pose);
      const auto place = inBase(goal->place_pose);
      const double width = graspWidth(object.pose, goal->object_dimensions, tool_rotation_);
      if (width + 2 * pad_thickness_ >= open_width_) {
        auto failure = policy::makeFailure(
          Failure::TARGET_UNAVAILABLE, Failure::EXECUTOR, "Object is wider than the clear opening between gripper pads");
        failure.target_specific = true;
        throw FailureError(failure);
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
        auto failure = policy::makeFailure(
          Failure::TARGET_UNAVAILABLE, Failure::EXECUTOR, "This executor requires matching pick/place object orientation");
        failure.target_specific = true;
        throw FailureError(failure);
      }
      const auto pre_pick = pick + tf2::Vector3(0, 0, approach_);
      const double minimum_z = std::max(pick.z() + lift_, put.z() + retreat_);
      const double safe_z = std::max(transport_z_, minimum_z);
      // Lift the requested clearance vertically; reach transport height during
      // the subsequent collision-checked transfer, not at maximum reach.
      const tf2::Vector3 lifted(pick.x(), pick.y(), pick.z() + lift_);

      current_stage = Failure::PREPARE;
      syncScene();
      scene_->prepare(goal->object_id, object, goal->object_dimensions);

      stage(Action::Feedback::PICKING, "Opening, approaching and gripping " + goal->object_id);
      step("Open gripper", Failure::OPEN, [&]() {return grip(open_width_);});
      step("Move to pre-grasp", Failure::PREGRASP, [&]() {return move(pre_pick, false);});
      step("Descend to grasp", Failure::DESCEND, [&]() {return move(pick, true);});
      step(
        "Close gripper", Failure::CLOSE, [&]() {
          // A failed/partial close can still contact an object. Never silently
          // mark it empty or attach before the close command has completed.
          grasp_state_ = GraspState::NeedsRecovery;
          const double target_width = 2 * pad_thickness_ + std::max(0.0, width - compression_);
          RCLCPP_INFO(node_->get_logger(),
            "Close gripper: projected object %.1f mm, pad per side %.1f mm, compression %.1f mm -> target %.1f mm",
            width * 1000, pad_thickness_ * 1000, compression_ * 1000, target_width * 1000);
          if (!grip(target_width)) {
            if (motion_->lastFailure().command_state == Failure::NOT_SENT) {
              grasp_state_ = GraspState::Empty;  // Planning never reached contact.
            }
            return false;
          }
          scene_->attach();
          grasp_state_ = GraspState::Attached;
          return true;
        });

      stage(Action::Feedback::TRANSFERRING, "Lifting and transferring");
      step("Lift object", Failure::LIFT, [&]() {return move(lifted, true);});
      step("Transfer object", Failure::TRANSFER, [&]() {
          for (const double z : {safe_z, (safe_z + minimum_z) / 2, minimum_z}) {
            if (move(tf2::Vector3(put.x(), put.y(), z), false)) {return true;}
            const auto failure = motion_->lastFailure();
            if (failure.code != Failure::PLAN_FAILED || failure.command_state != Failure::NOT_SENT ||
              interrupted(handle) || z <= minimum_z + 1e-6)
            {
              return false;
            }
            RCLCPP_INFO(node_->get_logger(), "Transfer height %.3f m unavailable; trying a lower height", z);
          }
          return false;
        });

      stage(Action::Feedback::PLACING, "Lowering, releasing and retreating");
      step("Descend to place", Failure::PLACE, [&]() {return move(put, true);});
      step(
        "Release object", Failure::RELEASE, [&]() {
          if (!grip(open_width_)) {return false;}
          grasp_state_ = GraspState::Released;
          scene_->release();
          return true;
        });
      step("Retreat", Failure::RETREAT, [&]() {return move(put + tf2::Vector3(0, 0, retreat_), true);});
      result->success = true;
      result->message = "Motion sequence completed; object placement needs visual verification";
      result->failure = policy::makeFailure(Failure::NONE, Failure::EXECUTOR, "");
    } catch (const FailureError & error) {
      result->failure = error.failure;
    } catch (const tf2::TransformException & error) {
      result->failure = policy::makeFailure(
        Failure::TF_UNAVAILABLE, Failure::SCENE, error.what());
    } catch (const std::invalid_argument & error) {
      result->failure = policy::makeFailure(Failure::INPUT_INVALID, Failure::EXECUTOR, error.what());
      result->failure.target_specific = true;
    } catch (const std::exception & error) {
      result->failure = policy::makeFailure(Failure::UNCLASSIFIED, Failure::EXECUTOR, error.what());
    }
    if (result->failure.code == Failure::SCENE_INCONSISTENT) {scene_ready_ = false;}
    try {
      scene_->restoreTouch();
      if (grasp_state_ == GraspState::Released && !scene_->holding()) {
        grasp_state_ = GraspState::Empty;
      }
    } catch (const std::exception & error) {
      scene_ready_ = false;
      if (result->success) {
        current_stage = Failure::CLEANUP;
        result->failure = policy::makeFailure(
          Failure::SCENE_INCONSISTENT, Failure::SCENE, error.what());
      } else {
        result->failure.detail += "; restoring collision checks failed: " + std::string(error.what());
      }
      result->success = false;
    }
    result->failure = snapshot(result->failure, current_stage, handle->get_goal()->object_id);
    if (!result->success) {
      result->message = result->failure.detail;
      const auto decision = policy::decideStep(result->failure);
      RCLCPP_WARN(node_->get_logger(), "PickPlace code=%u stage=%u decision=%u: %s",
        result->failure.code, current_stage, static_cast<unsigned>(decision), result->message.c_str());
    }
    if (rclcpp::ok()) {
      if (handle->is_canceling()) {
        result->success = false;
        result->message = "Cancellation handled; gripper state retained; " + result->message;
        if (result->failure.code == Failure::NONE) {
          result->failure.code = Failure::EXECUTION_FAILED;
          result->failure.detail = result->message;
        }
        if (grasp_state_ != GraspState::Empty) {result->message += "; explicit recovery required";}
        handle->canceled(result);
      } else if (result->success) {
        handle->succeed(result);
      } else {
        if (grasp_state_ != GraspState::Empty) {
          result->message += "; explicit grasp recovery required, no automatic release performed";
        }
        handle->abort(result);
      }
    }
    busy_ = false;
  }

  rclcpp::Node::SharedPtr node_;
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<Motion> motion_;
  std::unique_ptr<Scene> scene_;
  std::mutex observations_mutex_;
  my_robot_interface::msg::DetectedObjectArray::ConstSharedPtr observations_;
  int64_t observations_after_ns_ = 0;
  rclcpp::Subscription<my_robot_interface::msg::DetectedObjectArray>::SharedPtr objects_sub_;
  rclcpp::CallbackGroup::SharedPtr recovery_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_scene_service_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp_action::Server<NamedAction>::SharedPtr named_server_;
  std::thread worker_;
  std::atomic<bool> busy_{false};
  std::atomic<GraspState> grasp_state_{GraspState::Empty};
  std::atomic<bool> scene_ready_{true};
  std::atomic<bool> cancel_requested_{false};
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
