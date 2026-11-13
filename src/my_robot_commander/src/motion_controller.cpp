#include "my_robot_commander/motion_controller.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <mutex>
#include <stdexcept>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

namespace my_robot_commander
{
namespace
{
bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}
}  // namespace

struct MotionController::Impl
{
  using Group = moveit::planning_interface::MoveGroupInterface;
  using Execute = moveit_msgs::action::ExecuteTrajectory;
  using Handle = rclcpp_action::ClientGoalHandle<Execute>;
  rclcpp::Node::SharedPtr node;
  Options options;
  std::unique_ptr<Group> arm;
  std::unique_ptr<Group> gripper;
  std::mutex mutex;
  rclcpp_action::Client<Execute>::SharedPtr execute_client;
  std::string error;
  bool execution_unknown = false;
  bool before_execution = true;

  Impl(const rclcpp::Node::SharedPtr & input_node, const Options & input_options)
  : node(input_node), options(input_options)
  {
    if (!node) {
      throw std::invalid_argument("MotionController requires a ROS node");
    }
    if (!(options.velocity_scaling > 0 && options.velocity_scaling <= 1 &&
      options.acceleration_scaling > 0 && options.acceleration_scaling <= 1 &&
      std::isfinite(options.planning_time) && options.planning_time > 0 &&
      std::isfinite(options.cartesian_step) && options.cartesian_step > 0 &&
      std::isfinite(options.execution_timeout) && options.execution_timeout > 0 &&
      std::isfinite(options.cancel_timeout) && options.cancel_timeout > 0))
    {
      throw std::invalid_argument("Invalid motion scaling, planning time or Cartesian step");
    }
    arm = std::make_unique<Group>(
      node, Group::Options(
        options.arm_group, "robot_description", node->get_namespace()));
    gripper = std::make_unique<Group>(
      node, Group::Options(
        options.gripper_group, "robot_description", node->get_namespace()));
    for (auto * group : {arm.get(), gripper.get()}) {
      group->setMaxVelocityScalingFactor(options.velocity_scaling);
      group->setMaxAccelerationScalingFactor(options.acceleration_scaling);
      group->setPlanningTime(options.planning_time);
    }
    arm->setPoseReferenceFrame(options.reference_frame);
    if (!options.end_effector_link.empty() &&
      !arm->setEndEffectorLink(options.end_effector_link))
    {
      throw std::invalid_argument("Invalid end effector link: " + options.end_effector_link);
    }
    execute_client = rclcpp_action::create_client<Execute>(node, "execute_trajectory");
  }

  bool fail(const std::string & message)
  {
    error = message;
    RCLCPP_WARN(node->get_logger(), "%s", message.c_str());
    return false;
  }

  bool interrupted() const
  {
    return !rclcpp::ok() || (options.interrupted && options.interrupted());
  }

  template<typename Future>
  bool wait(Future & future, double seconds, bool cancelable = true) const
  {
    using namespace std::chrono_literals;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (rclcpp::ok() && (!cancelable || !interrupted()) &&
      std::chrono::steady_clock::now() < deadline)
    {
      if (future.wait_for(20ms) == std::future_status::ready) {return true;}
    }
    return future.wait_for(0ms) == std::future_status::ready;
  }

  bool execute(const moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    before_execution = false;
    using namespace std::chrono_literals;
    if (execution_unknown) {return fail("Previous execution state is unknown; recovery required");}
    if (interrupted()) {return fail("Motion canceled before execution");}
    if (!execute_client->wait_for_action_server(2s)) {
      return fail("ExecuteTrajectory server unavailable");
    }
    const auto cancel_late = std::make_shared<std::atomic<bool>>(false);
    rclcpp_action::Client<Execute>::SendGoalOptions callbacks;
    callbacks.goal_response_callback =
      [client = execute_client, cancel_late](Handle::SharedPtr goal) {
        if (goal && *cancel_late) {client->async_cancel_goal(goal);}
      };
    Execute::Goal request;
    request.trajectory = trajectory;
    // Latch before sending: exceptions or lost responses must never permit a
    // second trajectory while the first one's state is unknown.
    execution_unknown = true;
    auto response = execute_client->async_send_goal(request, callbacks);
    if (!wait(response, 5.0)) {
      *cancel_late = true;
      // Cover a response arriving between wait() and setting cancel_late.
      if (response.wait_for(0ms) == std::future_status::ready) {
        if (const auto late_handle = response.get()) {
          execute_client->async_cancel_goal(late_handle);
        }
      }
      return fail("ExecuteTrajectory response missing; execution state unknown");
    }
    const auto handle = response.get();
    if (!handle) {
      execution_unknown = false;
      return fail("ExecuteTrajectory rejected the trajectory");
    }
    auto result = execute_client->async_get_result(handle);
    const bool canceled = !wait(result, options.execution_timeout);
    if (canceled) {
      execute_client->async_cancel_goal(handle);
      if (!wait(result, options.cancel_timeout, false)) {
        return fail("Trajectory cancellation unconfirmed; execution state unknown");
      }
    }
    const auto completed = result.get();
    if (!completed.result) {
      return fail("ExecuteTrajectory returned no result; execution state unknown");
    }
    if (completed.code == rclcpp_action::ResultCode::UNKNOWN) {
      return fail("ExecuteTrajectory status unknown; recovery required");
    }
    execution_unknown = false;
    if (canceled || interrupted()) {
      return fail("Trajectory interrupted; no subsequent motion sent");
    }
    if (completed.code != rclcpp_action::ResultCode::SUCCEEDED ||
      completed.result->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      return fail(
        "Trajectory execution failed, MoveIt code: " +
        std::to_string(completed.result->error_code.val));
    }
    error.clear();
    return true;
  }

  bool planAndExecute(Group & group)
  {
    before_execution = true;
    if (execution_unknown) {return fail("Previous execution state is unknown; recovery required");}
    if (interrupted()) {return fail("Motion canceled before planning");}
    Group::Plan plan;
    const auto planned = group.plan(plan);
    if (planned != moveit::core::MoveItErrorCode::SUCCESS) {
      return fail("Planning failed, MoveIt code: " + std::to_string(planned.val));
    }
    return execute(plan.trajectory_);
  }

  bool namedTarget(Group & group, const std::string & name)
  {
    before_execution = true;
    error.clear();
    group.clearPoseTargets();
    group.setStartStateToCurrentState();
    if (!group.setNamedTarget(name)) {
      return fail("Unknown named target: " + name);
    }
    return planAndExecute(group);
  }

  std::pair<double, double> widthLimits() const
  {
    const auto model = gripper->getRobotModel();
    const auto * group = model->getJointModelGroup(options.gripper_group);
    const auto & joints = group->getActiveJointModelNames();
    if (joints.size() != 1 || joints.front() != options.gripper_width_joint ||
      model->getJointModel(joints.front())->getType() != moveit::core::JointModel::PRISMATIC)
    {
      throw std::invalid_argument("Width control requires one prismatic gripper joint");
    }
    const auto & bounds = model->getVariableBounds(options.gripper_width_joint);
    if (!bounds.position_bounded_ || !std::isfinite(bounds.min_position_) ||
      !std::isfinite(bounds.max_position_) || bounds.min_position_ >= bounds.max_position_)
    {
      throw std::invalid_argument("Gripper width requires finite model position limits");
    }
    return {bounds.min_position_, bounds.max_position_};
  }
};

MotionController::MotionController(const rclcpp::Node::SharedPtr & node)
: MotionController(node, Options{}) {}

MotionController::MotionController(const rclcpp::Node::SharedPtr & node, const Options & options)
: impl_(std::make_unique<Impl>(node, options)) {}

MotionController::~MotionController() = default;

bool MotionController::moveToNamedTarget(const std::string & name)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->namedTarget(*impl_->arm, name);
}

bool MotionController::moveToJoints(const std::vector<double> & joints)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->before_execution = true;
  auto & arm = *impl_->arm;
  if (joints.size() != arm.getVariableCount() ||
    !std::all_of(joints.begin(), joints.end(), [](double v) {return std::isfinite(v);}))
  {
    return impl_->fail("Invalid joint count or non-finite joint value");
  }
  arm.clearPoseTargets();
  arm.setStartStateToCurrentState();
  if (!arm.setJointValueTarget(joints)) {
    return impl_->fail("Joint target is outside configured bounds");
  }
  return impl_->planAndExecute(arm);
}

bool MotionController::moveToPose(
  const std::array<double, 3> & position, const std::array<double, 3> & rpy, bool cartesian)
{
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = impl_->options.reference_frame;
  target.pose.position.x = position[0];
  target.pose.position.y = position[1];
  target.pose.position.z = position[2];
  tf2::Quaternion q;
  q.setRPY(rpy[0], rpy[1], rpy[2]);
  target.pose.orientation.x = q.x();
  target.pose.orientation.y = q.y();
  target.pose.orientation.z = q.z();
  target.pose.orientation.w = q.w();
  return moveToPose(target, cartesian);
}

bool MotionController::moveToPose(const geometry_msgs::msg::PoseStamped & pose, bool cartesian)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->before_execution = true;
  auto target = pose;
  auto & q = target.pose.orientation;
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  if (target.header.frame_id.empty() || !finitePose(target.pose) ||
    !std::isfinite(norm) || norm < 1e-9)
  {
    return impl_->fail("Invalid pose, quaternion or empty frame_id");
  }
  q.x /= norm; q.y /= norm; q.z /= norm; q.w /= norm;
  auto & arm = *impl_->arm;
  arm.clearPoseTargets();
  arm.setStartStateToCurrentState();
  bool success = false;
  if (!cartesian) {
    if (arm.setPoseTarget(target)) {
      success = impl_->planAndExecute(arm);
    } else {
      success = impl_->fail("MoveIt rejected pose target");
    }
  } else {
    // Cartesian API 的 waypoints 不含 Header，必须明确在统一参考系下输入。
    if (target.header.frame_id != impl_->options.reference_frame) {
      return impl_->fail("Cartesian target must be in " + impl_->options.reference_frame);
    }
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = arm.computeCartesianPath(
      {target.pose}, impl_->options.cartesian_step, 0.0, trajectory, true);
    if (fraction >= 1.0 - 1e-6 && !trajectory.joint_trajectory.points.empty()) {
      success = impl_->execute(trajectory);
    } else {
      success = impl_->fail("Cartesian path incomplete: " + std::to_string(fraction * 100) +
          "%; no partial trajectory executed (check IK, collision and target pose)");
    }
  }
  arm.clearPoseTargets();
  return success;
}

bool MotionController::moveGripperToNamedTarget(const std::string & name)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->namedTarget(*impl_->gripper, name);
}

bool MotionController::openGripper()
{
  return moveGripperToNamedTarget(impl_->options.gripper_open_target);
}

bool MotionController::moveGripperToJoints(const std::vector<double> & joints)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->before_execution = true;
  auto & gripper = *impl_->gripper;
  if (joints.size() != gripper.getVariableCount() ||
    !std::all_of(joints.begin(), joints.end(), [](double v) {return std::isfinite(v);}))
  {
    return impl_->fail("Invalid gripper joint target");
  }
  gripper.setStartStateToCurrentState();
  if (!gripper.setJointValueTarget(joints)) {
    return impl_->fail("Gripper target is outside configured bounds");
  }
  return impl_->planAndExecute(gripper);
}

bool MotionController::closeGripper()
{
  return moveGripperToNamedTarget(impl_->options.gripper_close_target);
}

bool MotionController::moveGripperToWidth(double width)
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  impl_->before_execution = true;
  auto & gripper = *impl_->gripper;
  const auto limits = impl_->widthLimits();
  if (!std::isfinite(width) || width < limits.first || width > limits.second) {
    return impl_->fail("Gripper width is outside model limits");
  }
  gripper.setStartStateToCurrentState();
  // Set the independent joint by name; MoveIt updates its mimic fingers.
  if (!gripper.setJointValueTarget(
      std::map<std::string, double>{{impl_->options.gripper_width_joint, width}}))
  {
    return impl_->fail("Gripper width is outside configured bounds");
  }
  const bool executed = impl_->planAndExecute(gripper);
  if (const auto state = gripper.getCurrentState(0.2)) {
    const double actual = state->getVariablePosition(impl_->options.gripper_width_joint);
    RCLCPP_INFO(impl_->node->get_logger(),
      "Gripper target %.1f mm, feedback %.1f mm, target-feedback %.1f mm; command %s (grasp unverified)",
      width * 1000, actual * 1000, (width - actual) * 1000, executed ? "completed" : "failed");
  }
  return executed;
}

std::pair<double, double> MotionController::gripperWidthLimits() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->widthLimits();
}

void MotionController::validateLinks(const std::vector<std::string> & links) const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  for (const auto & link : links) {
    if (link.empty() || !impl_->arm->getRobotModel()->hasLinkModel(link)) {
      throw std::invalid_argument("Unknown robot link: " + link);
    }
  }
}

std::string MotionController::lastError() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->error;
}

bool MotionController::lastFailureBeforeExecution() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->before_execution && !impl_->error.empty() && !impl_->execution_unknown;
}

bool MotionController::executionStateKnown() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return !impl_->execution_unknown;
}

}  // namespace my_robot_commander
