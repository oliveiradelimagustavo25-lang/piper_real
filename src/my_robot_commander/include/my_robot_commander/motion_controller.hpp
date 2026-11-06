#ifndef MY_ROBOT_COMMANDER__MOTION_CONTROLLER_HPP_
#define MY_ROBOT_COMMANDER__MOTION_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace my_robot_commander
{

// 同步运动接口；调用期间需有 executor 处理传入节点的 ROS 回调。
// 同一实例的运动按顺序执行；true 表示轨迹执行成功，不代表已确认夹住物体。
class MotionController
{
public:
  struct Options
  {
    std::string arm_group = "arm";
    std::string gripper_group = "gripper";
    std::string reference_frame = "base_link";
    std::string end_effector_link;  // 空值沿用 MoveIt 配置。
    std::string gripper_open_target = "open";
    std::string gripper_close_target = "close";
    double velocity_scaling = 0.2;
    double acceleration_scaling = 0.2;
    double planning_time = 5.0;
    double cartesian_step = 0.005;  // m，直线路径插值步长。
  };

  explicit MotionController(const rclcpp::Node::SharedPtr & node);
  MotionController(const rclcpp::Node::SharedPtr & node, const Options & options);
  ~MotionController();

  MotionController(const MotionController &) = delete;
  MotionController & operator=(const MotionController &) = delete;

  bool moveToNamedTarget(const std::string & name);
  bool moveToJoints(const std::vector<double> & joints);
  bool moveToPose(const geometry_msgs::msg::PoseStamped & pose, bool cartesian = false);
  bool moveToPose(
    const std::array<double, 3> & position, const std::array<double, 3> & rpy,
    bool cartesian = false);
  bool moveGripperToNamedTarget(const std::string & name);
  bool moveGripperToJoints(const std::vector<double> & joints);
  bool openGripper();
  bool closeGripper();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace my_robot_commander
#endif  // MY_ROBOT_COMMANDER__MOTION_CONTROLLER_HPP_
