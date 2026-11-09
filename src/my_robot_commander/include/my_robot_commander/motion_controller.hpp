#ifndef MY_ROBOT_COMMANDER__MOTION_CONTROLLER_HPP_
#define MY_ROBOT_COMMANDER__MOTION_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace my_robot_commander
{

// 底层对上的同步 C++ 接口；无任务、视觉或碰撞场景管理逻辑。
// 调用期间需有 executor 处理传入节点的 ROS 回调，同一实例的运动串行执行。
// 长度为 m、角度为 rad；关节向量沿 MoveIt 组的变量顺序。
// true 仅表示 MoveIt 执行成功，不证明真机到位或夹住物体；输入/规划/执行被拒返回 false。
// 配置或 ROS/MoveIt 异常可向调用者抛出；不提供运动中取消。
// 当前执行器 Action 的取消在基本动作完成后生效。
class MotionController
{
public:
  struct Options
  {
    std::string arm_group = "arm";
    std::string gripper_group = "gripper";
    std::string reference_frame = "base_link";
    std::string end_effector_link;  // 空值沿用 MoveIt 配置。
    std::string gripper_open_target = "gripper_open";
    std::string gripper_close_target = "gripper_close";
    std::string gripper_width_joint = "gripper";  // Independent joint, total opening in metres.
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

  bool moveToNamedTarget(const std::string & name);  // 仅 arm 组。
  bool moveToJoints(const std::vector<double> & joints);
  bool moveToPose(const geometry_msgs::msg::PoseStamped & pose, bool cartesian = false);
  bool moveToPose(
    const std::array<double, 3> & position, const std::array<double, 3> & rpy,
    bool cartesian = false);
  bool moveGripperToNamedTarget(const std::string & name);
  bool moveGripperToJoints(const std::vector<double> & joints);
  bool moveGripperToWidth(double width);  // 绝对总开口，0.04 表示 40 mm。
  std::pair<double, double> gripperWidthLimits() const;  // MoveIt 模型中的总开口范围，m。
  bool openGripper();
  bool closeGripper();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace my_robot_commander
#endif  // MY_ROBOT_COMMANDER__MOTION_CONTROLLER_HPP_
