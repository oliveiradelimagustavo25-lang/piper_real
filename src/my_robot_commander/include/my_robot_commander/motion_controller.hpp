#ifndef MY_ROBOT_COMMANDER__MOTION_CONTROLLER_HPP_
#define MY_ROBOT_COMMANDER__MOTION_CONTROLLER_HPP_

#include <array>
#include <functional>
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
// true 表示轨迹执行成功；真机到位取决于底层真实状态及容差，不能证明夹住物体。
// 配置或 ROS/MoveIt 异常可抛出。取消通过 interrupted 请求，不等同于硬件急停。
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
    double execution_timeout = 120.0;  // s，单个轨迹的执行上限。
    double cancel_timeout = 5.0;       // s，取消后等待最终结果的上限。
    std::function<bool()> interrupted; // 由调用者提供，不能阻塞或发起运动。
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
  void validateLinks(const std::vector<std::string> & links) const;
  std::string lastError() const;
  bool lastFailureBeforeExecution() const;  // 最近一次失败是否发生在轨迹下发前。
  bool executionStateKnown() const;  // false 时禁止新运动和场景清理，需人工确认恢复。
  bool openGripper();
  bool closeGripper();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace my_robot_commander
#endif  // MY_ROBOT_COMMANDER__MOTION_CONTROLLER_HPP_
