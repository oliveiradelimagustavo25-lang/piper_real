#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace my_robot_hardware
{
// A topic adapter, not a CAN driver. No enable, homing or firmware-mode commands.
// A private executor receives feedback. Only the controller thread writes exported states.
class AgxTopicSystem : public hardware_interface::SystemInterface
{
  using JointState = sensor_msgs::msg::JointState;
  using CallbackReturn = hardware_interface::CallbackReturn;
  using Return = hardware_interface::return_type;
  using Clock = std::chrono::steady_clock;
  using List = controller_manager_msgs::srv::ListControllers;
  using Switch = controller_manager_msgs::srv::SwitchController;
  using Stop = std_srvs::srv::Trigger;
  enum class State {Inactive, Active, Faulted, Stopped};

public:
  ~AgxTopicSystem() override
  {
    running_ = false;
    executor_.cancel();
    if (spin_thread_.joinable()) {spin_thread_.join();}
  }

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS || info.joints.empty()) {
      return CallbackReturn::ERROR;
    }
    try {
      const auto number = [&](const std::string & key, double fallback) {
          const auto item = info_.hardware_parameters.find(key);
          const auto value = item == info_.hardware_parameters.end() ? fallback : std::stod(
            item->second);
          if (!std::isfinite(value) || value <= 0) {throw std::invalid_argument(key);}
          return value;
        };
      feedback_timeout_ = number("feedback_timeout", 1.0);
      startup_timeout_ = number("startup_timeout", 40.0);
      for (const auto & joint : info.joints) {
        if (joint.command_interfaces.size() != 1 ||
          joint.command_interfaces.front().name != hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces.size() != 2 ||
          joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
          throw std::invalid_argument(
                  "Expected position command and position/velocity state interfaces");
        }
      }
      const auto missing = std::numeric_limits<double>::quiet_NaN();
      positions_.assign(info.joints.size(), missing);
      velocities_.assign(info.joints.size(), missing);
      commands_.assign(info.joints.size(), missing);
      cached_positions_ = positions_;
      cached_velocities_ = velocities_;
      rclcpp::NodeOptions options;
      options.arguments({"--ros-args", "-r", "__ns:=" + info_.hardware_parameters.at("namespace")});
      options.use_global_arguments(false);
      node_ = std::make_shared<rclcpp::Node>("agx_topic_hardware", options);
      executor_.add_node(node_);
      subscription_ = node_->create_subscription<JointState>(
        "feedback/joint_states", rclcpp::SensorDataQoS(),
        [this](JointState::ConstSharedPtr msg) {receive(*msg);});
      publisher_ = node_->create_publisher<JointState>("control/joint_states", 1);
      list_client_ = node_->create_client<List>("controller_manager/list_controllers");
      switch_client_ = node_->create_client<Switch>("controller_manager/switch_controller");
      stop_client_ = node_->create_client<Stop>("control_stop");
      spin_thread_ = std::thread(
        [this]() {
          auto retry_at = Clock::now();
          while (running_ && rclcpp::ok()) {
            executor_.spin_once(std::chrono::milliseconds(50));
            if (state_ == State::Faulted && Clock::now() >= retry_at) {
              stopControllers();
              retry_at = Clock::now() + std::chrono::seconds(1);
            }
          }
        });
    } catch (const std::exception & error) {
      RCLCPP_ERROR(rclcpp::get_logger("agx_topic_hardware"), "%s", error.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> result;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      result.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &positions_[i]);
      result.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &velocities_[i]);
    }
    return result;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> result;
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      result.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &commands_[i]);
    }
    return result;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (state_ == State::Faulted) {return CallbackReturn::ERROR;}
    const auto deadline = Clock::now() + std::chrono::duration<double>(startup_timeout_);
    while (rclcpp::ok() && Clock::now() < deadline) {
      if (fresh()) {
        copyState();
        // Never initialize a real arm from mock zero values. Keep exported pointers stable.
        std::copy(positions_.begin(), positions_.end(), commands_.begin());
        state_ = State::Active;
        stop_sent_ = false;
        RCLCPP_INFO(node_->get_logger(), "Commands synchronized to actual joint feedback");
        return CallbackReturn::SUCCESS;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_ERROR(node_->get_logger(), "No complete, fresh AGX feedback; activation refused");
    return CallbackReturn::ERROR;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (state_ == State::Active) {state_ = State::Inactive;}
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State &) override
  {
    if (state_ != State::Stopped) {state_ = State::Faulted;}
    return CallbackReturn::SUCCESS;
  }

  Return read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    copyState();
    if (state_ == State::Stopped) {return Return::ERROR;}
    if (state_ == State::Active && !fresh()) {
      // Retain interface ownership only while stopping its controllers. Humble
      // rejects stop requests after read(ERROR) has made those interfaces unavailable.
      state_ = State::Faulted;
      reportStale();
    }
    return Return::OK;
  }

  Return write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (state_ != State::Active) {return Return::OK;}
    if (!fresh()) {
      state_ = State::Faulted;
      reportStale();
      return Return::OK;
    }
    if (!std::all_of(commands_.begin(), commands_.end(), [](double v) {return std::isfinite(v);})) {
      state_ = State::Faulted;
      RCLCPP_ERROR(node_->get_logger(), "Non-finite command; stopping command publication");
      return Return::OK;
    }
    JointState msg;
    msg.header.stamp = node_->now();
    for (const auto & joint : info_.joints) {msg.name.push_back(joint.name);}
    msg.position = commands_;
    publisher_->publish(msg);
    return Return::OK;
  }

private:
  void reportStale() const
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    const double message_age = (node_->now().nanoseconds() - last_stamp_) / 1e9;
    const double receipt_age = std::chrono::duration<double>(Clock::now() - received_at_).count();
    RCLCPP_ERROR(node_->get_logger(),
      "AGX feedback stale: message age %.3f s, receipt age %.3f s, limit %.3f s; last rejected: %s",
      message_age, receipt_age, feedback_timeout_, rejected_.load());
  }

  void copyState()
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    std::copy(cached_positions_.begin(), cached_positions_.end(), positions_.begin());
    std::copy(cached_velocities_.begin(), cached_velocities_.end(), velocities_.begin());
  }

  bool fresh() const
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (last_stamp_ == 0) {return false;}
    const double age = (node_->now().nanoseconds() - last_stamp_) / 1e9;
    return age >= -0.1 && age <= feedback_timeout_ &&
           std::chrono::duration<double>(Clock::now() - received_at_).count() <= feedback_timeout_;
  }

  void receive(const JointState & msg)
  {
    const auto stamp = rclcpp::Time(msg.header.stamp).nanoseconds();
    const double age = (node_->now().nanoseconds() - stamp) / 1e9;
    if (stamp <= last_stamp_) {rejected_ = "timestamp not increasing"; return;}
    if (age < -0.1 || age > feedback_timeout_) {rejected_ = "timestamp outside allowed age"; return;}
    if (msg.position.size() != msg.name.size() || msg.velocity.size() != msg.name.size() ||
      std::set<std::string>(msg.name.begin(), msg.name.end()).size() != msg.name.size())
    {rejected_ = "invalid array sizes or duplicate joints"; return;}
    std::vector<double> positions, velocities;
    for (const auto & joint : info_.joints) {
      const auto found = std::find(msg.name.begin(), msg.name.end(), joint.name);
      if (found == msg.name.end()) {rejected_ = "required joint missing"; return;}
      const auto index = static_cast<std::size_t>(found - msg.name.begin());
      if (!std::isfinite(msg.position[index]) || !std::isfinite(msg.velocity[index])) {
        rejected_ = "non-finite joint feedback"; return;
      }
      positions.push_back(msg.position[index]);
      velocities.push_back(msg.velocity[index]);
    }
    // Keep exported vector storage stable: ros2_control holds pointers to it.
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    cached_positions_ = std::move(positions);
    cached_velocities_ = std::move(velocities);
    last_stamp_ = stamp;
    received_at_ = Clock::now();
    rejected_ = "none since last valid feedback";
  }

  template<typename Service>
  typename Service::Response::SharedPtr call(
    const typename rclcpp::Client<Service>::SharedPtr & client,
    const std::shared_ptr<typename Service::Request> & request)
  {
    if (!client->service_is_ready()) {return nullptr;}
    auto pending = client->async_send_request(request);
    if (executor_.spin_until_future_complete(pending, std::chrono::seconds(3)) ==
      rclcpp::FutureReturnCode::SUCCESS) {return pending.get();}
    client->remove_pending_request(pending);
    RCLCPP_ERROR(node_->get_logger(), "Controller recovery service timed out; retrying");
    return nullptr;
  }

  void stopControllers()
  {
    // Runs on the private I/O thread, never on the controller update thread.
    // Keep interface ownership until affected controllers stop (Humble requirement).
    if (!stop_sent_) {
      const auto stopped = call<Stop>(stop_client_, std::make_shared<Stop::Request>());
      stop_sent_ = stopped && stopped->success;
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
        "Hardware stop packet %s; physical standstill unconfirmed", stop_sent_ ? "sent" : "unconfirmed");
    }
    const auto listed = call<List>(list_client_, std::make_shared<List::Request>());
    if (!listed) {return;}
    auto request = std::make_shared<Switch::Request>();
    request->strictness = Switch::Request::STRICT;
    request->timeout.sec = 2;
    for (const auto & controller : listed->controller) {
      if (controller.state != "active") {continue;}
      const bool owns = std::any_of(
        info_.joints.begin(), info_.joints.end(), [&](const auto & joint) {
          return std::find(
            controller.claimed_interfaces.begin(), controller.claimed_interfaces.end(),
            joint.name + "/position") != controller.claimed_interfaces.end();
        });
      if (owns) {request->deactivate_controllers.push_back(controller.name);}
    }
    if (request->deactivate_controllers.empty()) {
      if (stop_sent_) {state_ = State::Stopped;}
      return;
    }
    const auto switched = call<Switch>(switch_client_, request);
    if (switched && switched->ok) {
      RCLCPP_WARN(
        node_->get_logger(),
        "Affected controllers deactivated; manual recovery required");
    } else if (switched) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Controller deactivation not confirmed; retrying");
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<JointState>::SharedPtr subscription_;
  rclcpp::Publisher<JointState>::SharedPtr publisher_;
  rclcpp::Client<List>::SharedPtr list_client_;
  rclcpp::Client<Switch>::SharedPtr switch_client_;
  rclcpp::Client<Stop>::SharedPtr stop_client_;
  std::atomic<bool> stop_sent_{false};
  std::thread spin_thread_;
  mutable std::mutex feedback_mutex_;
  std::vector<double> cached_positions_, cached_velocities_;
  std::vector<double> positions_, velocities_, commands_;
  double feedback_timeout_ = 1.0, startup_timeout_ = 40.0;
  std::atomic<const char *> rejected_{"none since last valid feedback"};
  int64_t last_stamp_ = 0;
  Clock::time_point received_at_;
  std::atomic<State> state_{State::Inactive};
  std::atomic<bool> running_{true};
};
}  // namespace my_robot_hardware

PLUGINLIB_EXPORT_CLASS(my_robot_hardware::AgxTopicSystem, hardware_interface::SystemInterface)
