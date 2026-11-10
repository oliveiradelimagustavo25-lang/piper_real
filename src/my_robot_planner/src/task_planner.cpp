#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <my_robot_interface/action/pick_place.hpp>
#include <my_robot_interface/action/move_to_named.hpp>
#include <my_robot_interface/action/sort_objects.hpp>
#include <my_robot_interface/msg/detected_object_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class TaskPlanner : public rclcpp::Node
{
  using Sort = my_robot_interface::action::SortObjects;
  using Pick = my_robot_interface::action::PickPlace;
  using Named = my_robot_interface::action::MoveToNamed;
  using Object = my_robot_interface::msg::DetectedObject;
  using Objects = my_robot_interface::msg::DetectedObjectArray;
  using SortHandle = rclcpp_action::ServerGoalHandle<Sort>;
  using PickHandle = rclcpp_action::ClientGoalHandle<Pick>;
  using NamedHandle = rclcpp_action::ClientGoalHandle<Named>;
  enum class Phase {WAITING, SENDING, EXECUTING, OBSERVING};

public:
  TaskPlanner() : Node("task_planner"), tf_buffer_(get_clock())
  {
    frame_ = declare_parameter<std::string>("reference_frame", "base_link");
    tool_frame_ = declare_parameter<std::string>("end_effector_frame", "link6");
    camera_frame_ = declare_parameter<std::string>("required_camera_frame", "");
    camera_tf_max_age_ = declare_parameter<double>("camera_tf_max_age", 1.0);
    one_per_color_ = declare_parameter<bool>("one_object_per_color", false);
    tool_to_tcp_ = xyz(declare_parameter<std::vector<double>>("tool_to_tcp", {0, 0, 0.132}));
    default_strategy_ = declare_parameter<std::string>("default_strategy", "color_order");
    const auto order = declare_parameter<std::vector<int64_t>>("default_color_order", {1, 2, 3});
    for (const auto color : order) {
      if (color < 1 || color > 3) {
        throw std::invalid_argument("default_color_order must contain colors 1/2/3");
      }
      default_order_.push_back(static_cast<uint8_t>(color));
    }
    max_age_ = declare_parameter<double>("max_observation_age", 2.0);
    timeout_ = declare_parameter<double>("observation_timeout", 5.0);
    observation_target_ = declare_parameter<std::string>("observation_target", "");
    settle_time_ = declare_parameter<double>("observation_settle_time", 0.3);
    const auto place_modes = declare_parameter<std::vector<std::string>>(
      "place_position_modes", {"absolute", "absolute", "absolute"});
    if (place_modes.size() != 3 || !std::all_of(place_modes.begin(), place_modes.end(),
      [](const auto & mode) {return mode == "absolute" || mode == "relative";}))
    {
      throw std::invalid_argument("place_position_modes needs absolute/relative for each of x/y/z");
    }
    std::transform(place_modes.begin(), place_modes.end(), place_relative_axes_.begin(),
      [](const auto & mode) {return mode == "relative";});
    places_[1] = xyz(declare_parameter<std::vector<double>>(
        "red.place_position", {0.26, -0.04, 0.005}));
    places_[2] = xyz(declare_parameter<std::vector<double>>(
        "green.place_position", {0.26, 0.0, 0.005}));
    places_[3] = xyz(declare_parameter<std::vector<double>>(
        "blue.place_position", {0.26, 0.04, 0.005}));
    if (!validPolicy(default_strategy_, default_order_) || frame_.empty() || tool_frame_.empty() ||
      !std::isfinite(max_age_) || max_age_ <= 0 || !std::isfinite(timeout_) || timeout_ <= 0 ||
      !std::isfinite(settle_time_) || settle_time_ < 0 ||
      !std::isfinite(camera_tf_max_age_) || camera_tf_max_age_ <= 0)
    {
      throw std::invalid_argument("Invalid task policy, frames or observation timeouts");
    }

    subscription_ = create_subscription<Objects>(
      declare_parameter<std::string>("objects_topic", "/perception/objects"),
      rclcpp::SensorDataQoS(), [this](Objects::ConstSharedPtr objects) {
        observations_ = objects;
        last_received_ = now();
      });
    pick_client_ = rclcpp_action::create_client<Pick>(this,
      declare_parameter<std::string>("pick_place_action", "/manipulation/pick_place"));
    named_client_ = rclcpp_action::create_client<Named>(this,
      declare_parameter<std::string>("move_to_named_action", "/manipulation/move_to_named"));
    server_ = rclcpp_action::create_server<Sort>(this,
      declare_parameter<std::string>("action_name", "/sorting/sort_objects"),
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Sort::Goal> goal) {
        const auto strategy = goal->strategy.empty() ? default_strategy_ : goal->strategy;
        const auto & order = goal->color_order.empty() ? default_order_ : goal->color_order;
        if (busy_ || !validPolicy(strategy, order)) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        busy_ = true;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<SortHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](std::shared_ptr<SortHandle> handle) {
        task_ = handle;
        const auto goal = handle->get_goal();
        strategy_ = goal->strategy.empty() ? default_strategy_ : goal->strategy;
        order_ = goal->color_order.empty() ? default_order_ : goal->color_order;
        pending_.clear();
        batch_ready_ = false;
        completed_ = 0;
        phase_ = Phase::WAITING;
        // A new batch starts at the current pose. A named observation pose is
        // an optional between-object policy, never an Action precondition.
        needs_observation_ = false;
        observations_after_ = now() + rclcpp::Duration::from_seconds(settle_time_);
        observations_.reset();
        waiting_since_ = std::chrono::steady_clock::now();
        waiting_reason_.clear();
        feedback("");
      });
    // 所有任务状态都在同一个 executor 中更新；Action 等待使用异步回调。
    timer_ = create_wall_timer(100ms, [this]() { tick(); });
    // Start the TF thread only after configuration/setup succeeds; constructor errors
    // must not wait for a listener thread while the ROS context is still active.
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
    RCLCPP_INFO(get_logger(), "Task planner ready; waiting for SortObjects goals");
  }

private:
  static tf2::Vector3 xyz(const std::vector<double> & values)
  {
    if (values.size() != 3 || !std::all_of(values.begin(), values.end(),
      [](double v) { return std::isfinite(v); }))
    {
      throw std::invalid_argument("Expected three finite xyz parameter values");
    }
    return {values[0], values[1], values[2]};
  }

  static bool validPolicy(const std::string & strategy, const std::vector<uint8_t> & order)
  {
    const std::set<uint8_t> colors(order.begin(), order.end());
    return (strategy == "color_order" || strategy == "nearest") && !order.empty() &&
           colors.size() == order.size() && *colors.begin() >= 1 && *colors.rbegin() <= 3;
  }

  std::vector<Object> objectsInBase()
  {
    const auto & header = observations_->header;
    if (header.frame_id.empty()) {
      throw std::invalid_argument("Object list has no frame_id");
    }
    geometry_msgs::msg::TransformStamped transform;
    if (header.frame_id != frame_) {
      transform = tf_buffer_.lookupTransform(frame_, header.frame_id, rclcpp::Time(header.stamp));
    }
    std::set<std::string> ids;
    auto objects = observations_->objects;
    for (auto & object : objects) {
      const auto & p = object.pose.position;
      const auto & d = object.dimensions;
      tf2::Quaternion q;
      tf2::fromMsg(object.pose.orientation, q);
      if (object.object_id.empty() || !ids.insert(object.object_id).second ||
        object.color < 1 || object.color > 3 || !std::isfinite(p.x) ||
        !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(d.x) || !std::isfinite(d.y) || !std::isfinite(d.z) ||
        d.x <= 0 || d.y <= 0 || d.z <= 0 || !std::isfinite(q.length2()) || q.length2() < 1e-12)
      {
        throw std::invalid_argument("Invalid object ID, color, pose or dimensions");
      }
      q.normalize();
      object.pose.orientation = tf2::toMsg(q);
      if (header.frame_id != frame_) {
        geometry_msgs::msg::PoseStamped source, result;
        source.header = header;
        source.pose = object.pose;
        tf2::doTransform(source, result, transform);
        object.pose = result.pose;
      }
    }
    return objects;
  }

  Object selectTarget(const std::vector<Object> & candidates)
  {
    if (strategy_ == "color_order") {
      const auto rank = [this](uint8_t color) {
        return std::find(order_.begin(), order_.end(), color) - order_.begin();
      };
      return *std::min_element(candidates.begin(), candidates.end(),
        [&](const auto & a, const auto & b) {
          return rank(a.color) != rank(b.color) ? rank(a.color) < rank(b.color) :
                 a.object_id < b.object_id;
        });
    }
    // 每次派发前重新查询当前 TCP，距离策略不使用上一轮缓存的位置。
    const auto transform = tf_buffer_.lookupTransform(frame_, tool_frame_, tf2::TimePointZero);
    tf2::Transform tool;
    tf2::fromMsg(transform.transform, tool);
    const auto tcp = tool * tool_to_tcp_;
    const auto distance = [&](const Object & object) {
      const auto & p = object.pose.position;
      return (tf2::Vector3(p.x, p.y, p.z) - tcp).length2();
    };
    return *std::min_element(candidates.begin(), candidates.end(),
      [&](const auto & a, const auto & b) {
        return distance(a) != distance(b) ? distance(a) < distance(b) :
               a.object_id < b.object_id;
      });
  }

  void feedback(const std::string & id)
  {
    auto update = std::make_shared<Sort::Feedback>();
    update->current_object_id = id;
    update->completed_count = completed_;
    task_->publish_feedback(update);
  }

  void finish(bool success, const std::string & message, bool canceled = false)
  {
    auto result = std::make_shared<Sort::Result>();
    result->success = success;
    result->message = message;
    if (canceled) {
      task_->canceled(result);
    } else if (success) {
      task_->succeed(result);
    } else {
      task_->abort(result);
    }
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    task_.reset();
    active_pick_.reset();
    active_named_.reset();
    busy_ = false;
  }

  void waiting(const std::string & reason)
  {
    if (reason != waiting_reason_) {
      waiting_reason_ = reason;
      RCLCPP_INFO(get_logger(), "%s", reason.c_str());
    }
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - waiting_since_).count()
      >= timeout_)
    {
      finish(false, "Timed out: " + reason);
    }
  }

  void observe()
  {
    Named::Goal goal;
    goal.target = observation_target_;
    phase_ = Phase::OBSERVING;
    cancel_sent_ = false;
    const auto task = task_;
    rclcpp_action::Client<Named>::SendGoalOptions options;
    options.goal_response_callback = [this, task](NamedHandle::SharedPtr handle) {
        if (task_ != task) { return; }
        if (!handle) {
          finish(false, "Observation motion rejected", task->is_canceling());
        } else {
          active_named_ = handle;
        }
      };
    options.result_callback = [this, task](const NamedHandle::WrappedResult & result) {
        if (task_ != task) { return; }
        active_named_.reset();
        if (task->is_canceling()) {
          finish(false, "Sorting canceled after observation motion", true);
        } else if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result->success) {
          finish(false, "Observation motion failed: " + result.result->message);
        } else {
          needs_observation_ = false;
          observations_.reset();
          observations_after_ = now() + rclcpp::Duration::from_seconds(settle_time_);
          waiting_since_ = std::chrono::steady_clock::now();
          phase_ = Phase::WAITING;
          waiting_reason_.clear();
        }
      };
    RCLCPP_INFO(get_logger(), "Moving to observation target: %s", observation_target_.c_str());
    named_client_->async_send_goal(goal, options);
  }

  void dispatch(const Object & object)
  {
    Pick::Goal goal;
    goal.object_id = object.object_id;
    goal.object_pose.header = observations_->header;
    goal.object_pose.header.frame_id = frame_;
    goal.object_pose.pose = object.pose;
    goal.object_dimensions = object.dimensions;
    goal.place_pose.header.frame_id = frame_;
    // Resolve once from this object's observed centre, along reference_frame axes.
    // Copy the configured values so relative offsets never accumulate across tasks.
    auto place = places_[object.color];
    const tf2::Vector3 origin(object.pose.position.x, object.pose.position.y, object.pose.position.z);
    for (int axis = 0; axis < 3; ++axis) {
      if (place_relative_axes_[axis]) {place[axis] += origin[axis];}
    }
    if (!std::isfinite(place.x()) || !std::isfinite(place.y()) || !std::isfinite(place.z())) {
      throw std::invalid_argument("Resolved place position is not finite");
    }
    RCLCPP_INFO(
      get_logger(), "%s in %s: object [%.3f, %.3f, %.3f] -> place [%.3f, %.3f, %.3f] m",
      object.object_id.c_str(), frame_.c_str(), origin.x(), origin.y(), origin.z(),
      place.x(), place.y(), place.z());
    goal.place_pose.pose.position.x = place.x();
    goal.place_pose.pose.position.y = place.y();
    goal.place_pose.pose.position.z = place.z();
    goal.place_pose.pose.orientation = object.pose.orientation;
    feedback(object.object_id);
    phase_ = Phase::SENDING;
    cancel_sent_ = false;
    const auto task = task_;
    rclcpp_action::Client<Pick>::SendGoalOptions options;
    options.goal_response_callback = [this, task](PickHandle::SharedPtr handle) {
        if (task_ != task) { return; }
        if (!handle) {
          finish(false, "PickPlace rejected the target", task->is_canceling());
          return;
        }
        active_pick_ = handle;
        phase_ = Phase::EXECUTING;
      };
    options.result_callback = [this, task, id = object.object_id](const PickHandle::WrappedResult & r) {
        if (task_ != task) { return; }
        active_pick_.reset();
        const bool success = r.code == rclcpp_action::ResultCode::SUCCEEDED && r.result->success;
        if (success) {
          pending_.erase(id);
          ++completed_;
          feedback(id);
        }
        if (task->is_canceling()) {
          finish(false, "Sorting canceled after active PickPlace returned", true);
        } else if (!success) {
          finish(false, "PickPlace failed for " + id + ": " + r.result->message);
        } else if (pending_.empty()) {
          finish(true, "Batch motion sequences completed; placement needs visual verification");
        } else {
          phase_ = Phase::WAITING;
          needs_observation_ = !observation_target_.empty();
          waiting_since_ = std::chrono::steady_clock::now();
          waiting_reason_.clear();
        }
      };
    pick_client_->async_send_goal(goal, options);
  }

  void tick()
  {
    if (!task_) { return; }
    if (task_->is_canceling()) {
      if (phase_ == Phase::WAITING) {
        finish(false, "Sorting canceled before next pick", true);
      } else if (active_pick_ && !cancel_sent_) {
        cancel_sent_ = true;
        pick_client_->async_cancel_goal(active_pick_);
      } else if (active_named_ && !cancel_sent_) {
        cancel_sent_ = true;
        named_client_->async_cancel_goal(active_named_);
      }
      return;
    }
    if (phase_ != Phase::WAITING) { return; }
    try {
      if (!camera_frame_.empty()) {
        const auto transform = tf_buffer_.lookupTransform(frame_, camera_frame_, tf2::TimePointZero);
        const auto stamp = rclcpp::Time(transform.header.stamp, get_clock()->get_clock_type());
        const double age = (now() - stamp).seconds();
        if (stamp.nanoseconds() == 0 || age < -0.1 || age > camera_tf_max_age_) {
          throw std::runtime_error("Required camera TF is stale");
        }
      }
      if (needs_observation_) {
        if (!named_client_->action_server_is_ready()) {
          waiting("Waiting for observation motion server");
        } else {
          observe();
        }
        return;
      }
      if (!observations_ || (now() - last_received_).seconds() > max_age_) {
        waiting("Waiting for fresh object observations");
        return;
      }
      const auto stamp = rclcpp::Time(observations_->header.stamp, get_clock()->get_clock_type());
      if (last_received_ < observations_after_ ||
        (stamp.nanoseconds() != 0 && stamp < observations_after_))
      {
        waiting("Waiting for images acquired after observation motion settled");
        return;
      }
      if (stamp.nanoseconds() != 0) {
        const double age = (now() - stamp).seconds();
        if (age < -0.1 || age > max_age_) {
          waiting("Waiting for observations with a current timestamp");
          return;
        }
      }
      const auto objects = objectsInBase();
      if (!batch_ready_) {
        for (const auto color : order_) {
          if (std::none_of(objects.begin(), objects.end(),
            [color](const auto & object) { return object.color == color; }))
          {
            waiting("Waiting for requested color " + std::to_string(color));
            return;
          }
        }
        // 固定本批 ID，防止已放置物体被重新识别后再次进入任务。
        for (const auto color : order_) {
          for (const auto & object : objects) {
            if (object.color == color) {
              pending_[object.object_id] = color;
              if (one_per_color_) {break;}
            }
          }
        }
        batch_ready_ = true;
      }
      std::vector<Object> candidates;
      for (const auto & object : objects) {
        const auto pending = pending_.find(object.object_id);
        if (pending != pending_.end() && pending->second == object.color) {
          candidates.push_back(object);
        }
      }
      // 颜色顺序中，前一种颜色暂时丢失时等待它，不提前抓下一种。
      if (strategy_ == "color_order") {
        const auto first = std::find_if(order_.begin(), order_.end(), [this](uint8_t color) {
          return std::any_of(pending_.begin(), pending_.end(),
            [color](const auto & item) { return item.second == color; });
        });
        candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
          [first](const auto & object) { return object.color != *first; }), candidates.end());
      }
      if (candidates.empty()) {
        waiting("Waiting for a remaining object in this batch");
      } else if (!pick_client_->action_server_is_ready()) {
        waiting("Waiting for PickPlace action server");
      } else {
        dispatch(selectTarget(candidates));
      }
    } catch (const tf2::TransformException & error) {
      waiting("Waiting for TF: " + std::string(error.what()));
    } catch (const std::exception & error) {
      finish(false, error.what());
    }
  }

  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<Objects>::SharedPtr subscription_;
  rclcpp_action::Server<Sort>::SharedPtr server_;
  rclcpp_action::Client<Pick>::SharedPtr pick_client_;
  rclcpp_action::Client<Named>::SharedPtr named_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  Objects::ConstSharedPtr observations_;
  rclcpp::Time last_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time observations_after_{0, 0, RCL_ROS_TIME};
  std::shared_ptr<SortHandle> task_;
  PickHandle::SharedPtr active_pick_;
  NamedHandle::SharedPtr active_named_;
  std::string observation_target_;
  std::string frame_, tool_frame_, camera_frame_, default_strategy_, strategy_, waiting_reason_;
  tf2::Vector3 tool_to_tcp_;
  std::array<tf2::Vector3, 4> places_;
  std::array<bool, 3> place_relative_axes_{};
  std::vector<uint8_t> default_order_, order_;
  std::map<std::string, uint8_t> pending_;
  double max_age_, timeout_, settle_time_, camera_tf_max_age_;
  uint32_t completed_ = 0;
  bool busy_ = false, batch_ready_ = false, cancel_sent_ = false;
  bool needs_observation_ = false, one_per_color_ = false;
  Phase phase_ = Phase::WAITING;
  std::chrono::steady_clock::time_point waiting_since_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TaskPlanner>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("task_planner"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
