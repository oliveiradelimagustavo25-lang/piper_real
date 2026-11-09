// Copyright 2026 piper_real contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Adapted from Point Spread's Apache-2.0 ROS wrapper; notice covers local changes.

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vidu_ros2_interfaces/msg/extrinsics.hpp>
#include "vidu_ros2_wrapper/conversions.hpp"


namespace
{
using Clock = std::chrono::steady_clock;
using Health = vidu_ros2::CameraHealth;

void require(bool condition, const std::string & message)
{
  if (!condition) {throw std::runtime_error(message);}
}

// A failed write is tolerable if a successful read confirms the required value.
bool verify_mode(PDstream & stream, const char * name, bool value, std::string & warning)
{
  const bool written = stream.set(name, value);
  bool actual = !value;
  const bool verified = stream.get(name, actual) && actual == value;
  if (!written || !verified) {
    warning += std::string(name) + (verified ? " write failed, readback correct; " :
      " is unconfirmed; ");
  }
  return verified;
}

void run(const rclcpp::Node::SharedPtr & node)
{
  rcl_interfaces::msg::ParameterDescriptor startup_only;
  startup_only.read_only = true;
  startup_only.description = "Restart to change the camera grid/configuration";
  const auto color_frame = node->declare_parameter<std::string>(
    "color_frame_id", "camera_color_optical_frame", startup_only);
  const auto depth_frame = node->declare_parameter<std::string>(
    "depth_frame_id", "camera_depth_optical_frame", startup_only);
  const auto publish_tf = node->declare_parameter<bool>("publish_tf", true, startup_only);
  tf2_ros::StaticTransformBroadcaster optical_tf(node);
  const auto color_url = node->declare_parameter<std::string>(
    "color_camera_info_url", "",
    startup_only);
  const auto depth_url = node->declare_parameter<std::string>(
    "depth_camera_info_url", "",
    startup_only);
  const auto color_order = node->declare_parameter<std::string>(
    "sdk_color_order", "auto",
    startup_only);
  auto positive_seconds = [&](const char * name, double fallback) {
      const auto value = node->declare_parameter<double>(name, fallback, startup_only);
      if (!std::isfinite(value) || value <= 0) {
        RCLCPP_WARN(node->get_logger(), "Invalid %s; using %.1f seconds", name, fallback);
        return fallback;
      }
      return value;
    };
  const auto retry_interval =
    std::chrono::duration<double>(positive_seconds("retry_interval", 2.0));
  vidu_ros2::FrameWatchdog watchdog{
    positive_seconds("startup_timeout", 10.0), positive_seconds("frame_timeout", 5.0)};
  const bool frames_valid = !color_frame.empty() && !depth_frame.empty() &&
    color_frame != depth_frame;
  const bool config_valid = frames_valid &&
    (color_order == "auto" || color_order == "rgb" || color_order == "bgr");

  auto raw_pub = node->create_publisher<sensor_msgs::msg::Image>("camera/color/origin", 2);
  auto aligned_pub = node->create_publisher<sensor_msgs::msg::Image>("camera/color/aligned", 2);
  auto depth_pub = node->create_publisher<sensor_msgs::msg::Image>("camera/depth", 2);
  auto ir_pub = node->create_publisher<sensor_msgs::msg::Image>("camera/infrared", 2);
  auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("camera/pointcloud", 2);
  auto color_info_pub =
    node->create_publisher<sensor_msgs::msg::CameraInfo>("camera/info/color", 2);
  auto depth_info_pub =
    node->create_publisher<sensor_msgs::msg::CameraInfo>("camera/info/depth", 2);
  auto extrinsics_pub = node->create_publisher<vidu_ros2_interfaces::msg::Extrinsics>(
    "camera/extrinsics/color_to_depth", 2);

  // Declaration order ensures streams are released before their device.
  std::unique_ptr<PDdevice> device;
  std::unique_ptr<PDstream> stream;
  vidu_ros2::CalibrationCache color, depth;
  std::string serial;
  Health health;
  bool grid_verified = false, retry_calibration = true, retry_settings = true;
  auto next_retry = Clock::now();
  auto last_log = Clock::now() - std::chrono::seconds(5);
  std::string last_report;
  rclcpp::WallRate poll(100);
  // Construct the executor once while the context is valid. The free
  // rclcpp::spin_some(node) constructs a new executor and can race SIGINT.
  const auto context = node->get_node_base_interface()->get_context();
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(node);

  auto read_calibration = [&](const char * id, const std::string & name,
      const std::string & url, vidu_ros2::CalibrationCache & cache) {
      std::string error;
      try {
        PDstream calibration_stream(*device, id);
        intrinsics intrinsic;
        extrinsics transform;
        require(calibration_stream.init(), "Cannot initialize " + name + " calibration stream");
        if (name == "depth") {
          // SDK getCamPara example shows that DistortRemove changes returned K/D.
          // Establish the raw ToF mode BEFORE reading its corresponding intrinsics.
          std::string warning;
          require(
            verify_mode(calibration_stream, "DistortRemove", false, warning),
            "Cannot establish raw ToF mode before calibration: " + warning);
        }
        require(
          calibration_stream.getCamPara(intrinsic, transform),
          "Cannot read " + name + " calibration");
        if (cache.refresh(*node, name, url, intrinsic, transform, error)) {
          if (name == "color" && publish_tf) {
            std_msgs::msg::Header header;
            header.stamp = node->now(); header.frame_id = color_frame;
            optical_tf.sendTransform(vidu_ros2::depth_to_color_tf(transform, header, depth_frame));
          }
          RCLCPP_INFO(
            node->get_logger(), "%s: SDK model=%d -> %s, %ux%u",
            name.c_str(), static_cast<int>(intrinsic.model), cache.info.distortion_model.c_str(),
            cache.info.width, cache.info.height);
        } else {
          error = name + ": " + error;
        }
      } catch (const std::exception & failure) {
        error = failure.what();
      }
      if (!error.empty()) {
        error +=
          cache.valid ? " (retaining validated calibration); " : " (waiting for calibration); ";
      }
      return error;
    };

  while (rclcpp::ok(context)) {
    const auto now = Clock::now();
    if (!config_valid) {
      // Invalid configuration must not repeatedly open a physical device.
      health.faults[Health::Mode] =
        "Invalid optical frame parameters or sdk_color_order; waiting for restart";
    } else if (now >= next_retry) {
      next_retry = now + std::chrono::duration_cast<Clock::duration>(retry_interval);
      try {
        bool opened_stream = false;
        if (!device) {
          auto candidate = std::make_unique<PDdevice>();
          require(candidate->init(), "No valid camera device; retrying");
          const auto connected_serial = candidate->getDeviceSN();
          // Never reuse another camera's calibration, even at the same resolution.
          if (connected_serial.empty() || connected_serial != serial) {
            color = vidu_ros2::CalibrationCache();
            depth = vidu_ros2::CalibrationCache();
          }
          if (!rclcpp::ok(context)) {break;}
          serial = connected_serial;
          device = std::move(candidate);
          retry_calibration = true;
          RCLCPP_INFO(node->get_logger(), "Camera connected: %s", serial.c_str());
        }
        if (retry_calibration) {
          // SDK inhibits individual RGB/ToF acquisition while PCL runs. Release
          // PCL before reopening calibration streams; do not run competing streams.
          stream.reset();
          grid_verified = false;
          const auto color_error = read_calibration("RGB", "color", color_url, color);
          if (!rclcpp::ok(context)) {break;}
          const auto depth_error = read_calibration("ToF", "depth", depth_url, depth);
          if (!rclcpp::ok(context)) {break;}
          health.faults[Health::Calibration] = color_error + depth_error;
          retry_calibration = !health.faults[Health::Calibration].empty();
        }
        if (!stream) {
          auto candidate = std::make_unique<PDstream>(*device, "PCL");
          require(candidate->init(), "PCL initialization failed; retrying");
          stream = std::move(candidate);
          opened_stream = true;
          retry_settings = true;
        }
        if (retry_settings) {
          std::string warning;
          if (!stream->set("RGB::StreamFps", 30.0f)) {warning += "RGB FPS; ";}
          if (!stream->set("ToF::Distance", 2.5f)) {warning += "ToF distance; ";}
          if (!stream->set("ToF::StreamFps", 30.0f)) {warning += "ToF FPS; ";}
          if (!stream->set("ToF::Threshold", 100)) {warning += "ToF threshold; ";}
          verify_mode(*stream, "PCL::EnableRgbAttach", true, warning);
          const bool depth_grid = verify_mode(*stream, "PCL::DepthAlign2Color", false, warning);
          const bool raw_tof = verify_mode(*stream, "ToF::DistortRemove", false, warning);
          grid_verified = depth_grid && raw_tof;
          if (!config_valid) {warning += "Invalid optical frame parameters; ";}
          health.faults[Health::Mode] = warning;
          retry_settings = !warning.empty();
        }
        if (opened_stream) {
          watchdog.started(Clock::now());
          RCLCPP_INFO(
            node->get_logger(), "PCL configured; waiting up to %.1fs for first frame",
            watchdog.startup_timeout);
        }
      } catch (const std::exception & failure) {
        health.faults[Health::Mode] = failure.what();
        stream.reset(); device.reset(); grid_verified = false;
      }
    }

    if (stream && config_valid) {
      try {
        auto frame = stream->waitFrames();
        const auto received = node->now();  // Frame reception estimate, not exposure time.
        if (frame) {
          if (!watchdog.has_frame) {
            RCLCPP_INFO(
              node->get_logger(), "First SDK frame received after %.3fs, images=%u",
              watchdog.idle_seconds(Clock::now()), frame->getImageNum());
          }
          watchdog.received(Clock::now());
          std::string data_error;
          const auto count = frame->getImageNum();
          std_msgs::msg::Header color_header, depth_header;
          color_header.stamp = received; color_header.frame_id = color_frame;
          depth_header.stamp = received; depth_header.frame_id = depth_frame;
          // Keep independent outputs alive when a different channel is invalid.
          auto channel = [&](const char * name, const std::function<void()> & publish) {
              try {
                publish();
              } catch (const std::exception & failure) {
                data_error += std::string(name) + ": " + failure.what() + "; ";
              }
            };
          channel(
            "color", [&] {
              require(count >= 4, "Missing original color image");
              const auto raw = vidu_ros2::to_bgr(frame->getImage(3), color_order);
              require(
                color.valid && raw.cols == static_cast<int>(color.info.width) &&
                raw.rows == static_cast<int>(color.info.height), "No matching color calibration");
              auto info = color.info; info.header = color_header;
              color_info_pub->publish(info);
              raw_pub->publish(*cv_bridge::CvImage(color_header, "bgr8", raw).toImageMsg());
              vidu_ros2_interfaces::msg::Extrinsics extrinsic_info;
              extrinsic_info.header = color_header;
              std::copy(
                color.extrinsic.rotation, color.extrinsic.rotation + 9,
                extrinsic_info.rotation.begin());
              std::copy(
                color.extrinsic.translation, color.extrinsic.translation + 3,
                extrinsic_info.translation.begin());
              extrinsics_pub->publish(extrinsic_info);
            });
          cv::Mat xyz;
          channel(
            "depth", [&] {
              require(grid_verified, "Depth grid unconfirmed; waiting for mode verification");
              require(count >= 1, "Missing XYZ image");
              const auto candidate = vidu_ros2::image_view(frame->getImage(0), CV_32FC3);
              require(
                depth.valid && candidate.cols == static_cast<int>(depth.info.width) &&
                candidate.rows == static_cast<int>(depth.info.height),
                "No matching depth calibration");
              const auto z = vidu_ros2::depth_metres(candidate);
              auto info = depth.info; info.header = depth_header;
              depth_info_pub->publish(info);
              depth_pub->publish(*cv_bridge::CvImage(depth_header, "32FC1", z).toImageMsg());
              xyz = candidate;
            });
          if (!xyz.empty()) {
            channel(
              "aligned color/cloud", [&] {
                require(count >= 3, "Missing aligned color image");
                const auto aligned = vidu_ros2::to_bgr(frame->getImage(2), color_order);
                require(aligned.size() == xyz.size(), "Aligned color/XYZ dimensions differ");
                const auto cloud = vidu_ros2::to_cloud(aligned, xyz, depth_header);
                aligned_pub->publish(
                  *cv_bridge::CvImage(
                    depth_header, "bgr8",
                    aligned).toImageMsg());
                cloud_pub->publish(cloud);
              });
            channel(
              "infrared", [&] {
                require(count >= 2, "Missing ToF intensity image");
                const auto ir = vidu_ros2::image_view(frame->getImage(1), CV_16UC1);
                require(ir.size() == xyz.size(), "ToF intensity/XYZ dimensions differ");
                ir_pub->publish(*cv_bridge::CvImage(depth_header, "16UC1", ir).toImageMsg());
              });
          }
          health.faults[Health::Data] = data_error;
        } else if (watchdog.expired(Clock::now())) {
          health.faults[Health::Data] = std::string(
            watchdog.has_frame ? "Camera stream stalled for " :
            "No first SDK frame after ") + std::to_string(watchdog.idle_seconds(Clock::now())) +
            "s; reconnecting after retry interval";
          RCLCPP_WARN(node->get_logger(), "%s", health.faults[Health::Data].c_str());
          stream.reset(); device.reset(); grid_verified = false;
          next_retry = Clock::now() + std::chrono::duration_cast<Clock::duration>(retry_interval);
        } else if (watchdog.idle_seconds(Clock::now()) >= 2.0) {
          health.faults[Health::Data] = watchdog.has_frame ?
            "Waiting for next SDK frame; keeping current stream until frame_timeout" :
            "Waiting for first SDK frame; keeping current stream until startup_timeout";
        }
      } catch (const std::exception & failure) {
        health.faults[Health::Data] = failure.what();
        stream.reset(); device.reset(); grid_verified = false;
      }
    } else {
      health.faults[Health::Data] = "Waiting for camera stream/valid frame configuration";
      if (!color.valid || !depth.valid) {
        health.faults[Health::Calibration] = "Waiting for validated camera calibration";
      }
    }

    const auto report = "Calibration: " + health.faults[Health::Calibration] +
      " | Mode: " + health.faults[Health::Mode] + " | Data: " + health.faults[Health::Data];
    const bool faulty = std::any_of(
      health.faults.begin(), health.faults.end(),
      [](const std::string & fault) {return !fault.empty();});
    if (Clock::now() - last_log >= std::chrono::seconds(5)) {
      if (faulty) {
        if (health.error()) {RCLCPP_ERROR(node->get_logger(), "%s", report.c_str());} else {
          RCLCPP_WARN(node->get_logger(), "%s", report.c_str());
        }
      } else if (!last_report.empty()) {
        RCLCPP_INFO(node->get_logger(), "Camera recovered; all channels ready");
      }
      last_report = faulty ? report : "";
      last_log = Clock::now();
    }
    if (!rclcpp::ok(context)) {break;}
    try {
      executor.spin_some();
    } catch (const rclcpp::exceptions::RCLError & error) {
      if (!rclcpp::ok(context)) {break;}
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 5000,
        "ROS callback processing: %s", error.what());
    }
    if (rclcpp::ok(context)) {poll.sleep();}
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("vidu_ros2_wrapper_node");
  run(node);
  rclcpp::shutdown();
  return 0;
}
