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

#pragma once

#include <array>
#include <chrono>
#include <string>
#include <rclcpp/node.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "Vidu_SDK_Api.h"

namespace vidu_ros2
{
cv::Mat image_view(const PDimage & image, int expected_type);
// auto follows PFNC; explicit rgb/bgr describes verified SDK buffer bytes.
cv::Mat to_bgr(const PDimage & image, const std::string & input_order = "auto");
cv::Mat depth_metres(const cv::Mat & xyz);
sensor_msgs::msg::PointCloud2 to_cloud(
  const cv::Mat & bgr, const cv::Mat & xyz, const std_msgs::msg::Header & header);
sensor_msgs::msg::CameraInfo camera_info(const intrinsics & intrinsic);
void validate_info(const sensor_msgs::msg::CameraInfo & info);
// File calibration must describe the same raw image size; depth K/D must also
// agree with the SDK, which already used its own calibration to generate XYZ.
void validate_override(
  const sensor_msgs::msg::CameraInfo & info,
  const sensor_msgs::msg::CameraInfo & sdk, bool depth);
// The loader reports invalid files to its caller; the node retains its cache and retries.
sensor_msgs::msg::CameraInfo load_info(
  rclcpp::Node & node, const std::string & name,
  const std::string & url, const sensor_msgs::msg::CameraInfo & sdk, bool depth);
// A failed refresh never overwrites the last validated calibration. Reset when
// the device serial changes; validate image dimensions before using this cache.
struct CalibrationCache
{
  sensor_msgs::msg::CameraInfo info;
  extrinsics extrinsic;
  bool valid = false;
  bool refresh(
    rclcpp::Node & node, const std::string & name, const std::string & url,
    const intrinsics & intrinsic, const extrinsics & transform, std::string & error);
};

// Stream lifecycle timing uses a monotonic clock, independent of ROS /clock.
// Initial acquisition gets its own budget, starting after stream configuration.
struct FrameWatchdog
{
  using Clock = std::chrono::steady_clock;
  double startup_timeout;
  double frame_timeout;
  Clock::time_point last_activity{};
  bool active = false;
  bool has_frame = false;
  void started(Clock::time_point now);
  void received(Clock::time_point now);
  double idle_seconds(Clock::time_point now) const;
  bool expired(Clock::time_point now) const;
};

// Active categories, not the number of repeated log messages.
struct CameraHealth
{
  enum Category {Calibration, Mode, Data};
  std::array<std::string, 3> faults;
  bool error() const;
};
}  // namespace vidu_ros2
