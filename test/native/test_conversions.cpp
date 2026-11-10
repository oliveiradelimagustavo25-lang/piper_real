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

#include <GenICam/PFNC.h>
#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "vidu_ros2_wrapper/conversions.hpp"


using vidu_ros2::CalibrationCache;
using vidu_ros2::CameraHealth;
using vidu_ros2::FrameWatchdog;
using vidu_ros2::camera_info;
using vidu_ros2::depth_metres;
using vidu_ros2::to_bgr;
using vidu_ros2::to_cloud;
using vidu_ros2::validate_info;
using vidu_ros2::validate_override;

TEST(Color, ChannelsAndCloud)
{
  cv::Mat rgb(1, 3, CV_8UC3);
  rgb.at<cv::Vec3b>(0, 0) = {255, 0, 0};
  rgb.at<cv::Vec3b>(0, 1) = {0, 255, 0};
  rgb.at<cv::Vec3b>(0, 2) = {0, 0, 255};
  PDimage input;
  input.FillImgae(rgb.data, 3, 1, PFNC_RGB8);
  const auto converted = to_bgr(input);
  EXPECT_EQ(converted.at<cv::Vec3b>(0, 0), cv::Vec3b(0, 0, 255));
  EXPECT_EQ(rgb.at<cv::Vec3b>(0, 0), cv::Vec3b(255, 0, 0));
  input.FillImgae(converted.data, 3, 1, PFNC_BGR8);
  EXPECT_EQ(cv::norm(to_bgr(input), converted), 0);
  cv::Mat xyz(1, 3, CV_32FC3, cv::Scalar(0.1, 0.2, 1.5));
  std_msgs::msg::Header header;
  header.frame_id = "depth_optical"; header.stamp.sec = 42;
  const auto cloud = to_cloud(converted, xyz, header);
  sensor_msgs::PointCloud2ConstIterator<uint8_t> r(cloud, "r"), g(cloud, "g"), b(cloud, "b");
  EXPECT_EQ(*r, 255); EXPECT_EQ(*g, 0); EXPECT_EQ(*b, 0);
  ++r; ++g; ++b;
  EXPECT_EQ(*r, 0); EXPECT_EQ(*g, 255); EXPECT_EQ(*b, 0);
  ++r; ++g; ++b;
  EXPECT_EQ(*r, 0); EXPECT_EQ(*g, 0); EXPECT_EQ(*b, 255);
  EXPECT_EQ(cloud.header, header);
  input.FillImgae(rgb.data, 3, 1, PFNC_Mono8);
  EXPECT_THROW(to_bgr(input), std::runtime_error);
}

TEST(Depth, MetresInvalidPointsAndGrid)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  cv::Mat xyz(1, 5, CV_32FC3);
  xyz.at<cv::Vec3f>(0, 0) = {0.1, -0.2, 1.25};
  xyz.at<cv::Vec3f>(0, 1) = {0, 0, 0};
  xyz.at<cv::Vec3f>(0, 2) = {0, 0, -1};
  xyz.at<cv::Vec3f>(0, 3) = {nan, 0, 1};
  xyz.at<cv::Vec3f>(0, 4) = {0, 0, std::numeric_limits<float>::infinity()};
  const auto depth = depth_metres(xyz);
  ASSERT_EQ(depth.type(), CV_32FC1);
  EXPECT_FLOAT_EQ(depth.at<float>(0, 0), 1.25);
  for (int x = 1; x < 5; ++x) {
    EXPECT_TRUE(std::isnan(depth.at<float>(0, x)));
  }
  cv::Mat bgr(1, 5, CV_8UC3, cv::Scalar(0, 0, 255));
  const auto cloud = to_cloud(bgr, xyz, std_msgs::msg::Header());
  ASSERT_EQ(cloud.width, 1u);
  sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
  EXPECT_FLOAT_EQ(*z, 1.25);
  EXPECT_THROW(to_cloud(bgr.col(0), xyz, std_msgs::msg::Header()), std::runtime_error);
  EXPECT_THROW(depth_metres(bgr), std::runtime_error);
}

intrinsics calibration()
{
  intrinsics i;
  i.width = 640; i.height = 480; i.fx = 400; i.fy = 410; i.cx = 320; i.cy = 240;
  i.model = distortion::DISTORTION_BROWN_CONRADY;
  for (int k = 0; k < 5; ++k) {
    i.coeffs[k] = 0.001f * k;
  }
  return i;
}

TEST(Calibration, ModelsAndOverrides)
{
  auto i = calibration();
  const auto sdk = camera_info(i);
  EXPECT_EQ(sdk.distortion_model, "plumb_bob");
  for (int k = 0; k < 5; ++k) {
    EXPECT_DOUBLE_EQ(sdk.d[k], i.coeffs[k]);
  }
  EXPECT_NO_THROW(validate_override(sdk, sdk, true));
  auto file = sdk;
  file.k[0] += 10;
  EXPECT_NO_THROW(validate_override(file, sdk, false));
  EXPECT_THROW(validate_override(file, sdk, true), std::runtime_error);
  file = sdk; file.width = 320;
  EXPECT_THROW(validate_override(file, sdk, false), std::runtime_error);
  file = sdk; file.d.resize(4);
  EXPECT_THROW(validate_info(file), std::runtime_error);
  file = sdk; file.k[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(validate_info(file), std::runtime_error);
  i.model = distortion::DISTORTION_INVERSE_BROWN_CONRADY;
  EXPECT_THROW(camera_info(i), std::runtime_error);
  i.model = distortion::DISTORTION_UNKNOWN;
  EXPECT_THROW(camera_info(i), std::runtime_error);
  for (auto & d : i.coeffs) {
    d = 0;
  }
  EXPECT_NO_THROW(camera_info(i));
  i.fx = 0;
  EXPECT_THROW(camera_info(i), std::runtime_error);
}

TEST(CalibrationFile, ExplicitFileNeverSilentlyFallsBack)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("offline_calibration_test");
  intrinsics i;
  i.width = 640; i.height = 480; i.fx = 500; i.fy = 500; i.cx = 320; i.cy = 240;
  const auto sdk = vidu_ros2::camera_info(i);
  EXPECT_EQ(vidu_ros2::load_info(*node, "color", "", sdk, false), sdk);
  EXPECT_THROW(
    vidu_ros2::load_info(*node, "color", "file:///nonexistent/vidu.yaml", sdk, false),
    std::runtime_error);
  EXPECT_THROW(
    vidu_ros2::load_info(*node, "color", "invalid://calibration", sdk, false),
    std::runtime_error);
  char path[] = "/tmp/vidu_calibration_XXXXXX.yaml";
  const int fd = mkstemps(path, 5);
  ASSERT_GE(fd, 0);
  close(fd);
  {
    std::ofstream file(path);
    file << "image_width: 640\nimage_height: 480\ncamera_name: color\n"
      "camera_matrix:\n  rows: 3\n  cols: 3\n  data: [510, 0, 320, 0, 500, 240, 0, 0, 1]\n"
      "distortion_model: plumb_bob\ndistortion_coefficients:\n  rows: 1\n  cols: 5\n"
      "  data: [0.1, 0, 0, 0, 0]\n"
      "rectification_matrix:\n  rows: 3\n  cols: 3\n  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n"
      "projection_matrix:\n  rows: 3\n  cols: 4\n"
      "  data: [510, 0, 320, 0, 0, 500, 240, 0, 0, 0, 1, 0]\n";
  }
  const std::string url = std::string("file://") + path;
  const auto loaded = vidu_ros2::load_info(*node, "color", url, sdk, false);
  EXPECT_DOUBLE_EQ(loaded.k[0], 510);
  EXPECT_DOUBLE_EQ(loaded.d[0], 0.1);
  EXPECT_THROW(vidu_ros2::load_info(*node, "depth", url, sdk, true), std::runtime_error);
  // Failed refreshes keep the previous validated snapshot, including when
  // the SDK model is unsupported or an explicitly configured file is missing.
  CalibrationCache cache;
  extrinsics transform;
  std::fill(transform.rotation, transform.rotation + 9, 0.0f);
  transform.rotation[0] = transform.rotation[4] = transform.rotation[8] = 1.0f;
  std::fill(transform.translation, transform.translation + 3, 0.0f);
  std::string error;
  ASSERT_TRUE(cache.refresh(*node, "color", url, i, transform, error));
  const auto previous = cache.info;
  EXPECT_FALSE(cache.refresh(*node, "color", "file:///nonexistent/vidu.yaml", i, transform, error));
  EXPECT_TRUE(cache.valid);
  EXPECT_EQ(cache.info, previous);
  EXPECT_FALSE(error.empty());
  auto unsupported = i;
  unsupported.model = distortion::DISTORTION_FTHETA;
  EXPECT_FALSE(cache.refresh(*node, "color", "", unsupported, transform, error));
  EXPECT_EQ(cache.info, previous);
  EXPECT_TRUE(cache.refresh(*node, "color", "", i, transform, error));
  EXPECT_TRUE(error.empty());
  EXPECT_EQ(cache.info, sdk);
  CalibrationCache empty;
  EXPECT_FALSE(empty.refresh(*node, "depth", "", unsupported, transform, error));
  EXPECT_FALSE(empty.valid);
  std::remove(path);
  node.reset();
  rclcpp::shutdown();
}

TEST(Recovery, AllThreeActiveCategoriesEscalateWithoutLatching)
{
  CameraHealth health;
  health.faults[CameraHealth::Calibration] = "no intrinsics";
  EXPECT_FALSE(health.error());
  health.faults[CameraHealth::Mode] = "alignment unconfirmed";
  EXPECT_FALSE(health.error());
  health.faults[CameraHealth::Data] = "unsupported pixels";
  EXPECT_TRUE(health.error());
  health.faults[CameraHealth::Data].clear();
  EXPECT_FALSE(health.error());
}

TEST(Calibration, KannalaBrandtAndUnknownModelDiagnostics)
{
  auto i = calibration();
  i.model = distortion::DISTORTION_KANNALA_BRANDT4;
  i.coeffs[4] = 0;
  const auto fish = camera_info(i);
  EXPECT_EQ(fish.distortion_model, "equidistant");
  ASSERT_EQ(fish.d.size(), 4u);
  for (size_t k = 0; k < 4; ++k) {
    EXPECT_DOUBLE_EQ(fish.d[k], i.coeffs[k]);
  }
  EXPECT_NO_THROW(validate_override(fish, fish, true));
  auto invalid = fish;
  invalid.d.push_back(0);
  EXPECT_THROW(validate_info(invalid), std::runtime_error);
  i.coeffs[4] = 0.1;
  EXPECT_THROW(camera_info(i), std::runtime_error);
  i.model = distortion::DISTORTION_INVERSE_BROWN_CONRADY;
  try {
    camera_info(i);
    FAIL() << "Unsupported model must not be relabelled";
  } catch (const std::runtime_error & error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("model=3"), std::string::npos);
    EXPECT_NE(message.find("D=["), std::string::npos);
  }
}

TEST(Color, ExplicitSdkByteOrderStillValidatesPixelLayout)
{
  cv::Mat bytes(1, 1, CV_8UC3, cv::Scalar(0, 0, 255));  // actual BGR red
  PDimage input;
  input.FillImgae(bytes.data, 1, 1, PFNC_RGB8);  // incorrect SDK metadata
  EXPECT_EQ(to_bgr(input, "bgr").at<cv::Vec3b>(0, 0), cv::Vec3b(0, 0, 255));
  EXPECT_EQ(to_bgr(input).at<cv::Vec3b>(0, 0), cv::Vec3b(255, 0, 0));
  EXPECT_EQ(to_bgr(input, "rgb").at<cv::Vec3b>(0, 0), cv::Vec3b(255, 0, 0));
  EXPECT_THROW(to_bgr(input, "guess"), std::runtime_error);
  input.FillImgae(bytes.data, 1, 1, PFNC_Mono8);
  EXPECT_THROW(to_bgr(input, "bgr"), std::runtime_error);
}

TEST(Recovery, FirstFrameBudgetStartsAtStreamAndSurvivesStartupDelay)
{
  using std::chrono::seconds;
  const auto initial = FrameWatchdog::Clock::time_point{};
  FrameWatchdog timer{10.0, 5.0};
  EXPECT_FALSE(timer.expired(initial + seconds(60)));  // No stream yet.
  const auto configured = initial + seconds(30);      // Device/calibration took time.
  timer.started(configured);
  EXPECT_FALSE(timer.expired(configured + seconds(2)));
  EXPECT_FALSE(timer.expired(configured + seconds(9)));
  EXPECT_TRUE(timer.expired(configured + seconds(10)));
  timer.received(configured + seconds(3));             // Slow but valid first frame.
  EXPECT_FALSE(timer.expired(configured + seconds(5)));
  EXPECT_FALSE(timer.expired(configured + seconds(7)));
  EXPECT_TRUE(timer.expired(configured + seconds(8)));  // 5s continuously silent.
  timer.received(configured + seconds(8));
  EXPECT_FALSE(timer.expired(configured + seconds(9)));
  timer.started(configured + seconds(60));            // Reopened stream gets startup budget.
  EXPECT_FALSE(timer.has_frame);
  EXPECT_FALSE(timer.expired(configured + seconds(65)));
  EXPECT_TRUE(timer.expired(configured + seconds(70)));
}

TEST(Extrinsics, OpticalTfPreservesDepthToColorDirection)
{
  extrinsics transform;
  const float rotation[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
  std::copy(rotation, rotation + 9, transform.rotation);
  transform.translation[0] = 0.03f;
  transform.translation[1] = 0.002f;
  transform.translation[2] = -0.001f;
  std_msgs::msg::Header header;
  header.frame_id = "color_optical";
  const auto tf = vidu_ros2::depth_to_color_tf(transform, header, "depth_optical");
  EXPECT_EQ(tf.header.frame_id, "color_optical");
  EXPECT_EQ(tf.child_frame_id, "depth_optical");
  EXPECT_NEAR(tf.transform.translation.x, 0.03, 1e-6);
  EXPECT_NEAR(tf.transform.translation.z, -0.001, 1e-6);
  EXPECT_NEAR(tf.transform.rotation.z, std::sqrt(0.5), 1e-6);
  EXPECT_NEAR(tf.transform.rotation.w, std::sqrt(0.5), 1e-6);
  EXPECT_THROW(
    vidu_ros2::depth_to_color_tf(transform, header, "color_optical"),
    std::runtime_error);
  transform.rotation[0] = 2;
  EXPECT_THROW(vidu_ros2::validate_extrinsics(transform), std::runtime_error);
}
