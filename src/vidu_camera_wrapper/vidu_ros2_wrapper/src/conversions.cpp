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

#include "vidu_ros2_wrapper/conversions.hpp"
#include <tf2/LinearMath/Matrix3x3.h>

#include <GenICam/PFNC.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <camera_info_manager/camera_info_manager.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>


namespace vidu_ros2
{
void validate_extrinsics(const extrinsics & transform)
{
  cv::Matx33d rotation;
  for (int i = 0; i < 9; ++i) {
    rotation.val[i] = transform.rotation[i];
  }
  const cv::Vec3d translation(
    transform.translation[0], transform.translation[1], transform.translation[2]);
  if (!cv::checkRange(cv::Mat(rotation)) || !cv::checkRange(cv::Mat(translation)) ||
    cv::norm(rotation * rotation.t() - cv::Matx33d::eye()) > 1e-4 ||
    std::abs(cv::determinant(rotation) - 1.0) > 1e-4)
  {
    throw std::runtime_error("Invalid SDK depth-to-color extrinsics");
  }
}

geometry_msgs::msg::TransformStamped depth_to_color_tf(
  const extrinsics & transform, const std_msgs::msg::Header & color_header,
  const std::string & depth_frame)
{
  validate_extrinsics(transform);
  if (color_header.frame_id.empty() || depth_frame.empty() ||
    color_header.frame_id == depth_frame)
  {
    throw std::runtime_error("Invalid RGB/depth optical TF frames");
  }
  geometry_msgs::msg::TransformStamped result;
  result.header = color_header;
  result.child_frame_id = depth_frame;
  auto & output = result.transform;
  output.translation.x = transform.translation[0];
  output.translation.y = transform.translation[1];
  output.translation.z = transform.translation[2];
  const auto * r = transform.rotation;
  tf2::Quaternion q;
  tf2::Matrix3x3(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]).getRotation(q);
  q.normalize();
  output.rotation.x = q.x(); output.rotation.y = q.y();
  output.rotation.z = q.z(); output.rotation.w = q.w();
  return result;
}
namespace
{
bool valid_point(const cv::Vec3f & p)
{
  return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]) && p[2] > 0;
}
void require(bool condition, const std::string & reason)
{
  if (!condition) {throw std::runtime_error(reason);}
}
}  // namespace

cv::Mat image_view(const PDimage & image, int expected_type)
{
  require(
    image.GetImageData() && image.GetImageWidth() > 0 && image.GetImageHeight() > 0,
    "Empty SDK image");
  require(image.GetImageCVType() == expected_type, "Unexpected SDK image layout");
  // SDK images expose packed buffers, without a separate row-stride API.
  return cv::Mat(
    image.GetImageHeight(), image.GetImageWidth(), expected_type,
    image.GetImageData());
}

cv::Mat to_bgr(const PDimage & image, const std::string & input_order)
{
  require(
    input_order == "auto" || input_order == "rgb" || input_order == "bgr",
    "sdk_color_order must be auto, rgb or bgr");
  const auto format = image.GetImagePixelFormat();
  require(
    format == PFNC_BGR8 || format == PFNC_RGB8,
    "Unsupported color pixel format: " + std::to_string(format));
  require(
    image.GetImageData() && image.GetImageWidth() > 0 && image.GetImageHeight() > 0,
    "Empty color image");
  // SDK 2.1 returns -1 from GetImageCVType() for PFNC_BGR8. Both PFNC
  // formats above explicitly define packed 8-bit three-channel pixels.
  auto result = cv::Mat(
    image.GetImageHeight(), image.GetImageWidth(), CV_8UC3,
    image.GetImageData());
  if (input_order == "rgb" || (input_order == "auto" && format == PFNC_RGB8)) {
    cv::Mat converted;
    cv::cvtColor(result, converted, cv::COLOR_RGB2BGR);
    return converted;
  }
  return result;
}

cv::Mat depth_metres(const cv::Mat & xyz)
{
  require(!xyz.empty() && xyz.type() == CV_32FC3, "Expected XYZ float32 in metres");
  cv::Mat depth(xyz.size(), CV_32FC1);
  for (int y = 0; y < xyz.rows; ++y) {
    for (int x = 0; x < xyz.cols; ++x) {
      const auto p = xyz.at<cv::Vec3f>(y, x);
      depth.at<float>(y, x) = valid_point(p) ? p[2] : std::numeric_limits<float>::quiet_NaN();
    }
  }
  return depth;
}

sensor_msgs::msg::PointCloud2 to_cloud(
  const cv::Mat & bgr, const cv::Mat & xyz, const std_msgs::msg::Header & header)
{
  require(
    !xyz.empty() && xyz.type() == CV_32FC3 && bgr.type() == CV_8UC3 &&
    bgr.size() == xyz.size(), "Cloud XYZ and color must share a pixel grid");
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.height = 1;
  cloud.is_dense = true;  // Invalid points are omitted.
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(xyz.total());
  sensor_msgs::PointCloud2Iterator<float> ox(cloud, "x"), oy(cloud, "y"), oz(cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> r(cloud, "r"), g(cloud, "g"), b(cloud, "b");
  size_t count = 0;
  for (int y = 0; y < xyz.rows; ++y) {
    for (int x = 0; x < xyz.cols; ++x) {
      const auto p = xyz.at<cv::Vec3f>(y, x);
      if (!valid_point(p)) {continue;}
      const auto color = bgr.at<cv::Vec3b>(y, x);
      *ox = p[0]; *oy = p[1]; *oz = p[2];
      *r = color[2]; *g = color[1]; *b = color[0];
      ++ox; ++oy; ++oz; ++r; ++g; ++b; ++count;
    }
  }
  modifier.resize(count);
  return cloud;
}

void validate_info(const sensor_msgs::msg::CameraInfo & info)
{
  const auto finite = [](double v) {return std::isfinite(v);};
  require(
    info.width > 0 && info.height > 0 && info.k[0] > 0 && info.k[4] > 0 &&
    std::all_of(info.k.begin(), info.k.end(), finite) &&
    std::all_of(info.d.begin(), info.d.end(), finite) &&
    std::all_of(info.r.begin(), info.r.end(), finite) &&
    std::all_of(info.p.begin(), info.p.end(), finite), "Invalid camera calibration values");
  require(
    info.k[1] == 0 && info.k[3] == 0 && info.k[6] == 0 && info.k[7] == 0 &&
    info.k[8] == 1, "Unsupported camera matrix (expected zero skew)");
  require(
    (info.distortion_model == "plumb_bob" && info.d.size() == 5) ||
    (info.distortion_model == "rational_polynomial" && info.d.size() == 8) ||
    (info.distortion_model == "equidistant" && info.d.size() == 4),
    "Unsupported distortion model or coefficient count");
  const cv::Mat rotation(3, 3, CV_64F, const_cast<double *>(info.r.data()));
  require(
    cv::norm(rotation * rotation.t() - cv::Mat::eye(3, 3, CV_64F)) < 1e-6 &&
    std::abs(cv::determinant(rotation) - 1.0) < 1e-6, "Invalid rectification rotation");
  require(
    info.p[8] == 0 && info.p[9] == 0 && info.p[11] == 0,
    "Invalid projection matrix bottom row");
  require(
    info.p[0] > 0 && info.p[5] > 0 && info.p[10] == 1,
    "Invalid projection matrix");
  require(
    info.binning_x <= 1 && info.binning_y <= 1 && info.roi.x_offset == 0 &&
    info.roi.y_offset == 0 && info.roi.width == 0 && info.roi.height == 0 &&
    !info.roi.do_rectify, "ROI/binning calibration is not supported for the raw SDK grid");
}

sensor_msgs::msg::CameraInfo camera_info(const intrinsics & i)
{
  sensor_msgs::msg::CameraInfo info;
  require(i.width > 0 && i.height > 0, "Invalid SDK calibration dimensions");
  info.width = i.width; info.height = i.height;
  info.k = {i.fx, 0, i.cx, 0, i.fy, i.cy, 0, 0, 1};
  info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  info.p = {i.fx, 0, i.cx, 0, 0, i.fy, i.cy, 0, 0, 0, 1, 0};
  const bool brown = i.model == distortion::DISTORTION_BROWN_CONRADY ||
    i.model == distortion::DISTORTION_UNKNOWN;
  if (i.model == distortion::DISTORTION_KANNALA_BRANDT4) {
    // SDK camParaDef.h and deproject example define k1,k2,k3,k4,0.
    require(i.coeffs[4] == 0, "Kannala-Brandt4 reserved fifth coefficient must be zero");
    info.distortion_model = "equidistant";
    info.d.assign(i.coeffs, i.coeffs + 4);
  } else if (brown) {
    info.distortion_model = "plumb_bob";
    info.d.assign(i.coeffs, i.coeffs + 5);
    if (i.model == distortion::DISTORTION_UNKNOWN) {
      require(
        std::all_of(info.d.begin(), info.d.end(), [](double d) {return d == 0;}),
        "Rectilinear SDK model has nonzero distortion");
    }
  } else {
    std::ostringstream reason;
    reason << "Unsupported SDK distortion model=" << static_cast<int>(i.model)
           << " size=" << i.width << "x" << i.height << " fx=" << i.fx << " fy=" << i.fy
           << " cx=" << i.cx << " cy=" << i.cy << " D=[";
    for (int k = 0; k < 5; ++k) {
      reason << (k ? "," : "") << i.coeffs[k];
    }
    reason << "]";
    throw std::runtime_error(reason.str());
  }
  validate_info(info);
  return info;
}

void validate_override(
  const sensor_msgs::msg::CameraInfo & info,
  const sensor_msgs::msg::CameraInfo & sdk, bool depth)
{
  validate_info(info);
  require(
    info.width == sdk.width && info.height == sdk.height,
    "Calibration file dimensions do not match SDK image grid");
  if (depth) {
    const auto close = [](double a, double b) {
        return std::abs(a - b) <= 1e-6 * std::max(1.0, std::max(std::abs(a), std::abs(b)));
      };
    require(
      info.distortion_model == sdk.distortion_model && info.d.size() == sdk.d.size() &&
      std::equal(info.k.begin(), info.k.end(), sdk.k.begin(), close) &&
      std::equal(
        info.d.begin(), info.d.end(),
        sdk.d.begin(), close),
      "Depth calibration conflicts with SDK XYZ/registration; "
      "changing CameraInfo cannot recalibrate SDK");
  }
}
sensor_msgs::msg::CameraInfo load_info(
  rclcpp::Node & node, const std::string & name,
  const std::string & url, const sensor_msgs::msg::CameraInfo & sdk, bool depth)
{
  if (url.empty()) {
    RCLCPP_INFO(node.get_logger(), "%s calibration: SDK", name.c_str());
    return sdk;
  }
  // Startup snapshot only. Destroy the manager/service before streaming, so
  // set_camera_info cannot silently change calibration behind this snapshot.
  camera_info_manager::CameraInfoManager manager(&node, name);
  if (!manager.validateURL(url) || !manager.loadCameraInfo(url)) {
    throw std::runtime_error("Cannot load " + name + " calibration: " + url);
  }
  auto info = manager.getCameraInfo();
  vidu_ros2::validate_override(info, sdk, depth);
  RCLCPP_INFO(node.get_logger(), "%s calibration: %s", name.c_str(), url.c_str());
  return info;
}
bool CalibrationCache::refresh(
  rclcpp::Node & node, const std::string & name,
  const std::string & url, const intrinsics & intrinsic, const extrinsics & transform,
  std::string & error)
{
  try {
    auto candidate = load_info(node, name, url, camera_info(intrinsic), name == "depth");
    if (name == "color") {validate_extrinsics(transform);}
    info = std::move(candidate);
    extrinsic = transform;
    valid = true;
    error.clear();
    return true;
  } catch (const std::exception & failure) {
    error = failure.what();
    return false;
  }
}

void FrameWatchdog::started(Clock::time_point now)
{
  last_activity = now;
  active = true;
  has_frame = false;
}

void FrameWatchdog::received(Clock::time_point now)
{
  last_activity = now;
  has_frame = true;
}

double FrameWatchdog::idle_seconds(Clock::time_point now) const
{
  return std::chrono::duration<double>(now - last_activity).count();
}

bool FrameWatchdog::expired(Clock::time_point now) const
{
  return active && idle_seconds(now) >= (has_frame ? frame_timeout : startup_timeout);
}

bool CameraHealth::error() const
{
  return std::all_of(
    faults.begin(), faults.end(),
    [](const std::string & fault) {return !fault.empty();});
}
}  // namespace vidu_ros2
