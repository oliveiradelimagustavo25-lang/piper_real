#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <sensor_msgs/msg/camera_info.hpp>

namespace my_robot_perception
{
// These consumers use raw pixels with K/D, without applying ROI/binning or R/P.
inline std::string camera_info_error(const sensor_msgs::msg::CameraInfo & info)
{
  if (info.width == 0 || info.height == 0 || info.header.frame_id.empty()) {
    return "Missing calibration dimensions or optical frame";
  }
  const auto finite = [](double v) {return std::isfinite(v);};
  if (!std::all_of(info.k.begin(), info.k.end(), finite) ||
    !std::all_of(info.d.begin(), info.d.end(), finite) || info.k[0] <= 0 || info.k[4] <= 0 ||
    info.k[1] != 0 || info.k[3] != 0 || info.k[6] != 0 || info.k[7] != 0 || info.k[8] != 1)
  {
    return "Invalid intrinsics or unsupported skew";
  }
  if (!((info.distortion_model == "plumb_bob" && info.d.size() == 5) ||
    (info.distortion_model == "rational_polynomial" && info.d.size() == 8) ||
    (info.distortion_model == "equidistant" && info.d.size() == 4)))
  {
    return "Unsupported distortion model or coefficient count";
  }
  if (info.binning_x > 1 || info.binning_y > 1 || info.roi.x_offset != 0 ||
    info.roi.y_offset != 0 || info.roi.width != 0 || info.roi.height != 0 || info.roi.do_rectify)
  {
    return "ROI/binning requires calibration for the actual output grid";
  }
  return "";
}
}  // namespace my_robot_perception
