#include "my_robot_perception/camera_info_validation.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <my_robot_interface/msg/detected_object_array.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vidu_ros2_interfaces/msg/extrinsics.hpp>
#include <limits>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class DepthLocalizer : public rclcpp::Node
{
  using Image = sensor_msgs::msg::Image;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

public:
  DepthLocalizer()
  : Node("depth_localizer")
  {
    rcl_interfaces::msg::ParameterDescriptor startup_only;
    startup_only.read_only = true;
    startup_only.description = "Startup configuration; edit YAML and restart to change";

    const auto labels_topic = declare_parameter<std::string>(
      "labels_topic", "/perception/color_labels", startup_only);
    const auto depth_topic = declare_parameter<std::string>(
      "depth_topic", "/camera/depth", startup_only);
    const auto info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/camera/info/depth", startup_only);
    const auto output_topic = declare_parameter<std::string>(
      "points_topic", "/perception/object_points", startup_only);
    target_frame_ = declare_parameter<std::string>("target_frame", "", startup_only);
    objects_frame_ = declare_parameter<std::string>("objects_frame", "base_link", startup_only);
    const auto dimensions = declare_parameter<std::vector<double>>(
      "object_dimensions", {0.01, 0.01, 0.01}, startup_only);
    top_tolerance_ = declare_parameter<double>("top_surface_tolerance", 0.002, startup_only);
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.1, startup_only);
    if (objects_frame_.empty() || dimensions.size() != 3 ||
      !std::all_of(
        dimensions.begin(), dimensions.end(),
        [](double value) {return std::isfinite(value) && value > 0;}) ||
      !std::isfinite(top_tolerance_) || top_tolerance_ <= 0 ||
      !std::isfinite(tf_timeout_) || tf_timeout_ < 0)
    {
      throw std::invalid_argument("Invalid object frame, dimensions or top surface tolerance");
    }
    dimensions_.x = dimensions[0];
    dimensions_.y = dimensions[1];
    dimensions_.z = dimensions[2];
    min_depth_ = declare_parameter<double>("min_depth", 0.05, startup_only);
    max_depth_ = declare_parameter<double>("max_depth", 3.0, startup_only);
    depth_tolerance_ = declare_parameter<double>("depth_tolerance", 0.02, startup_only);
    min_pixels_ = declare_parameter<int>("min_valid_pixels", 5, startup_only);
    erosion_size_ = declare_parameter<int>("erosion_kernel", 1, startup_only);
    const double sync_slop = declare_parameter<double>("sync_slop", 0.03, startup_only);
    if (!std::isfinite(min_depth_) || !std::isfinite(max_depth_) ||
      !std::isfinite(depth_tolerance_) || !std::isfinite(sync_slop) ||
      min_depth_ <= 0 || max_depth_ <= min_depth_ || depth_tolerance_ <= 0 ||
      min_pixels_ < 1 || erosion_size_ < 1 || erosion_size_ % 2 == 0 || sync_slop <= 0)
    {
      throw std::invalid_argument("Invalid depth, pixel count, erosion or sync parameters");
    }
    input_timeout_ = declare_parameter<double>("input_timeout", 1.0, startup_only);
    surface_separation_ = declare_parameter<double>("surface_separation", 0.005, startup_only);
    if (!std::isfinite(input_timeout_) || input_timeout_ <= 0 ||
      !std::isfinite(surface_separation_) || surface_separation_ <= 0)
    {
      throw std::invalid_argument("Input timeout and surface separation must be positive");
    }
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
    objects_pub_ = create_publisher<my_robot_interface::msg::DetectedObjectArray>(
      declare_parameter<std::string>("objects_topic", "/perception/objects", startup_only), 10);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
        const auto error = my_robot_perception::camera_info_error(*info);
        if (error.empty()) {
          info_ = info;
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Rejected CameraInfo, retaining last valid calibration: %s", error.c_str());
        }
      });

    color_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      declare_parameter<std::string>("color_info_topic", "/camera/info/color", startup_only),
      rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
        const auto error = my_robot_perception::camera_info_error(*info);
        if (error.empty()) {color_info_ = info;} else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Rejected RGB CameraInfo, retaining valid calibration: %s", error.c_str());
        }
      });
    extrinsics_sub_ = create_subscription<vidu_ros2_interfaces::msg::Extrinsics>(
      declare_parameter<std::string>(
        "extrinsics_topic", "/camera/extrinsics/color_to_depth",
        startup_only),
      rclcpp::SensorDataQoS(),
      [this](vidu_ros2_interfaces::msg::Extrinsics::ConstSharedPtr transform) {
        cv::Matx33d rotation;
        cv::Vec3d translation;
        for (int i = 0; i < 9; ++i) {rotation.val[i] = transform->rotation[i];}
        for (int i = 0; i < 3; ++i) {translation[i] = transform->translation[i];}
        if (cv::checkRange(cv::Mat(rotation)) && cv::checkRange(cv::Mat(translation)) &&
        cv::norm(rotation * rotation.t() - cv::Matx33d::eye()) < 1e-4 &&
        std::abs(cv::determinant(rotation) - 1.0) < 1e-4)
        {
          extrinsics_ = transform;
          depth_to_color_rotation_ = rotation;
          depth_to_color_translation_ = translation;
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Invalid RGB/depth extrinsics; retaining valid calibration");
        }
      });

    // 按采集时间匹配；颜色标签图保留了原 RGB 图的时间戳。
    labels_sub_.subscribe(this, labels_topic, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, depth_topic, rmw_qos_profile_sensor_data);
    labels_sub_.registerCallback(
      [this](Image::ConstSharedPtr labels) {
        if (last_labels_header_.frame_id.empty()) {
          last_pair_received_ = std::chrono::steady_clock::now();
        }
        last_labels_header_ = labels->header;
      });
    last_pair_received_ = std::chrono::steady_clock::now();
    input_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this]() {
        if (!input_stale_ && !last_labels_header_.frame_id.empty() && std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_pair_received_).count() > input_timeout_)
        {
          auto invalid = last_labels_header_;
          invalid.stamp = now();
          publish_points(invalid, {});
          input_stale_ = true;
          RCLCPP_WARN(get_logger(), "RGB/depth pairing timed out; object observations invalidated");
        }
      });
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), labels_sub_, depth_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop));
    sync_->registerCallback(
      std::bind(
        &DepthLocalizer::localize, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(
      get_logger(), "Depth localization ready; publishing surface points and object centers");
  }

private:
  static float median(std::vector<float> values)
  {
    const auto middle = values.begin() + values.size() / 2;
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
  }

  struct TargetPoint
  {
    cv::Point3f xyz;
    uint8_t color;
    cv::Point seed;
    float depth;
  };

  // Legacy Vidu topic name is color_to_depth, but the SDK deproject example
  // applies this RGB extrinsic as P_rgb = R * P_tof + t (row-major, metres).
  // Project measured points only; holes remain unknown and are never filled.
  cv::Mat project_depth(const Image & labels, const Image & depth_msg, const cv::Mat & depth)
  {
    if (!info_ || depth_msg.header.frame_id != info_->header.frame_id ||
      depth_msg.width != info_->width || depth_msg.height != info_->height)
    {
      throw std::runtime_error("Waiting for depth CameraInfo matching depth grid");
    }
    const bool same_grid = labels.header.frame_id == depth_msg.header.frame_id;
    const auto & color = same_grid ? info_ : color_info_;
    if (!color || labels.header.frame_id.empty() ||
      labels.header.frame_id != color->header.frame_id ||
      labels.width != color->width || labels.height != color->height ||
      (!same_grid && (!extrinsics_ || extrinsics_->header.frame_id != color->header.frame_id)))
    {
      throw std::runtime_error("Waiting for RGB calibration and verified RGB/depth extrinsics");
    }
    if (!rays_info_ || rays_info_->k != info_->k || rays_info_->d != info_->d ||
      rays_info_->width != info_->width || rays_info_->height != info_->height ||
      rays_info_->distortion_model != info_->distortion_model)
    {
      std::vector<cv::Point2d> pixels;
      pixels.reserve(depth.total());
      for (int y = 0; y < depth.rows; ++y) {
        for (int x = 0; x < depth.cols; ++x) {pixels.emplace_back(x, y);}
      }
      const cv::Mat k(3, 3, CV_64F, const_cast<double *>(info_->k.data()));
      if (info_->distortion_model == "equidistant") {
        cv::fisheye::undistortPoints(pixels, depth_rays_, k, cv::Mat(info_->d));
      } else {
        cv::undistortPoints(pixels, depth_rays_, k, cv::Mat(info_->d));
      }
      rays_info_ = info_;
    }
    std::vector<cv::Point3d> measured;
    measured.reserve(depth.total());
    for (int y = 0; y < depth.rows; ++y) {
      for (int x = 0; x < depth.cols; ++x) {
        const double z = depth_msg.encoding == "32FC1" ? depth.at<float>(y, x) :
          depth.at<uint16_t>(y, x) * 0.001;
        if (!std::isfinite(z) || z < min_depth_ || z > max_depth_) {continue;}
        const auto & ray = depth_rays_[y * depth.cols + x];
        cv::Vec3d point(ray.x * z, ray.y * z, z);
        if (!same_grid) {point = depth_to_color_rotation_ * point + depth_to_color_translation_;}
        if (std::isfinite(point[0]) && std::isfinite(point[1]) &&
          std::isfinite(point[2]) && point[2] > 0)
        {
          measured.emplace_back(point[0], point[1], point[2]);
        }
      }
    }
    const float missing = std::numeric_limits<float>::quiet_NaN();
    cv::Mat mapped(labels.height, labels.width, CV_32FC3, cv::Scalar(missing, missing, missing));
    if (measured.empty()) {return mapped;}
    std::vector<cv::Point2d> projected;
    const cv::Mat k(3, 3, CV_64F, const_cast<double *>(color->k.data()));
    if (color->distortion_model == "equidistant") {
      cv::fisheye::projectPoints(
        measured, projected, cv::Vec3d(), cv::Vec3d(), k,
        cv::Mat(color->d));
    } else {
      cv::projectPoints(measured, cv::Vec3d(), cv::Vec3d(), k, cv::Mat(color->d), projected);
    }
    for (size_t i = 0; i < measured.size(); ++i) {
      const auto pixel = projected[i];
      if (!std::isfinite(pixel.x) || !std::isfinite(pixel.y) || pixel.x < -0.5 || pixel.y < -0.5 ||
        pixel.x >= mapped.cols - 0.5 || pixel.y >= mapped.rows - 0.5) {continue;}
      auto & previous = mapped.at<cv::Vec3f>(cvRound(pixel.y), cvRound(pixel.x));
      // Occluding foreground wins when several points hit the same RGB pixel.
      if (!std::isfinite(previous[2]) || measured[i].z < previous[2]) {
        previous = cv::Vec3f(measured[i].x, measured[i].y, measured[i].z);
      }
    }
    return mapped;
  }

  // Require a measured foreground boundary on at least one adequately sampled
  // side. A continuous background or missing surround is not evidence for an
  // object's depth. The RGB region remains valid independently of this check.
  bool filter_object_surface(
    const cv::Mat & mapped, const cv::Mat & labels,
    const cv::Rect & box, std::vector<cv::Point3f> & points) const
  {
    if (points.size() < 3) {return false;}
    const cv::Mat samples(static_cast<int>(points.size()), 3, CV_32F,
      const_cast<cv::Point3f *>(points.data()));
    cv::PCA plane(samples, cv::Mat(), cv::PCA::DATA_AS_ROW);
    if (plane.eigenvalues.at<float>(1) <= 1e-12f) {return false;}
    // Trim boundary outliers before estimating plane noise. A small amount of
    // background must not inflate the tolerance enough to erase a 1cm step.
    std::vector<float> residuals;
    for (const auto & point : points) {
      residuals.push_back(
        std::abs(
          plane.eigenvectors.at<float>(2, 0) * (point.x - plane.mean.at<float>(0, 0)) +
          plane.eigenvectors.at<float>(2, 1) * (point.y - plane.mean.at<float>(0, 1)) +
          plane.eigenvectors.at<float>(2, 2) * (point.z - plane.mean.at<float>(0, 2))));
    }
    const auto cutoff = std::max(surface_separation_ / 2, 3.0 * median(residuals));
    std::vector<cv::Point3f> support;
    for (size_t i = 0; i < points.size(); ++i) {
      if (residuals[i] <= cutoff) {support.push_back(points[i]);}
    }
    if (support.size() < static_cast<size_t>(std::max(3, min_pixels_))) {return false;}
    points = std::move(support);
    plane = cv::PCA(
      cv::Mat(static_cast<int>(points.size()), 3, CV_32F, points.data()),
      cv::Mat(), cv::PCA::DATA_AS_ROW);
    if (plane.eigenvalues.at<float>(1) <= 1e-12f) {return false;}
    const cv::Vec3f center(plane.mean.at<float>(0, 0), plane.mean.at<float>(0, 1),
      plane.mean.at<float>(0, 2));
    const cv::Vec3f normal(plane.eigenvectors.at<float>(2, 0),
      plane.eigenvectors.at<float>(2, 1), plane.eigenvectors.at<float>(2, 2));
    const double tolerance = surface_separation_ +
      3 * std::sqrt(std::max(0.0f, plane.eigenvalues.at<float>(2)));
    const int margin = std::max(3, std::min(box.width, box.height) / 10);
    const cv::Rect surround = cv::Rect(
      box.x - margin, box.y - margin,
      box.width + 2 * margin, box.height + 2 * margin) & cv::Rect(0, 0, mapped.cols, mapped.rows);
    std::array<size_t, 4> observed{}, behind{};
    const float facing = normal.dot(center) >= 0 ? 1.0f : -1.0f;
    for (int y = surround.y; y < surround.y + surround.height; ++y) {
      for (int x = surround.x; x < surround.x + surround.width; ++x) {
        if (box.contains({x, y}) || labels.at<uint8_t>(y, x) != 0) {continue;}
        const auto point = mapped.at<cv::Vec3f>(y, x);
        if (!std::isfinite(point[2])) {continue;}
        const size_t side = x < box.x ? 0 : (x >= box.x + box.width ? 1 :
          (y < box.y ? 2 : 3));
        ++observed[side];
        if (facing * normal.dot(point - center) > tolerance) {++behind[side];}
      }
    }
    for (size_t side = 0; side < observed.size(); ++side) {
      if (observed[side] >= static_cast<size_t>(min_pixels_) && behind[side] * 2 > observed[side]) {
        return true;
      }
    }
    return false;
  }

  void publish_points(std_msgs::msg::Header header, const std::vector<TargetPoint> & points)
  {
    if (points.empty() && !target_frame_.empty()) {header.frame_id = target_frame_;}
    sensor_msgs::msg::PointCloud2 output;
    output.header = header;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2Fields(
      7,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "label", 1, sensor_msgs::msg::PointField::UINT8,
      "u", 1, sensor_msgs::msg::PointField::UINT32,
      "v", 1, sensor_msgs::msg::PointField::UINT32,
      "depth", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(points.size());
    if (!points.empty()) {
      sensor_msgs::PointCloud2Iterator<float> x(output, "x"), y(output, "y"), z(output, "z");
      sensor_msgs::PointCloud2Iterator<float> depth(output, "depth");
      sensor_msgs::PointCloud2Iterator<uint8_t> label(output, "label");
      sensor_msgs::PointCloud2Iterator<uint32_t> u(output, "u"), v(output, "v");
      for (const auto & point : points) {
        *x = point.xyz.x; *y = point.xyz.y; *z = point.xyz.z; *label = point.color;
        *u = point.seed.x; *v = point.seed.y; *depth = point.depth;
        ++x; ++y; ++z; ++label; ++u; ++v; ++depth;
      }
    }
    points_pub_->publish(output);
    if (points.empty()) {
      my_robot_interface::msg::DetectedObjectArray empty;
      empty.header = header;
      empty.header.frame_id = objects_frame_;
      objects_pub_->publish(empty);
    }
  }

  void localize(const Image::ConstSharedPtr & labels_msg, const Image::ConstSharedPtr & depth_msg)
  {
    last_pair_received_ = std::chrono::steady_clock::now();
    input_stale_ = false;
    if (labels_msg->encoding != "mono8" ||
      (depth_msg->encoding != "32FC1" && depth_msg->encoding != "16UC1"))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Expected mono8 labels and 32FC1 (m) or 16UC1 (mm) depth");
      publish_points(labels_msg->header, {});
      return;
    }
    try {
      const auto labels = cv_bridge::toCvShare(labels_msg, "mono8");
      const auto depth = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
      const auto mapped = project_depth(*labels_msg, *depth_msg, depth->image);
      std::vector<TargetPoint> points;
      my_robot_interface::msg::DetectedObjectArray objects;
      objects.header = labels_msg->header;
      objects.header.frame_id = objects_frame_;
      tf2::Transform camera_to_objects;
      camera_to_objects.setIdentity();
      bool objects_tf_ready = true;
      try {
        if (labels_msg->header.frame_id != objects_frame_) {
          const auto transform = tf_buffer_->lookupTransform(
            objects_frame_, labels_msg->header.frame_id, rclcpp::Time(labels_msg->header.stamp),
            rclcpp::Duration::from_seconds(0));
          tf2::fromMsg(transform.transform, camera_to_objects);
        }
      } catch (const tf2::TransformException & error) {
        objects_tf_ready = false;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Object center TF unavailable: %s", error.what());
      }
      std::array<my_robot_interface::msg::DetectedObject, 3> best_objects;
      std::array<int, 3> best_areas{};
      const std::array<std::string, 3> ids{"red_cube", "green_cube", "blue_cube"};
      for (int color = 1; color <= 3; ++color) {
        cv::Mat mask = labels->image == color;
        cv::Mat components, stats, centroids;
        const int count = cv::connectedComponentsWithStats(mask, components, stats, centroids);
        for (int id = 1; id < count; ++id) {
          if (stats.at<int>(id, cv::CC_STAT_AREA) < min_pixels_) {
            continue;
          }
          const cv::Rect box(
            stats.at<int>(id, cv::CC_STAT_LEFT), stats.at<int>(id, cv::CC_STAT_TOP),
            stats.at<int>(id, cv::CC_STAT_WIDTH), stats.at<int>(id, cv::CC_STAT_HEIGHT));
          std::vector<cv::Point> pixels;
          cv::Mat region = components(box) == id;
          cv::findNonZero(region, pixels);
          const auto seed = pixels.front() + box.tl();
          if (erosion_size_ > 1) {
            cv::erode(
              region, region, cv::getStructuringElement(
                cv::MORPH_ELLIPSE, {erosion_size_, erosion_size_}),
              {-1, -1}, 1, cv::BORDER_CONSTANT, cv::Scalar(0));
            cv::findNonZero(region, pixels);
          }
          std::vector<cv::Point3f> valid_points;
          for (const auto & pixel : pixels) {
            const auto point = mapped.at<cv::Vec3f>(pixel + box.tl());
            const float z = point[2];
            if (std::isfinite(z) && z >= min_depth_ && z <= max_depth_) {
              valid_points.emplace_back(point[0], point[1], point[2]);
            }
          }
          std::sort(
            valid_points.begin(), valid_points.end(),
            [](const cv::Point3f & a, const cv::Point3f & b) {return a.z < b.z;});
          std::vector<cv::Point3f> inlier_points;
          for (size_t start = 0; start < valid_points.size(); ) {
            size_t end = start + 1;
            while (end < valid_points.size() &&
              valid_points[end].z - valid_points[end - 1].z <= depth_tolerance_) {++end;}
            if (end - start >= static_cast<size_t>(min_pixels_)) {
              std::vector<cv::Point3f> layer(valid_points.begin() + start,
                valid_points.begin() + end);
              if (filter_object_surface(mapped, labels->image, box, layer) &&
                layer.size() > inlier_points.size())
              {
                inlier_points = std::move(layer);
              }
            }
            start = end;
          }
          if (inlier_points.empty()) {continue;}
          std::vector<float> inliers, xs, ys;
          for (const auto & point : inlier_points) {
            inliers.push_back(point.z);
            xs.push_back(point.x);
            ys.push_back(point.y);
          }
          const float z = median(inliers);
          points.push_back({{median(xs), median(ys), z}, static_cast<uint8_t>(color), seed, z});

          // 第一版：平放、轴向对齐的已知尺寸物块，每种颜色保留最大连通区域。
          const int area = stats.at<int>(id, cv::CC_STAT_AREA);
          if (objects_tf_ready && area > best_areas[color - 1]) {
            std::vector<tf2::Vector3> surface;
            std::vector<float> heights;
            for (size_t i = 0; i < inlier_points.size(); ++i) {
              const auto point = camera_to_objects * tf2::Vector3(
                inlier_points[i].x, inlier_points[i].y, inlier_points[i].z);
              surface.push_back(point);
              heights.push_back(point.z());
            }
            std::sort(heights.begin(), heights.end());
            const double upper_z = heights[heights.size() * 9 / 10];
            tf2::Vector3 lower(1e9, 1e9, 0), upper(-1e9, -1e9, 0);
            std::vector<float> top_heights;
            for (const auto & point : surface) {
              if (std::abs(point.z() - upper_z) <= top_tolerance_) {
                lower.setX(std::min(lower.x(), point.x()));
                lower.setY(std::min(lower.y(), point.y()));
                upper.setX(std::max(upper.x(), point.x()));
                upper.setY(std::max(upper.y(), point.y()));
                top_heights.push_back(point.z());
              }
            }
            RCLCPP_DEBUG(
              get_logger(), "color=%d area=%d surface=%zu top=%zu upper_z=%.5f",
              color, area, surface.size(), top_heights.size(), upper_z);
            if (top_heights.size() >= static_cast<size_t>(min_pixels_)) {
              auto & object = best_objects[color - 1];
              object.object_id = ids[color - 1];
              object.color = color;
              object.dimensions = dimensions_;
              object.pose.position.x = (lower.x() + upper.x()) / 2;
              object.pose.position.y = (lower.y() + upper.y()) / 2;
              // objects_frame 的 Z 轴向上；上表面高度减半高得到物体中心。
              object.pose.position.z = median(top_heights) - dimensions_.z / 2;
              object.pose.orientation.w = 1.0;
              best_areas[color - 1] = area;
            }
          }
        }
      }

      for (size_t i = 0; i < best_objects.size(); ++i) {
        if (best_areas[i] > 0) {objects.objects.push_back(best_objects[i]);}
      }
      // 包括空列表，防止规划器继续使用上一次检测到的目标。
      objects_pub_->publish(objects);

      auto header = labels_msg->header;
      if (!points.empty() && !target_frame_.empty() && target_frame_ != header.frame_id) {
        // 使用采集时间查询 TF；暂不可用时丢弃本帧，避免使用错误的最新姿态。
        const auto transform = tf_buffer_->lookupTransform(
          target_frame_, header.frame_id, rclcpp::Time(header.stamp),
          rclcpp::Duration::from_seconds(tf_timeout_));
        for (auto & point : points) {
          geometry_msgs::msg::PointStamped source, result;
          source.header = header;
          source.point.x = point.xyz.x;
          source.point.y = point.xyz.y;
          source.point.z = point.xyz.z;
          tf2::doTransform(source, result, transform);
          point.xyz = cv::Point3f(result.point.x, result.point.y, result.point.z);
        }
      }
      if (!target_frame_.empty()) {
        header.frame_id = target_frame_;
      }
      publish_points(header, points);
    } catch (const tf2::TransformException & error) {
      publish_points(labels_msg->header, {});
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF unavailable: %s", error.what());
    } catch (const cv_bridge::Exception & error) {
      publish_points(labels_msg->header, {});
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Image conversion: %s", error.what());
    } catch (const cv::Exception & error) {
      publish_points(labels_msg->header, {});
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "OpenCV: %s", error.what());
    } catch (const std::runtime_error & error) {
      publish_points(labels_msg->header, {});
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
    }
  }

  std::string target_frame_, objects_frame_;
  geometry_msgs::msg::Vector3 dimensions_;
  double min_depth_, max_depth_, depth_tolerance_, top_tolerance_, tf_timeout_;
  int min_pixels_, erosion_size_;
  double input_timeout_, surface_separation_;
  bool input_stale_{false};
  std::chrono::steady_clock::time_point last_pair_received_;
  std_msgs::msg::Header last_labels_header_;
  rclcpp::TimerBase::SharedPtr input_timer_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_, color_info_, rays_info_;
  std::vector<cv::Point2d> depth_rays_;
  cv::Matx33d depth_to_color_rotation_;
  cv::Vec3d depth_to_color_translation_;
  vidu_ros2_interfaces::msg::Extrinsics::ConstSharedPtr extrinsics_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_sub_;
  rclcpp::Subscription<vidu_ros2_interfaces::msg::Extrinsics>::SharedPtr extrinsics_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  message_filters::Subscriber<Image> labels_sub_, depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<my_robot_interface::msg::DetectedObjectArray>::SharedPtr objects_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DepthLocalizer>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("depth_localizer"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
