#include <algorithm>
#include <array>
#include <cmath>
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
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class DepthLocalizer : public rclcpp::Node
{
  using Image = sensor_msgs::msg::Image;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

public:
  DepthLocalizer() : Node("depth_localizer")
  {
    const auto labels_topic = declare_parameter<std::string>(
      "labels_topic", "/perception/color_labels");
    const auto depth_topic = declare_parameter<std::string>(
      "depth_topic", "/camera_sensor/depth/image_raw");
    const auto info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/camera_sensor/camera_info");
    const auto output_topic = declare_parameter<std::string>(
      "points_topic", "/perception/object_points");
    target_frame_ = declare_parameter<std::string>("target_frame", "");
    objects_frame_ = declare_parameter<std::string>("objects_frame", "base_link");
    const auto dimensions = declare_parameter<std::vector<double>>(
      "object_dimensions", {0.01, 0.01, 0.01});
    top_tolerance_ = declare_parameter<double>("top_surface_tolerance", 0.002);
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.1);
    if (objects_frame_.empty() || dimensions.size() != 3 ||
      !std::all_of(dimensions.begin(), dimensions.end(),
      [](double value) { return std::isfinite(value) && value > 0; }) ||
      !std::isfinite(top_tolerance_) || top_tolerance_ <= 0 ||
      !std::isfinite(tf_timeout_) || tf_timeout_ < 0)
    {
      throw std::invalid_argument("Invalid object frame, dimensions or top surface tolerance");
    }
    dimensions_.x = dimensions[0];
    dimensions_.y = dimensions[1];
    dimensions_.z = dimensions[2];
    min_depth_ = declare_parameter<double>("min_depth", 0.05);
    max_depth_ = declare_parameter<double>("max_depth", 3.0);
    depth_tolerance_ = declare_parameter<double>("depth_tolerance", 0.02);
    min_pixels_ = declare_parameter<int>("min_valid_pixels", 5);
    erosion_size_ = declare_parameter<int>("erosion_kernel", 1);
    const double sync_slop = declare_parameter<double>("sync_slop", 0.03);
    if (!std::isfinite(min_depth_) || !std::isfinite(max_depth_) ||
      !std::isfinite(depth_tolerance_) || !std::isfinite(sync_slop) ||
      min_depth_ <= 0 || max_depth_ <= min_depth_ || depth_tolerance_ <= 0 ||
      min_pixels_ < 1 || erosion_size_ < 1 || erosion_size_ % 2 == 0 || sync_slop <= 0)
    {
      throw std::invalid_argument("Invalid depth, pixel count, erosion or sync parameters");
    }
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
    objects_pub_ = create_publisher<my_robot_interface::msg::DetectedObjectArray>(
      declare_parameter<std::string>("objects_topic", "/perception/objects"), 10);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) { info_ = info; });

    // 按采集时间匹配；颜色标签图保留了原 RGB 图的时间戳。
    labels_sub_.subscribe(this, labels_topic, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, depth_topic, rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), labels_sub_, depth_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop));
    sync_->registerCallback(std::bind(
      &DepthLocalizer::localize, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Depth localization ready; publishing surface points and object centers");
  }

private:
  static float median(std::vector<float> values)
  {
    const auto middle = values.begin() + values.size() / 2;
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
  }

  void localize(const Image::ConstSharedPtr & labels_msg, const Image::ConstSharedPtr & depth_msg)
  {
    // 必须提供与 RGB 对齐的深度和对应 CameraInfo，不能仅凭相同尺寸认定对齐。
    if (!info_ || labels_msg->header.frame_id.empty() ||
      labels_msg->header.frame_id != depth_msg->header.frame_id ||
      info_->header.frame_id != labels_msg->header.frame_id ||
      labels_msg->width != depth_msg->width || labels_msg->height != depth_msg->height ||
      labels_msg->width != info_->width || labels_msg->height != info_->height)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for matching CameraInfo and RGB-aligned depth (same optical frame and dimensions)");
      return;
    }
    if (labels_msg->encoding != "mono8" ||
      (depth_msg->encoding != "32FC1" && depth_msg->encoding != "16UC1"))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Expected mono8 labels and 32FC1 (m) or 16UC1 (mm) depth");
      return;
    }
    const double fx = info_->k[0], fy = info_->k[4];
    if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0 || fy <= 0 ||
      !std::isfinite(info_->k[2]) || !std::isfinite(info_->k[5]) ||
      !std::all_of(info_->d.begin(), info_->d.end(), [](double d) { return std::isfinite(d); }))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid camera intrinsics");
      return;
    }
    // 本节点使用原始彩色图的 K/D；整流图需提供对应的等效内参。
    const bool distorted = std::any_of(info_->d.begin(), info_->d.end(),
      [](double d) { return d != 0.0; });
    if (distorted && info_->distortion_model != "plumb_bob" &&
      info_->distortion_model != "rational_polynomial")
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Unsupported distortion model");
      return;
    }

    try {
      const auto labels = cv_bridge::toCvShare(labels_msg, "mono8");
      const auto depth = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
      std::vector<cv::Point3f> points;
      std::vector<uint8_t> colors;
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
            rclcpp::Duration::from_seconds(tf_timeout_));
          tf2::fromMsg(transform.transform, camera_to_objects);
        }
      } catch (const tf2::TransformException & error) {
        objects_tf_ready = false;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Object center TF unavailable: %s", error.what());
      }
      std::array<my_robot_interface::msg::DetectedObject, 3> best_objects;
      std::array<int, 3> best_areas{};
      const std::array<std::string, 3> ids{"red_cube", "green_cube", "blue_cube"};
      for (int color = 1; color <= 3; ++color) {
        cv::Mat mask = labels->image == color;
        if (erosion_size_ > 1) {
          cv::erode(mask, mask, cv::getStructuringElement(
            cv::MORPH_ELLIPSE, {erosion_size_, erosion_size_}));
        }
        cv::Mat components, stats, centroids;
        const int count = cv::connectedComponentsWithStats(mask, components, stats, centroids);
        for (int id = 1; id < count; ++id) {
          if (stats.at<int>(id, cv::CC_STAT_AREA) < min_pixels_) {
            continue;
          }
          std::vector<cv::Point> pixels;
          cv::findNonZero(components == id, pixels);
          std::vector<float> depths;
          std::vector<cv::Point> valid_pixels;
          for (const auto & pixel : pixels) {
            const float z = depth_msg->encoding == "32FC1" ?
              depth->image.at<float>(pixel) : depth->image.at<uint16_t>(pixel) * 0.001f;
            if (std::isfinite(z) && z >= min_depth_ && z <= max_depth_) {
              depths.push_back(z);
              valid_pixels.push_back(pixel);
            }
          }
          if (depths.size() < static_cast<size_t>(min_pixels_)) {
            continue;
          }
          const float center_depth = median(depths);
          std::vector<float> inliers;
          std::vector<cv::Point2d> inlier_pixels;
          cv::Point2d uv(0, 0);
          for (size_t i = 0; i < depths.size(); ++i) {
            if (std::abs(depths[i] - center_depth) <= depth_tolerance_) {
              inliers.push_back(depths[i]);
              inlier_pixels.emplace_back(valid_pixels[i].x, valid_pixels[i].y);
              uv += cv::Point2d(valid_pixels[i].x, valid_pixels[i].y);
            }
          }
          if (inliers.size() < static_cast<size_t>(min_pixels_)) {
            continue;
          }
          uv *= 1.0 / inliers.size();
          std::vector<cv::Point2d> ray;
          const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
            fx, 0, info_->k[2], 0, fy, info_->k[5], 0, 0, 1);
          cv::undistortPoints(std::vector<cv::Point2d>{uv}, ray, camera_matrix,
            distorted ? cv::Mat(info_->d) : cv::Mat());
          const float z = median(inliers);
          points.emplace_back(ray[0].x * z, ray[0].y * z, z);
          colors.push_back(static_cast<uint8_t>(color));

          // 第一版：平放、轴向对齐的已知尺寸物块，每种颜色保留最大连通区域。
          const int area = stats.at<int>(id, cv::CC_STAT_AREA);
          if (objects_tf_ready && area > best_areas[color - 1]) {
            std::vector<cv::Point2d> rays;
            cv::undistortPoints(inlier_pixels, rays, camera_matrix,
              distorted ? cv::Mat(info_->d) : cv::Mat());
            std::vector<tf2::Vector3> surface;
            std::vector<float> heights;
            for (size_t i = 0; i < rays.size(); ++i) {
              const auto point = camera_to_objects * tf2::Vector3(
                rays[i].x * inliers[i], rays[i].y * inliers[i], inliers[i]);
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
            RCLCPP_DEBUG(get_logger(), "color=%d area=%d surface=%zu top=%zu upper_z=%.5f",
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
        if (best_areas[i] > 0) { objects.objects.push_back(best_objects[i]); }
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
          source.point.x = point.x;
          source.point.y = point.y;
          source.point.z = point.z;
          tf2::doTransform(source, result, transform);
          point = cv::Point3f(result.point.x, result.point.y, result.point.z);
        }
      }
      if (!target_frame_.empty()) {
        header.frame_id = target_frame_;
      }
      sensor_msgs::msg::PointCloud2 output;
      output.header = header;
      sensor_msgs::PointCloud2Modifier modifier(output);
      modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "label", 1, sensor_msgs::msg::PointField::UINT8);
      modifier.resize(points.size());
      if (!points.empty()) {
        sensor_msgs::PointCloud2Iterator<float> x(output, "x"), y(output, "y"), z(output, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> label(output, "label");
        for (size_t i = 0; i < points.size(); ++i, ++x, ++y, ++z, ++label) {
          *x = points[i].x;
          *y = points[i].y;
          *z = points[i].z;
          *label = colors[i];
        }
      }
      // 无有效物块时发布空点云，表示本帧无检测结果。
      points_pub_->publish(output);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF unavailable: %s", error.what());
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Image conversion: %s", error.what());
    } catch (const cv::Exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "OpenCV: %s", error.what());
    }
  }

  std::string target_frame_, objects_frame_;
  geometry_msgs::msg::Vector3 dimensions_;
  double min_depth_, max_depth_, depth_tolerance_, top_tolerance_, tf_timeout_;
  int min_pixels_, erosion_size_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
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
