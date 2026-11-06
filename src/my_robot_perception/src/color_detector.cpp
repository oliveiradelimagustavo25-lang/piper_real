#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

class ColorDetector : public rclcpp::Node
{
public:
  ColorDetector() : Node("color_detector")
  {
    const auto image_topic = declare_parameter<std::string>(
      "image_topic", "/camera_sensor/image_raw");
    const auto labels_topic = declare_parameter<std::string>(
      "labels_topic", "/perception/color_labels");
    const auto debug_topic = declare_parameter<std::string>(
      "debug_topic", "/perception/color_debug");
    show_window_ = declare_parameter<bool>("show_window", false);
    publish_debug_ = declare_parameter<bool>("publish_debug", true);
    min_area_ = declare_parameter<int>("min_area", 12);
    max_area_ = declare_parameter<int>("max_area", 100000);
    const int kernel_size = declare_parameter<int>("morphology_kernel", 3);
    if (min_area_ < 1 || max_area_ < min_area_ ||
      kernel_size < 1 || kernel_size % 2 == 0)
    {
      throw std::invalid_argument("Invalid area limits or morphology_kernel (positive odd required)");
    }
    kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, {kernel_size, kernel_size});

    colors_ = {{
      {"red", 1, {0, 0, 255}, {}, {}},
      {"green", 2, {0, 255, 0}, {}, {}},
      {"blue", 3, {255, 0, 0}, {}, {}}
    }};
    const std::array<std::vector<int64_t>, 3> lower = {{
      {170, 100, 50}, {35, 80, 50}, {90, 80, 50}}};
    const std::array<std::vector<int64_t>, 3> upper = {{
      {10, 255, 255}, {85, 255, 255}, {135, 255, 255}}};
    for (size_t i = 0; i < colors_.size(); ++i) {
      auto & color = colors_[i];
      const auto lo = declare_parameter<std::vector<int64_t>>(color.name + ".lower", lower[i]);
      const auto hi = declare_parameter<std::vector<int64_t>>(color.name + ".upper", upper[i]);
      validate_bounds(color.name, lo, hi);
      color.lower = cv::Scalar(lo[0], lo[1], lo[2]);
      color.upper = cv::Scalar(hi[0], hi[1], hi[2]);
    }

    labels_pub_ = create_publisher<sensor_msgs::msg::Image>(labels_topic, 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_topic, 10);
    // Best effort 订阅同时兼容仿真与常见相机的 SensorDataQoS。
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      std::bind(&ColorDetector::image_callback, this, std::placeholders::_1));
    if (show_window_) {
      cv::namedWindow("Color detector", cv::WINDOW_NORMAL);
    }
    RCLCPP_INFO(get_logger(), "Listening on %s; labels: 0=background, 1=red, 2=green, 3=blue",
      image_topic.c_str());
  }

  ~ColorDetector() override
  {
    if (show_window_) {
      cv::destroyWindow("Color detector");
    }
  }

private:
  struct ColorRange
  {
    std::string name;
    uint8_t label;
    cv::Scalar display_color;
    cv::Scalar lower;
    cv::Scalar upper;
  };

  static void validate_bounds(
    const std::string & name, const std::vector<int64_t> & lo,
    const std::vector<int64_t> & hi)
  {
    if (lo.size() != 3 || hi.size() != 3) {
      throw std::invalid_argument(name + ": HSV bounds need exactly 3 integers");
    }
    for (size_t i = 0; i < 3; ++i) {
      const int limit = i == 0 ? 179 : 255;
      if (lo[i] < 0 || hi[i] < 0 || lo[i] > limit || hi[i] > limit ||
        (i > 0 && lo[i] > hi[i]))
      {
        throw std::invalid_argument(name + ": invalid HSV bounds");
      }
    }
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    try {
      const auto input = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat hsv;
      cv::cvtColor(input->image, hsv, cv::COLOR_BGR2HSV);
      cv::Mat labels = cv::Mat::zeros(hsv.size(), CV_8UC1);
      const bool need_debug = publish_debug_ || show_window_; // 仅在需要时生成观看调试图像，避免额外开销。
      cv::Mat debug;
      if (need_debug) {
        debug = input->image.clone(); // 仅用于调试显示，避免修改原图数据。
      }

      for (const auto & color : colors_) {
        cv::Mat mask;
        if (color.lower[0] <= color.upper[0]) {
          cv::inRange(hsv, color.lower, color.upper, mask);
        } else {
          // 红色跨越 H=179/0，合并两个区间。
          cv::Mat high_mask;
          cv::inRange(hsv, cv::Scalar(0, color.lower[1], color.lower[2]), color.upper, mask);
          cv::inRange(hsv, color.lower,
            cv::Scalar(179, color.upper[1], color.upper[2]), high_mask);
          cv::bitwise_or(mask, high_mask, mask);
        }
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel_);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_);
        // 阈值有重叠时按 red、green、blue 顺序分配，避免重复计数。
        mask.setTo(0, labels != 0);

        cv::Mat components, stats, centroids;
        const int count = cv::connectedComponentsWithStats(
          mask, components, stats, centroids, 8, CV_32S);
        int accepted = 0;
        for (int id = 1; id < count; ++id) {
          const int area = stats.at<int>(id, cv::CC_STAT_AREA);
          if (area < min_area_ || area > max_area_) {
            continue;
          }
          // 保留像素区域而非仅发布中心点，便于深度节点过滤异常值。
          labels.setTo(color.label, components == id);
          ++accepted;
          if (need_debug) {
            const cv::Rect box(
              stats.at<int>(id, cv::CC_STAT_LEFT), stats.at<int>(id, cv::CC_STAT_TOP),
              stats.at<int>(id, cv::CC_STAT_WIDTH), stats.at<int>(id, cv::CC_STAT_HEIGHT));
            const cv::Point center(
              cvRound(centroids.at<double>(id, 0)), cvRound(centroids.at<double>(id, 1)));
            cv::rectangle(debug, box, color.display_color, 2);
            cv::circle(debug, center, 3, color.display_color, -1);
            cv::putText(debug, color.name + " " + std::to_string(accepted),
              cv::Point(box.x, std::max(15, box.y - 5)), cv::FONT_HERSHEY_SIMPLEX,
              0.5, color.display_color, 1);
          }
        }
      }

      // 每帧发布，包括全零图，避免检测丢失后下游继续使用旧结果。
      labels_pub_->publish(*cv_bridge::CvImage(msg->header, "mono8", labels).toImageMsg()); //这个是给下游深度图的标签图，0=背景，1=红色，2=绿色，3=蓝色
      if (publish_debug_) {
        debug_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", debug).toImageMsg()); //这个是给rviz调试用的彩色图，标记了检测到的颜色区域
      }
      if (show_window_) {
        cv::imshow("Color detector", debug); //这个是给自己看的彩色图，标记了检测到的颜色区域
        cv::waitKey(1);
      }
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Image conversion: %s", error.what());
    } catch (const cv::Exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "OpenCV: %s", error.what());
    }
  }

  std::array<ColorRange, 3> colors_;
  cv::Mat kernel_;
  int min_area_, max_area_;
  bool show_window_, publish_debug_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr labels_pub_, debug_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ColorDetector>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("color_detector"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
