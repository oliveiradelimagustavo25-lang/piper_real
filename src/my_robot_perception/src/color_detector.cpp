#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <cstring>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class ColorDetector : public rclcpp::Node
{
  using Image = sensor_msgs::msg::Image;

public:
  ColorDetector()
  : Node("color_detector")
  {
    rcl_interfaces::msg::ParameterDescriptor startup_only;
    startup_only.read_only = true;
    startup_only.description = "Startup configuration; edit YAML and restart to change";

    const auto image_topic = declare_parameter<std::string>(
      "image_topic", "/camera/color/origin", startup_only);
    enable_detection_ = declare_parameter<bool>("enable_detection", true, startup_only);
    show_window_ = declare_parameter<bool>("show_window", false, startup_only);
    window_name_ = enable_detection_ ? "Color detector" : "RGB preview";
    if (show_window_) {
      cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    }
    input_timeout_ = declare_parameter<double>("input_timeout", 1.0, startup_only);
    if (!std::isfinite(input_timeout_) || input_timeout_ <= 0) {
      throw std::invalid_argument("input_timeout must be positive");
    }
    display_timer_ = create_wall_timer(
      std::chrono::milliseconds(20), [this]() {
        while (!pending_.empty() && std::chrono::duration<double>(
          std::chrono::steady_clock::now() - pending_.front().received).count() >=
        display_timeout_)
        {
          render(pending_.front());
          pending_.pop_front();
        }
        if (!last_frame_.image.empty() && !input_stale_ && std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_rgb_received_).count() > input_timeout_)
        {
          pending_.clear();
          auto stale = last_frame_;
          render(stale, nullptr, true);
          input_stale_ = true;
          RCLCPP_WARN(get_logger(), "RGB input timed out; displayed depth is invalid");
        }
      });
    if (!enable_detection_) {
      image_sub_ = create_subscription<Image>(
        image_topic, rclcpp::SensorDataQoS(),
        [this](Image::ConstSharedPtr image) {image_callback(image);});
      RCLCPP_INFO(get_logger(), "RGB preview on %s; detection disabled", image_topic.c_str());
      return;
    }
    const auto labels_topic = declare_parameter<std::string>(
      "labels_topic", "/perception/color_labels", startup_only);
    const auto debug_topic = declare_parameter<std::string>(
      "debug_topic", "/perception/color_debug", startup_only);
    const auto points_topic =
      declare_parameter<std::string>("depth_points_topic", "", startup_only);
    display_timeout_ = declare_parameter<double>("display_timeout", 0.15, startup_only);
    if (!std::isfinite(display_timeout_) || display_timeout_ <= 0) {
      throw std::invalid_argument("display_timeout must be positive");
    }
    publish_debug_ = declare_parameter<bool>("publish_debug", true, startup_only);
    min_area_ = declare_parameter<int>("min_area", 12, startup_only);
    max_area_ = declare_parameter<int>("max_area", 100000, startup_only);
    const auto reference = declare_parameter<std::vector<int64_t>>(
      "area_reference_size", {0, 0},
      startup_only);
    if (reference.size() != 2 || reference[0] < 0 || reference[1] < 0 ||
      ((reference[0] == 0) != (reference[1] == 0)))
    {
      throw std::invalid_argument("area_reference_size needs two positive sizes or [0, 0]");
    }
    reference_area_ = static_cast<double>(reference[0]) * reference[1];
    const int kernel_size = declare_parameter<int>("morphology_kernel", 3, startup_only);
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
      {170, 100, 50}, {35, 80, 50}, {90, 80, 70}}};
    const std::array<std::vector<int64_t>, 3> upper = {{
      {10, 255, 255}, {85, 255, 255}, {135, 255, 255}}};
    for (size_t i = 0; i < colors_.size(); ++i) {
      auto & color = colors_[i];
      const auto lo = declare_parameter<std::vector<int64_t>>(
        color.name + ".lower", lower[i],
        startup_only);
      const auto hi = declare_parameter<std::vector<int64_t>>(
        color.name + ".upper", upper[i],
        startup_only);
      validate_bounds(color.name, lo, hi);
      color.lower = cv::Scalar(lo[0], lo[1], lo[2]);
      color.upper = cv::Scalar(hi[0], hi[1], hi[2]);
    }

    labels_pub_ = create_publisher<sensor_msgs::msg::Image>(labels_topic, 10);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_topic, 10);
    image_sub_ = create_subscription<Image>(
      image_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](Image::ConstSharedPtr image) {image_callback(image);});
    if (!points_topic.empty() && (publish_debug_ || show_window_)) {
      points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        points_topic, 10,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr points) {
          if (points->header.stamp.sec < 0 || points->header.stamp.nanosec >= 1000000000u) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "Rejected depth annotation timestamp");
            return;
          }
          const auto stamp = rclcpp::Time(points->header.stamp).nanoseconds();
          const auto found = std::find_if(
            pending_.begin(), pending_.end(),
            [stamp](const Frame & frame) {
              return rclcpp::Time(frame.header.stamp).nanoseconds() == stamp;
            });
          if (found != pending_.end()) {
            render(*found, points.get());
            pending_.erase(pending_.begin(), std::next(found));
          }
        });

    }
    RCLCPP_INFO(
      get_logger(), "Listening on %s; labels: 0=background, 1=red, 2=green, 3=blue",
      image_topic.c_str());
  }

  ~ColorDetector() override
  {
    if (show_window_) {
      cv::destroyWindow(window_name_);
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

  struct Region
  {
    cv::Rect box;
    cv::Point center;
    std::string name;
    int number;
    uint8_t label;
    cv::Scalar color;
  };

  struct Frame
  {
    std_msgs::msg::Header header;
    cv::Mat image, regions;
    std::vector<Region> detections;
    std::chrono::steady_clock::time_point received;
  };

  // Results are paired with exactly the RGB frame that produced their labels.
  // A bounded display wait never delays detection or reuses an old depth.
  void render(
    Frame & frame, const sensor_msgs::msg::PointCloud2 * points = nullptr,
    bool stale = false)
  {
    const auto stamp = rclcpp::Time(frame.header.stamp).nanoseconds();
    if (stamp < last_display_stamp_) {return;}
    std::vector<float> depths(frame.detections.size(), std::numeric_limits<float>::quiet_NaN());
    if (points && points->width > 0 && points->height > 0) {
      try {
        const std::array<std::string, 4> names{"u", "v", "depth", "label"};
        std::array<uint32_t, 4> offsets{};
        for (size_t i = 0; i < names.size(); ++i) {
          const auto field = std::find_if(
            points->fields.begin(), points->fields.end(),
            [&](const sensor_msgs::msg::PointField & f) {return f.name == names[i];});
          const uint8_t expected = i == 3 ? sensor_msgs::msg::PointField::UINT8 :
            (i == 2 ? sensor_msgs::msg::PointField::FLOAT32 : sensor_msgs::msg::PointField::UINT32);
          const uint32_t size = i == 3 ? 1 : 4;
          if (field == points->fields.end() || field->datatype != expected || field->count != 1 ||
            field->offset > points->point_step || size > points->point_step - field->offset)
          {
            throw std::runtime_error("Invalid annotation field: " + names[i]);
          }
          offsets[i] = field->offset;
        }
        if (points->point_step == 0 ||
          uint64_t(points->width) * points->point_step > points->row_step ||
          uint64_t(points->row_step) * points->height > points->data.size())
        {
          throw std::runtime_error("Truncated or inconsistent annotation payload");
        }
        const auto read32 = [&](const uint8_t * data) {
            uint32_t value = 0;
            for (unsigned i = 0; i < 4; ++i) {
              value |= uint32_t(data[i]) << (8 * (points->is_bigendian ? 3 - i : i));
            }
            return value;
          };
        for (uint32_t y = 0; y < points->height; ++y) {
          for (uint32_t x = 0; x < points->width; ++x) {
            const auto * data = points->data.data() + size_t(y) * points->row_step +
              size_t(x) * points->point_step;
            const auto u = read32(data + offsets[0]), v = read32(data + offsets[1]);
            const auto bits = read32(data + offsets[2]);
            float depth;
            std::memcpy(&depth, &bits, sizeof(depth));
            if (u >= uint32_t(frame.regions.cols) || v >= uint32_t(frame.regions.rows)) {continue;}
            const int index = frame.regions.at<int>(v, u) - 1;
            if (index >= 0 && index < static_cast<int>(depths.size()) &&
              frame.detections[index].label == data[offsets[3]] && std::isfinite(depth) &&
              depth > 0)
            {
              depths[index] = depth;
            }
          }
        }
      } catch (const std::runtime_error & error) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Rejected depth annotation: %s", error.what());
      }
    }
    if (!stale) {last_frame_ = frame;}
    frame.image = frame.image.clone();
    for (size_t i = 0; i < frame.detections.size(); ++i) {
      const auto & region = frame.detections[i];
      const auto value = std::isfinite(depths[i]) ?
        std::to_string(std::lround(depths[i] * 1000.0)) + " mm" : "-- mm";
      cv::rectangle(frame.image, region.box, region.color, 2);
      cv::circle(frame.image, region.center, 3, region.color, -1);
      cv::putText(
        frame.image, region.name + std::to_string(region.number) + ": " + value,
        {region.box.x, std::max(15, region.box.y - 5)}, cv::FONT_HERSHEY_SIMPLEX,
        0.5, region.color, 1);
    }
    if (stale) {
      cv::putText(
        frame.image, "RGB INPUT STALE", {10, 25}, cv::FONT_HERSHEY_SIMPLEX,
        0.7, {0, 0, 255}, 2);
    }
    if (publish_debug_) {
      debug_pub_->publish(*cv_bridge::CvImage(frame.header, "bgr8", frame.image).toImageMsg());
    }
    if (show_window_) {
      cv::imshow(window_name_, frame.image);
      cv::waitKey(1);
    }
    last_display_stamp_ = stamp;
  }

  void image_callback(const Image::ConstSharedPtr & msg)
  {
    try {
      if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected RGB timestamp");
        return;
      }
      const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
      if (stamp < last_input_stamp_) {
        pending_.clear();
        last_display_stamp_ = -1;
      }
      last_input_stamp_ = stamp;
      const auto input = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      last_rgb_received_ = std::chrono::steady_clock::now();
      input_stale_ = false;
      if (!enable_detection_) {
        Frame frame;
        frame.header = msg->header;
        frame.image = input->image.clone();
        render(frame);
        return;
      }
      cv::Mat hsv;
      cv::cvtColor(input->image, hsv, cv::COLOR_BGR2HSV);
      cv::Mat labels = cv::Mat::zeros(hsv.size(), CV_8UC1);
      const double area_scale = reference_area_ > 0 ? hsv.total() / reference_area_ : 1.0;
      const bool need_debug = publish_debug_ || show_window_; // 仅在需要时生成观看调试图像，避免额外开销。
      Frame frame;
      if (need_debug) {
        frame.header = msg->header;
        frame.image = input->image.clone();
        frame.regions = cv::Mat::zeros(hsv.size(), CV_32SC1);
      }

      for (const auto & color : colors_) {
        cv::Mat mask;
        if (color.lower[0] <= color.upper[0]) {
          cv::inRange(hsv, color.lower, color.upper, mask);
        } else {
          // 红色跨越 H=179/0，合并两个区间。
          cv::Mat high_mask;
          cv::inRange(hsv, cv::Scalar(0, color.lower[1], color.lower[2]), color.upper, mask);
          cv::inRange(
            hsv, color.lower,
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
          if (area < min_area_ * area_scale || area > max_area_ * area_scale) {
            continue;
          }
          const cv::Rect box(
            stats.at<int>(id, cv::CC_STAT_LEFT), stats.at<int>(id, cv::CC_STAT_TOP),
            stats.at<int>(id, cv::CC_STAT_WIDTH), stats.at<int>(id, cv::CC_STAT_HEIGHT));
          const cv::Mat members = components(box) == id;
          labels(box).setTo(color.label, members);
          ++accepted;
          if (need_debug) {
            const cv::Point center(cvRound(centroids.at<double>(id, 0)),
              cvRound(centroids.at<double>(id, 1)));
            frame.detections.push_back(
              {box, center, color.name, accepted,
                color.label, color.display_color});
            frame.regions(box).setTo(static_cast<int>(frame.detections.size()), members);
          }
        }
      }

      // Publish the entire RGB region even when depth is missing.
      labels_pub_->publish(*cv_bridge::CvImage(msg->header, "mono8", labels).toImageMsg());
      if (need_debug) {
        frame.received = std::chrono::steady_clock::now();
        if (points_sub_) {
          if (pending_.size() >= 10) {
            render(pending_.front());
            pending_.pop_front();
          }
          pending_.push_back(std::move(frame));
        } else {
          render(frame);
        }
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
  double reference_area_{0};
  bool enable_detection_, show_window_, publish_debug_{false}, input_stale_{false};
  std::string window_name_;
  double display_timeout_{0.15}, input_timeout_{1.0};
  std::chrono::steady_clock::time_point last_rgb_received_;
  Frame last_frame_;
  int64_t last_display_stamp_{-1}, last_input_stamp_{-1};
  std::deque<Frame> pending_;
  rclcpp::TimerBase::SharedPtr display_timer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
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
