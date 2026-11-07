#include "my_robot_perception/camera_info_validation.hpp"
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <opencv2/aruco.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Matrix3x3.h>

#include <tf2/LinearMath/Quaternion.h>
#include <map>
#include <stdexcept>
#include <string>

class ArucoDetector : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  double marker_size_;
  bool show_window_;
  int binary_threshold_;
  std::string dictionary_name_;

public:
  ArucoDetector()
  : Node("aruco_detector")
  {
    rcl_interfaces::msg::ParameterDescriptor startup_only;
    startup_only.read_only = true;
    startup_only.description = "Startup configuration; edit YAML and restart to change";

    // Side length of one black marker, including its border, in metres.
    marker_size_ = this->declare_parameter<double>("marker_size", 0.03, startup_only);
    show_window_ = this->declare_parameter<bool>("show_window", false, startup_only);
    binary_threshold_ = this->declare_parameter<int>("binary_threshold", 160, startup_only);
    dictionary_name_ = declare_parameter<std::string>("dictionary", "DICT_4X4_50", startup_only);
    const std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dictionaries{
      {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
      {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
      {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
      {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
      {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
      {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
      {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
      {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
      {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
      {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
      {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
      {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
      {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
      {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
      {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
      {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
      {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
      {"DICT_APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5},
      {"DICT_APRILTAG_25h9", cv::aruco::DICT_APRILTAG_25h9},
      {"DICT_APRILTAG_36h10", cv::aruco::DICT_APRILTAG_36h10},
      {"DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11}};
    const auto selected = dictionaries.find(dictionary_name_);
    if (selected == dictionaries.end()) {
      throw std::invalid_argument("Unsupported ArUco dictionary: " + dictionary_name_);
    }
    dictionary_ = cv::aruco::getPredefinedDictionary(selected->second);

    if (!std::isfinite(marker_size_) || marker_size_ <= 0 ||
      binary_threshold_ < 0 || binary_threshold_ > 255)
    {
      throw std::invalid_argument("Invalid marker size or binary threshold");
    }
    const auto image_topic = declare_parameter<std::string>(
      "image_topic", "/camera/color/origin", startup_only);
    const auto info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/camera/info/color", startup_only);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    subscription_ =
      this->create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(), std::bind(
        &ArucoDetector::image_callback,
        this,
        std::placeholders::_1
      )
      );
    camera_info_subscription_ =
      this->create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, rclcpp::SensorDataQoS(),
      std::bind(&ArucoDetector::camera_info_callback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "ArUco dictionary=%s; marker_size=%.6f m",
      dictionary_name_.c_str(), marker_size_);
    if (show_window_) {
      cv::namedWindow("ArUco", cv::WINDOW_NORMAL);
    }
  }

  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    const auto error = my_robot_perception::camera_info_error(*msg);
    if (!error.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejected CameraInfo, retaining last valid calibration: %s", error.c_str());
      return;
    }
    camera_info_ = msg;
    camera_matrix_ = cv::Mat(3, 3, CV_64F, msg->k.data()).clone();
    dist_coeffs_ = cv::Mat(msg->d, true).reshape(1, 1);
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!camera_info_ || msg->header.frame_id != camera_info_->header.frame_id ||
      msg->width != camera_info_->width || msg->height != camera_info_->height)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for valid CameraInfo matching image frame and dimensions");
      return;
    }
    cv::Mat image;
    try {
      image = cv_bridge::toCvCopy(msg, "bgr8")->image;

    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "cv_bridge error: %s", e.what()
      );
      return;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    cv::aruco::detectMarkers(image, dictionary_, corners, ids);

    // Gazebo's large, uniform surfaces can confuse OpenCV 4.5's
    // adaptive threshold. Keep the normal detector for real images,
    // then use a deterministic binary fallback for simulation.
    if (ids.empty()) {
      cv::Mat gray;
      cv::Mat binary;
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
      cv::threshold(
        gray, binary, binary_threshold_, 255, cv::THRESH_BINARY);
      cv::aruco::detectMarkers(binary, dictionary_, corners, ids);
    }

    if (!ids.empty()) {
      // 1. 绘制marker边框
      cv::aruco::drawDetectedMarkers(
        image,
        corners,
        ids
      );

      // 2. 计算姿态
      std::vector<cv::Vec3d> rvecs;
      std::vector<cv::Vec3d> tvecs;

      auto pose_corners = corners;
      cv::Mat pose_distortion = dist_coeffs_;
      const bool fisheye = camera_info_->distortion_model == "equidistant";
      if (fisheye) {
        for (auto & marker : pose_corners) {
          std::vector<cv::Point2f> corrected;
          cv::fisheye::undistortPoints(
            marker, corrected, camera_matrix_,
            dist_coeffs_, cv::noArray(), camera_matrix_);
          marker = std::move(corrected);
        }
        pose_distortion = cv::Mat();
      }
      cv::aruco::estimatePoseSingleMarkers(
        pose_corners,
        marker_size_,
        camera_matrix_,
        pose_distortion,
        rvecs,
        tvecs
      );

      for (size_t i = 0; i < ids.size(); i++) {
        // Project overlay axes with the original image model.
        const float axis_length = static_cast<float>(marker_size_ * 0.5);
        if (fisheye) {
          const std::vector<cv::Point3f> axes{
            {0, 0, 0}, {axis_length, 0, 0},
            {0, axis_length, 0}, {0, 0, axis_length}};
          std::vector<cv::Point2f> projected;
          cv::fisheye::projectPoints(
            axes, projected, rvecs[i], tvecs[i],
            camera_matrix_, dist_coeffs_);
          cv::line(image, projected[0], projected[1], {0, 0, 255}, 2);
          cv::line(image, projected[0], projected[2], {0, 255, 0}, 2);
          cv::line(image, projected[0], projected[3], {255, 0, 0}, 2);
        } else {
          cv::aruco::drawAxis(
            image, camera_matrix_, dist_coeffs_,
            rvecs[i], tvecs[i], axis_length);
        }
        // 显示文字
        std::string text = "ID:" + std::to_string(ids[i]);

        text += " X:" + std::to_string(tvecs[i][0]);

        text += " Y:" + std::to_string(tvecs[i][1]);

        text += " Z:" + std::to_string(tvecs[i][2]);

        cv::putText(
          image, text,
          cv::Point(20, 50 + 10 * i),
          cv::FONT_HERSHEY_SIMPLEX,
          0.8,
          cv::Scalar(0, 255, 0),
          2
        );

        publish_tf(rvecs[i], tvecs[i], ids[i], msg->header);
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "No ArUco marker detected (dictionary=%s)", dictionary_name_.c_str());
    }

    if (show_window_) {
      cv::imshow("ArUco", image);
      cv::waitKey(1);
    }
  }

  void publish_tf(
    const cv::Vec3d & rvec,
    const cv::Vec3d & tvec,
    int id,
    const std_msgs::msg::Header & image_header)
  {

    for (int axis = 0; axis < 3; ++axis) {
      if (!std::isfinite(rvec[axis]) || !std::isfinite(tvec[axis])) {return;}
    }
    if (tvec[2] <= 0) {return;}
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = image_header.stamp;

    // OpenCV returns the pose in the image's optical coordinate frame.
    tf.header.frame_id = image_header.frame_id;

    tf.child_frame_id = "aruco_marker_" + std::to_string(id);

    tf.transform.translation.x = tvec[0];

    tf.transform.translation.y = tvec[1];

    tf.transform.translation.z = tvec[2];

    // rotation matrix -> quaternion
    tf2::Matrix3x3 m(
      R.at<double>(0, 0),
      R.at<double>(0, 1),
      R.at<double>(0, 2),
      R.at<double>(1, 0),
      R.at<double>(1, 1),
      R.at<double>(1, 2),
      R.at<double>(2, 0),
      R.at<double>(2, 1),
      R.at<double>(2, 2)
    );

    tf2::Quaternion q;

    m.getRotation(q);
    q.normalize();

    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf);
  }
};

int main(int argc, char ** argv)
{

  rclcpp::init(
    argc,
    argv
  );

  auto node =
    std::make_shared<ArucoDetector>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
