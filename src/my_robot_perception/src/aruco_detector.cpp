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





class ArucoDetector : public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        cv::Ptr<cv::aruco::Dictionary> dictionary_;
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;
        double marker_size_;
        bool show_window_;
        int binary_threshold_;



    public:
        ArucoDetector(): Node("aruco_detector")
        {
            // The 0.2 m board has a 50 px quiet zone around a 500 px marker.
            marker_size_ = this->declare_parameter<double>("marker_size", 0.1666666667);
            show_window_ = this->declare_parameter<bool>("show_window", true);
            binary_threshold_ = this->declare_parameter<int>("binary_threshold", 160);

            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            subscription_ =
                this->create_subscription<sensor_msgs::msg::Image>("/camera_sensor/image_raw",10,std::bind(
                        &ArucoDetector::image_callback,
                        this,
                        std::placeholders::_1
                    )
                );
            camera_info_subscription_ =
                this->create_subscription<sensor_msgs::msg::CameraInfo>(
                    "/camera_sensor/camera_info", 10,
                    std::bind(&ArucoDetector::camera_info_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),"Camera node started" );
            dictionary_ =cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
            camera_matrix_ =(cv::Mat_<double>(3,3)<<400.2,0,400,
                                                    0,400.2,300,
                                                    0,0,1);
            dist_coeffs_ =cv::Mat::zeros(1,5,CV_64F);
            if (show_window_)
            {
                cv::namedWindow("Gazebo Camera",cv::WINDOW_NORMAL);
            }
        }

        void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
        {
            camera_matrix_ = (cv::Mat_<double>(3, 3) <<
                msg->k[0], msg->k[1], msg->k[2],
                msg->k[3], msg->k[4], msg->k[5],
                msg->k[6], msg->k[7], msg->k[8]);
            dist_coeffs_ = cv::Mat::zeros(1, std::max<std::size_t>(5, msg->d.size()), CV_64F);
            for (std::size_t i = 0; i < msg->d.size(); ++i)
            {
                dist_coeffs_.at<double>(0, i) = msg->d[i];
            }
        }

        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
        {
            cv::Mat image;
            try
            {
                image =cv_bridge::toCvCopy(msg,"rgb8")->image;

            }
            catch(cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "cv_bridge error"
                );
                return;
            }

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;

            cv::aruco::detectMarkers(image,dictionary_,corners,ids);

            // Gazebo's large, uniform surfaces can confuse OpenCV 4.5's
            // adaptive threshold. Keep the normal detector for real images,
            // then use a deterministic binary fallback for simulation.
            if (ids.empty())
            {
                cv::Mat gray;
                cv::Mat binary;
                cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
                cv::threshold(
                    gray, binary, binary_threshold_, 255, cv::THRESH_BINARY);
                cv::aruco::detectMarkers(binary, dictionary_, corners, ids);
            }



            if(!ids.empty())
            {
                // 1. 绘制marker边框
                cv::aruco::drawDetectedMarkers(
                    image,
                    corners,
                    ids
                );


                // 2. 计算姿态
                std::vector<cv::Vec3d> rvecs;
                std::vector<cv::Vec3d> tvecs;

                cv::aruco::estimatePoseSingleMarkers(
                    corners,
                    marker_size_,
                    camera_matrix_,
                    dist_coeffs_,
                    rvecs,
                    tvecs
                );

                for(size_t i=0;i<ids.size();i++)
                {
                    // 绘制坐标轴
                    cv::aruco::drawAxis(
                        image,
                        camera_matrix_,
                        dist_coeffs_,
                        rvecs[i],
                        tvecs[i],
                        0.1
                    );
                    // 显示文字
                    std::string text ="ID:"+std::to_string(ids[i]);

                    text +=" X:"+std::to_string(tvecs[i][0]);

                    text +=" Y:"+std::to_string(tvecs[i][1]);

                    text +=" Z:"+std::to_string(tvecs[i][2]);

                    cv::putText(
                        image,text,
                        cv::Point(20,50+10*i),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.8,
                        cv::Scalar(0,255,0),
                        2
                    );

                    publish_tf(rvecs[i], tvecs[i], ids[i], msg->header);
                }
            }
            else
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "No ArUco marker detected in /camera_sensor/image_raw");
            }

            if (show_window_)
            {
                cv::imshow("Gazebo Camera",image);
                cv::waitKey(1);
            }
        }

        void publish_tf(
            const cv::Vec3d & rvec,
            const cv::Vec3d & tvec,
            int id,
            const std_msgs::msg::Header & image_header)
        {

            cv::Mat R;
            cv::Rodrigues(rvec,R);

            geometry_msgs::msg::TransformStamped tf;

            tf.header.stamp = image_header.stamp;

            // OpenCV returns the pose in the image's optical coordinate frame.
            tf.header.frame_id = image_header.frame_id.empty()
                ? "camera_optical_link"
                : image_header.frame_id;

            tf.child_frame_id ="aruco_marker_" + std::to_string(id);

            tf.transform.translation.x =tvec[0];

            tf.transform.translation.y =tvec[1];

            tf.transform.translation.z =tvec[2];

            
            // rotation matrix -> quaternion
            tf2::Matrix3x3 m(
                R.at<double>(0,0),
                R.at<double>(0,1),
                R.at<double>(0,2),
                R.at<double>(1,0),
                R.at<double>(1,1),
                R.at<double>(1,2),
                R.at<double>(2,0),
                R.at<double>(2,1),
                R.at<double>(2,2)
            );

            tf2::Quaternion q;

            m.getRotation(q);
            q.normalize();

            tf.transform.rotation.x=q.x();
            tf.transform.rotation.y=q.y();
            tf.transform.rotation.z=q.z();
            tf.transform.rotation.w=q.w();

            tf_broadcaster_->sendTransform(tf);
        }
};



int main(int argc,char **argv)
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
