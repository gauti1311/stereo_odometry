#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include "visualOdometry.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

const std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("stereo_odometry");
const std::string default_calibration_file = pkg_share_dir + "/config/kitti.yaml";

class StereoOdometryNode : public rclcpp::Node
{
public:
    StereoOdometryNode() : Node("stereo_odometry_node")
    {
        // Parameters for calibration and image topics
        this->declare_parameter<std::string>("calibration_file", default_calibration_file);
        this->declare_parameter<std::string>("left_image_topic", "left_image");
        this->declare_parameter<std::string>("right_image_topic", "right_image");

        // Get parameter values
        std::string calibration_file = this->get_parameter("calibration_file").as_string();
        std::string left_image_topic = this->get_parameter("left_image_topic").as_string();
        std::string right_image_topic = this->get_parameter("right_image_topic").as_string();

        // Load calibration parameters
        if (!loadCalibration(calibration_file, projMatrl_, projMatrr_, intrinsic_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load calibration parameters.");
            rclcpp::shutdown();
        }

        // Set up message filter subscribers for left and right images
        left_image_sub_.subscribe(this, left_image_topic);
        right_image_sub_.subscribe(this, right_image_topic);

        // Synchronize both image topics
        sync_ = std::make_shared<Sync>(SyncPolicy(10), left_image_sub_, right_image_sub_);
        sync_->registerCallback(std::bind(&StereoOdometryNode::imageCallback, this, std::placeholders::_1, std::placeholders::_2));

        // Set up odometry publisher
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_cam", 10);

        RCLCPP_INFO(this->get_logger(), "Stereo Odometry Node started.");
    }

private:
    // Calibration matrices
    cv::Mat projMatrl_, projMatrr_, intrinsic_;

    int frame_id = 0;                                // Initialize frame ID
    cv::Mat frame_pose = cv::Mat::eye(4, 4, CV_64F); // Initialize pose matrix
    cv::Mat rigid_body_transformation;               // To store the rigid body transformation
    double scale = 1.0;                              // Initialize scale (you can adjust this as needed)

    // Image buffers
    cv::Mat imageLeft_t0_, imageRight_t0_, imageLeft_t1_, imageRight_t1_;

    // Feature sets
    FeatureSet currentVOFeatures_;

    // ROS2 message filter subscribers
    message_filters::Subscriber<sensor_msgs::msg::Image> left_image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_image_sub_;

    // Synchronizer
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;

    // ROS2 publisher
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Callback for synchronized left and right images
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg, const sensor_msgs::msg::Image::ConstSharedPtr &right_msg)
    {
        cv_bridge::CvImagePtr left_cv_ptr, right_cv_ptr;
        try
        {
            left_cv_ptr = cv_bridge::toCvCopy(left_msg, sensor_msgs::image_encodings::BGR8);
            right_cv_ptr = cv_bridge::toCvCopy(right_msg, sensor_msgs::image_encodings::BGR8);

            imageLeft_t1_ = left_cv_ptr->image;
            imageRight_t1_ = right_cv_ptr->image;

            if (imageLeft_t0_.empty() || imageRight_t0_.empty())
            {
                // Initialize the previous images
                imageLeft_t0_ = imageLeft_t1_.clone();
                imageRight_t0_ = imageRight_t1_.clone();
                return; // Skip further processing until we have enough images
            }
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        processStereoImages(); // Call the processing function once both images are synchronized
    }

    // Stereo image processing and odometry calculation
    void processStereoImages()
    {
        if (imageLeft_t1_.empty() || imageRight_t1_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Left or right image is empty!");
            return; // Skip further processing
        }

        // Perform feature matching
        std::vector<cv::Point2f> pointsLeft_t0, pointsRight_t0, pointsLeft_t1, pointsRight_t1;
        // RCLCPP_INFO(this->get_logger(), "Error: Images arrives SS!");

        matchingFeatures(imageLeft_t0_, imageRight_t0_, imageLeft_t1_, imageRight_t1_, currentVOFeatures_,
                         pointsLeft_t0, pointsRight_t0, pointsLeft_t1, pointsRight_t1);

        // Triangulate points to get 3D
        cv::Mat points3D_t0, points4D_t0;
        cv::triangulatePoints(projMatrl_, projMatrr_, pointsLeft_t0, pointsRight_t0, points4D_t0);
        cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);

        // Track between frames to compute transformation (rotation, translation)
        cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat translation = cv::Mat::zeros(3, 1, CV_64F);
        trackingFrame2Frame(projMatrl_, projMatrr_, pointsLeft_t0, pointsLeft_t1, points3D_t0, rotation, translation, false);

        integrateOdometryStereo(frame_id, rigid_body_transformation, frame_pose, rotation, translation, scale);

        // Extract updated rotation and translation for odometry publishing
        cv::Mat updated_rotation = frame_pose(cv::Rect(0, 0, 3, 3));    // Extract rotation
        cv::Mat updated_translation = frame_pose(cv::Rect(3, 0, 1, 3)); // Extract translation

        // Publish odometry
        publishOdometry(updated_rotation, updated_translation);

        // Update old images for next iteration
        imageLeft_t0_ = imageLeft_t1_.clone();
        imageRight_t0_ = imageRight_t1_.clone();
    }

    // Function to publish odometry message
    void publishOdometry(const cv::Mat &rotation, const cv::Mat &translation)
    {

        // Transformation matrix from camera frame to base link frame
        cv::Mat T_camera_to_base = (cv::Mat_<double>(4, 4) << 0, 0, 1, 0, // X_camera -> Z_base
                                    -1, 0, 0, 0,                          // Y_camera -> -X_base
                                    0, -1, 0, 0,                          // Z_camera -> -Y_base
                                    0, 0, 0, 1);                          // Homogeneous transformation

        // Build the current transformation matrix from rotation and translation
        cv::Mat current_pose = cv::Mat::eye(4, 4, CV_64F);
        rotation.copyTo(current_pose(cv::Rect(0, 0, 3, 3)));   
        translation.copyTo(current_pose(cv::Rect(3, 0, 1, 3)));

        // Apply the camera to base transformation
        cv::Mat base_pose = T_camera_to_base * current_pose;

        // Extract updated translation and rotation in base_link frame
        cv::Mat updated_translation = base_pose(cv::Rect(3, 0, 1, 3));
        cv::Mat updated_rotation = base_pose(cv::Rect(0, 0, 3, 3));   

        // Prepare odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom_cam";
        odom_msg.child_frame_id = "base_link";

        // Set position from transformed translation
        odom_msg.pose.pose.position.x = updated_translation.at<double>(0);
        odom_msg.pose.pose.position.y = updated_translation.at<double>(1);
        odom_msg.pose.pose.position.z = updated_translation.at<double>(2);

        // Convert rotation matrix to quaternion
        tf2::Matrix3x3 tf_rot(
            updated_rotation.at<double>(0, 0), updated_rotation.at<double>(0, 1), updated_rotation.at<double>(0, 2),
            updated_rotation.at<double>(1, 0), updated_rotation.at<double>(1, 1), updated_rotation.at<double>(1, 2),
            updated_rotation.at<double>(2, 0), updated_rotation.at<double>(2, 1), updated_rotation.at<double>(2, 2));

        tf2::Quaternion q;
        tf_rot.getRotation(q);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);

        // Publish odometry
        odom_pub_->publish(odom_msg);
    }

    // Calibration loading function
    bool loadCalibration(const std::string &calibration_file, cv::Mat &projMatrl, cv::Mat &projMatrr, cv::Mat &intrinsic)
    {
        // Load camera intrinsics
        try
        {
            YAML::Node params = YAML::LoadFile(calibration_file);

            double fx = params["fx"].as<double>();
            double fy = params["fy"].as<double>();
            double cx = params["cx"].as<double>();
            double cy = params["cy"].as<double>();
            double bf = params["bf"].as<double>();

            // Set projection matrices
            projMatrl = (cv::Mat_<float>(3, 4) << fx, 0., cx, 0., 0., fy, cy, 0., 0, 0., 1., 0.);
            projMatrr = (cv::Mat_<float>(3, 4) << fx, 0., cx, bf, 0., fy, cy, 0., 0, 0., 1., 0.);

            // Set intrinsic matrix
            intrinsic = (cv::Mat_<double>(3, 3) << fx, 0., cx, 0., fy, cy, 0, 0., 1.);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load parameter file: %s", e.what());
            return false;
        }

        return true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
