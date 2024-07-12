#ifndef VK_ROS2_DRIVER_HPP_
#define VK_ROS2_DRIVER_HPP_

#include <vk_sdk/Sdk.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Core>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace vkc
{
    class ImuReceiver;
    class OdometryReceiver;
    class PointCloudReceiver;
    class DisparityReceiver;
    class ImageReceiver;

    class VkRos2Driver : public rclcpp::Node
    {
    public:
        VkRos2Driver(const rclcpp::NodeOptions &options);
        ~VkRos2Driver();

        friend class ImuReceiver;
        friend class OdometryReceiver;
        friend class PointCloudReceiver;
        friend class DisparityReceiver;
        friend class ImageReceiver;
    protected:
        rclcpp::Node::SharedPtr node_handle_;
    private:
        void log_cb(vkc::LogLevel level, std::string_view message);

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_inflated_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_ror_publisher_;
        image_transport::Publisher image_publisher_;
        image_transport::Publisher left_disparity_publisher_;
        image_transport::Publisher right_disparity_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<rclcpp::TimerBase> publish_timer_;
        std::vector<std::string> pointcloud_topics_;
        std::vector<std::string> imu_topics_;
        std::vector<std::string> odometry_topics_;
        std::vector<std::string> image_topics_;
        std::vector<std::string> disparity_topics_;
        bool publish_tf_;
        std::string odometry_frame_, base_link_frame_;
        std::unique_ptr<vkc::VisualKit> visualkit;
        image_transport::ImageTransport it_;

        std::map<std::string, image_transport::Publisher> image_publishers_;
        std::map<std::string, image_transport::Publisher> disparity_publishers_;
        std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> imu_publishers_;
        std::map<std::string, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odometry_publishers_;
        std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pointcloud_publishers_;
        std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_publishers_;
    };

    class ImageReceiver : public vkc::Receiver<vkc::Image>
    {
    public:
        inline ImageReceiver(const VkRos2Driver& driver,
                             image_transport::Publisher& publisher,
                             rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_publisher)
            : Receiver(), driver_(driver), publisher_(publisher), cam_info_publisher_(cam_info_publisher) {}

        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Image>> &message);

    private:
        const VkRos2Driver& driver_;
        image_transport::Publisher& publisher_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_publisher_;
    };

    class DisparityReceiver : public vkc::Receiver<vkc::Disparity>
    {
    public:
        inline DisparityReceiver(const VkRos2Driver& driver,
                                 image_transport::Publisher& publisher)
            : Receiver(), driver_(driver), publisher_(publisher) {}

        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Disparity>> &message);
    private:
        const VkRos2Driver& driver_;
        image_transport::Publisher& publisher_;
    };

    class ImuReceiver : public vkc::Receiver<vkc::Imu>
    {
    public:
        inline ImuReceiver(const VkRos2Driver& driver,
                           rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher,
                           std::string frame_id = "imu_link")
            : Receiver(), driver_(driver), frame_id_(frame_id)
        {
            publisher_ = publisher;
        }

        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Imu>> &message);
    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
        const VkRos2Driver& driver_;
        std::string frame_id_;
    };
    

    class OdometryReceiver : public vkc::Receiver<vkc::Odometry3d>
    {
    public:
        inline OdometryReceiver(const VkRos2Driver& driver,
                                rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher)
            : Receiver(), driver_(driver), publisher_(publisher) {}

        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::Odometry3d>> &message);
    private:
        const VkRos2Driver& driver_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    };

    class PointCloudReceiver : public vkc::Receiver<vkc::PointCloud>
    {
    public:
        inline PointCloudReceiver(const VkRos2Driver& driver,
                                  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher)
            : Receiver(), driver_(driver), publisher_(publisher) {}

        vkc::ReceiverStatus handle(const vkc::Message<vkc::Shared<vkc::PointCloud>> &message);
    private:
        const VkRos2Driver& driver_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    };

} // namespace vkc

#endif // VK_ROS2_DRIVER_HPP_
