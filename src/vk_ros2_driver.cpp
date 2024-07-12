#include "vk_ros2_driver/vk_ros2_driver.hpp"

vkc::ReceiverStatus vkc::ImuReceiver::handle(const vkc::Message<vkc::Shared<vkc::Imu>> &message) {
    sensor_msgs::msg::Imu imu_ros_message;
    auto imu = message.payload.reader();
    // Linear acceleration
    imu_ros_message.linear_acceleration.x = imu.getLinearAcceleration().getX();
    imu_ros_message.linear_acceleration.y = imu.getLinearAcceleration().getY();
    imu_ros_message.linear_acceleration.z = imu.getLinearAcceleration().getZ();

    // Angular velocity
    imu_ros_message.angular_velocity.x = imu.getAngularVelocity().getX();
    imu_ros_message.angular_velocity.y = imu.getAngularVelocity().getY();
    imu_ros_message.angular_velocity.z = imu.getAngularVelocity().getZ();

    // Orientation

    imu_ros_message.header.stamp = rclcpp::Time(
        imu.getHeader().getStampMonotonic() + imu.getHeader().getClockOffset());
    imu_ros_message.header.frame_id = frame_id_;
    publisher_->publish(imu_ros_message);

    auto body = imu.getExtrinsic().getBodyFrame();
    if (driver_.publish_tf_) {
        geometry_msgs::msg::TransformStamped imu_tf;
        imu_tf.header.stamp = imu_ros_message.header.stamp;
        imu_tf.header.frame_id = driver_.base_link_frame_;
        imu_tf.child_frame_id = frame_id_;
        imu_tf.transform.translation.x = body.getPosition().getX();
        imu_tf.transform.translation.y = body.getPosition().getY();
        imu_tf.transform.translation.z = body.getPosition().getZ();
        imu_tf.transform.rotation.x = body.getOrientation().getX();
        imu_tf.transform.rotation.y = body.getOrientation().getY();
        imu_tf.transform.rotation.z = body.getOrientation().getZ();
        imu_tf.transform.rotation.w = body.getOrientation().getW();
        driver_.static_broadcaster_->sendTransform(imu_tf);
    }

    return vkc::ReceiverStatus::Open;
}

vkc::ReceiverStatus vkc::OdometryReceiver::handle(const vkc::Message<vkc::Shared<vkc::Odometry3d>> &message) {
    nav_msgs::msg::Odometry odometry_ros_message;
    auto odometry = message.payload.reader();
    odometry_ros_message.header.stamp = rclcpp::Time(
        odometry.getHeader().getStampMonotonic() + odometry.getHeader().getClockOffset());
    odometry_ros_message.header.frame_id = driver_.odometry_frame_;
    odometry_ros_message.child_frame_id = driver_.base_link_frame_;
    odometry_ros_message.pose.pose.position.x = odometry.getPose().getPosition().getX();
    odometry_ros_message.pose.pose.position.y = odometry.getPose().getPosition().getY();
    odometry_ros_message.pose.pose.position.z = odometry.getPose().getPosition().getZ();
    odometry_ros_message.pose.pose.orientation.x = odometry.getPose().getOrientation().getX();
    odometry_ros_message.pose.pose.orientation.y = odometry.getPose().getOrientation().getY();
    odometry_ros_message.pose.pose.orientation.z = odometry.getPose().getOrientation().getZ();
    odometry_ros_message.pose.pose.orientation.w = odometry.getPose().getOrientation().getW();
    odometry_ros_message.twist.twist.linear.x = odometry.getTwist().getLinear().getX();
    odometry_ros_message.twist.twist.linear.y = odometry.getTwist().getLinear().getY();
    odometry_ros_message.twist.twist.linear.z = odometry.getTwist().getLinear().getZ();
    odometry_ros_message.twist.twist.angular.x = odometry.getTwist().getAngular().getX();
    odometry_ros_message.twist.twist.angular.y = odometry.getTwist().getAngular().getY();
    odometry_ros_message.twist.twist.angular.z = odometry.getTwist().getAngular().getZ();
    publisher_->publish(odometry_ros_message);
    auto poseCov = odometry.getPoseCovariance();
    for (size_t i = 0; i < poseCov.size(); i++) {
        odometry_ros_message.pose.covariance[i]=poseCov[i];
    }
    auto twistCov = odometry.getTwistCovariance();
    for (size_t i = 0; i < twistCov.size(); i++) {
        odometry_ros_message.twist.covariance[i]=twistCov[i];
    }

    odometry_ros_message.child_frame_id = driver_.base_link_frame_;
    // switch (odometry.getBodyFrame()) {
    //     case vkc::Odometry3d::BodyFrame::NED: {
    //         break;
    //     }
    //     case vkc::Odometry3d::BodyFrame::NWU: {
    //         break;
    //     }
    // }

    if (driver_.publish_tf_) {
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = odometry_ros_message.header.stamp;
        odom_tf.header.frame_id = odometry_ros_message.header.frame_id;
        odom_tf.child_frame_id = odometry_ros_message.child_frame_id;
        odom_tf.transform.translation.x = odometry_ros_message.pose.pose.position.x;
        odom_tf.transform.translation.y = odometry_ros_message.pose.pose.position.y;
        odom_tf.transform.translation.z = odometry_ros_message.pose.pose.position.z;
        odom_tf.transform.rotation = odometry_ros_message.pose.pose.orientation;
        driver_.tf_broadcaster_->sendTransform(odom_tf);
    }

    return vkc::ReceiverStatus::Open;
}

vkc::ReceiverStatus vkc::ImageReceiver::handle(const vkc::Message<vkc::Shared<vkc::Image>> &message) {
    cv::Mat imageMat;
    auto image = message.payload.reader();
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(
        image.getHeader().getStampMonotonic() + image.getHeader().getClockOffset());
    header.frame_id = std::string("camera_") + std::to_string(image.getSensorIdx());
    uint32_t imageHeight = image.getHeight();
    uint32_t imageWidth = image.getWidth();
    long imageSize = image.getData().size();
    auto imageEncoding = image.getEncoding();
    switch (imageEncoding)
    {
    case vkc::Image::Encoding::MONO8:
        imageMat = cv::Mat(imageHeight, imageWidth, CV_8UC1,
                            const_cast<unsigned char *>(image.getData().asBytes().begin()));
        cv::cvtColor(imageMat, imageMat, cv::COLOR_GRAY2RGB);
        break;
    case vkc::Image::Encoding::MONO16:
        imageMat = cv::Mat(imageHeight, imageWidth, CV_16UC1,
                            const_cast<unsigned char *>(image.getData().asBytes().begin()));
        cv::cvtColor(imageMat, imageMat, cv::COLOR_GRAY2RGB);
        break;
    case vkc::Image::Encoding::YUV420:
        imageMat = cv::Mat(imageHeight * 3 / 2, imageWidth, CV_8UC1,
                            const_cast<unsigned char *>(image.getData().asBytes().begin()));
        cv::cvtColor(imageMat, imageMat, cv::COLOR_YUV2BGR_IYUV);
        break;
    case vkc::Image::Encoding::BGR8:
        imageMat = cv::Mat(imageHeight, imageWidth, CV_8UC3,
                            const_cast<unsigned char *>(image.getData().asBytes().begin()));
        imageMat = imageMat.reshape(1, imageHeight);
        cv::cvtColor(imageMat, imageMat, cv::COLOR_BGR2RGB);
        break;
    case vkc::Image::Encoding::JPEG:
        imageMat = cv::imdecode(cv::Mat(1, imageSize, CV_8UC1,
                                        const_cast<unsigned char *>(image.getData().asBytes().begin())),
                               cv::IMREAD_COLOR);
        break;
    default:
        RCLCPP_WARN(driver_.get_logger(), "Unsupported image encoding");
        return vkc::ReceiverStatus::Closed;
    }

    auto body = image.getExtrinsic().getBodyFrame();
    if (driver_.publish_tf_) {
        geometry_msgs::msg::TransformStamped camera_optical_tf;
        camera_optical_tf.header.stamp = header.stamp;
        camera_optical_tf.header.frame_id = driver_.base_link_frame_;
        camera_optical_tf.child_frame_id = header.frame_id + "_optical";
        camera_optical_tf.transform.translation.x = body.getPosition().getX();
        camera_optical_tf.transform.translation.y = body.getPosition().getY();
        camera_optical_tf.transform.translation.z = body.getPosition().getZ();
        camera_optical_tf.transform.rotation.x = body.getOrientation().getX();
        camera_optical_tf.transform.rotation.y = body.getOrientation().getY();
        camera_optical_tf.transform.rotation.z = body.getOrientation().getZ();
        camera_optical_tf.transform.rotation.w = body.getOrientation().getW();
        driver_.static_broadcaster_->sendTransform(camera_optical_tf);

        // optical frame
        geometry_msgs::msg::TransformStamped camera_tf;
        camera_tf.header.stamp = header.stamp;
        camera_tf.header.frame_id = header.frame_id + "_optical";
        camera_tf.child_frame_id = header.frame_id;
        camera_tf.transform.translation.x = 0;
        camera_tf.transform.translation.y = 0;
        camera_tf.transform.translation.z = 0;
        camera_tf.transform.rotation.x = 0.5;
        camera_tf.transform.rotation.y = -0.5;
        camera_tf.transform.rotation.z = 0.5;
        camera_tf.transform.rotation.w = 0.5;
        driver_.static_broadcaster_->sendTransform(camera_tf);
    }

    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", imageMat).toImageMsg();
    publisher_.publish(msg);
    return vkc::ReceiverStatus::Open;
}

vkc::ReceiverStatus vkc::DisparityReceiver::handle(const vkc::Message<vkc::Shared<vkc::Disparity>> &message) {
    auto disparity = message.payload.reader();
    auto disparityEncoding = disparity.getEncoding();
    auto cv_encoding = disparityEncoding == vkc::Disparity::Encoding::DISPARITY8 ? CV_8UC1 : CV_16UC1;
    cv::Mat disparityMat = cv::Mat(
        disparity.getHeight(),
        disparity.getWidth(),
        cv_encoding, const_cast<unsigned char*>(disparity.getData().asBytes().begin()));
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(
        disparity.getHeader().getStampMonotonic() + disparity.getHeader().getClockOffset());
    header.frame_id = disparity.getStreamName();
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "mono16", disparityMat).toImageMsg();

    publisher_.publish(msg);

    return vkc::ReceiverStatus::Open;
}

vkc::ReceiverStatus vkc::PointCloudReceiver::handle(const vkc::Message<vkc::Shared<vkc::PointCloud>> &message) {
    auto point_cloud = message.payload.reader();

    // Retrieve the point cloud data
    unsigned char* pcData = const_cast<unsigned char*>(point_cloud.getPoints().asBytes().begin());
    int pointBytes = static_cast<int>(point_cloud.getPointStride());

    std::vector<int> fieldOffsets;
    std::vector<sensor_msgs::msg::PointField> fields;
    for (const auto& field : point_cloud.getFields())
    {
        auto fieldOffset = static_cast<int>(field.getOffset());
        fieldOffsets.push_back(fieldOffset);
        // std::cout <<field.getName().cStr() << " " << fieldOffset << std::endl;

        sensor_msgs::msg::PointField pointField;
        pointField.name = field.getName().cStr();
        pointField.offset = fieldOffset;
        pointField.count = 1;
        switch (field.getType()) {
            case vkc::Field::NumericType::FLOAT32: {
                pointField.datatype = sensor_msgs::msg::PointField::FLOAT32;
                break;
            }
            case vkc::Field::NumericType::FLOAT64: {
                pointField.datatype = sensor_msgs::msg::PointField::FLOAT64;
                break;
            }
            case vkc::Field::NumericType::UINT8: {
                pointField.datatype = sensor_msgs::msg::PointField::UINT8;
                break;
            }
            case vkc::Field::NumericType::UINT16: {
                pointField.datatype = sensor_msgs::msg::PointField::UINT16;
                break;
            }
            case vkc::Field::NumericType::UINT32: {
                pointField.datatype = sensor_msgs::msg::PointField::UINT32;
                break;
            }
            case vkc::Field::NumericType::UINT64: {
                RCLCPP_WARN(driver_.get_logger(), "UINT64 is not supported in ROS2 PointCloud2");
                return vkc::ReceiverStatus::Closed;
                break;
            }
            case vkc::Field::NumericType::INT8: {
                pointField.datatype = sensor_msgs::msg::PointField::INT8;
                break;
            }
            case vkc::Field::NumericType::INT16: {
                pointField.datatype = sensor_msgs::msg::PointField::INT16;
                break;
            }
            case vkc::Field::NumericType::INT32: {
                pointField.datatype = sensor_msgs::msg::PointField::INT32;
                break;
            }
            case vkc::Field::NumericType::INT64: {
                RCLCPP_WARN(driver_.get_logger(), "INT64 is not supported in ROS2 PointCloud2");
                return vkc::ReceiverStatus::Closed;
                break;
            }
        }
        fields.push_back(pointField);
    }

    auto pointsCount = point_cloud.getPoints().asBytes().size() / pointBytes;

    auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pc2_msg_->fields = fields;
    pc2_msg_->height = 1;
    pc2_msg_->width = pointsCount;
    pc2_msg_->is_bigendian = false;
    pc2_msg_->point_step = pointBytes;
    pc2_msg_->row_step = pointBytes * pointsCount;
    pc2_msg_->is_dense = false;
    pc2_msg_->data.resize(pc2_msg_->row_step);
    memcpy(pc2_msg_->data.data(), pcData, pc2_msg_->row_step);
    pc2_msg_->header.stamp = rclcpp::Time(
        point_cloud.getHeader().getStampMonotonic() + point_cloud.getHeader().getClockOffset());
    pc2_msg_->header.frame_id = driver_.odometry_frame_;
    publisher_->publish(*pc2_msg_);
    return vkc::ReceiverStatus::Open;
}

vkc::VkRos2Driver::VkRos2Driver(const rclcpp::NodeOptions &options)
    : Node("vk_ros2_driver", options),
      node_handle_(std::shared_ptr<VkRos2Driver>(this, [](auto *) {})),
      visualkit(std::move(vkc::VisualKit::create(std::nullopt))),
      it_(node_handle_) {

    this->declare_parameter("pointcloud_topics", rclcpp::PARAMETER_STRING_ARRAY);
    this->declare_parameter("imu_topics", rclcpp::PARAMETER_STRING_ARRAY);
    this->declare_parameter("odometry_topics", rclcpp::PARAMETER_STRING_ARRAY);
    this->declare_parameter("image_topics", rclcpp::PARAMETER_STRING_ARRAY);
    this->declare_parameter("disparity_topics", rclcpp::PARAMETER_STRING_ARRAY);
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<std::string>("odometry_frame", "odom");
    this->declare_parameter<std::string>("base_link_frame", "base_link");
    
    pointcloud_topics_ = this->get_parameter_or<std::vector<std::string>>("pointcloud_topics", {});
    imu_topics_ = this->get_parameter_or<std::vector<std::string>>("imu_topics", {});
    odometry_topics_ = this->get_parameter_or<std::vector<std::string>>("odometry_topics", {});
    image_topics_ = this->get_parameter_or<std::vector<std::string>>("image_topics", {});
    disparity_topics_ = this->get_parameter_or<std::vector<std::string>>("disparity_topics", {});

    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    odometry_frame_ = this->get_parameter("odometry_frame").as_string();
    base_link_frame_ = this->get_parameter("base_link_frame").as_string();

    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    vkc::installLoggingCallback(std::bind(
        &vkc::VkRos2Driver::log_cb, this, std::placeholders::_1, std::placeholders::_2));

    for (const auto& topic : pointcloud_topics_) {
        pointcloud_publishers_[topic] = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic, 10);
        visualkit->source().install(topic, std::make_unique<vkc::PointCloudReceiver>(
            *this, pointcloud_publishers_[topic]));
    }
    for (const auto& topic : imu_topics_) {
        imu_publishers_[topic] = this->create_publisher<sensor_msgs::msg::Imu>(topic, 10);
        visualkit->source().install(topic, std::make_unique<vkc::ImuReceiver>(
            *this, imu_publishers_[topic], topic));
    }
    for (const auto& topic : odometry_topics_) {
        odometry_publishers_[topic] = this->create_publisher<nav_msgs::msg::Odometry>(topic, 10);
        visualkit->source().install(topic, std::make_unique<vkc::OdometryReceiver>(
            *this, odometry_publishers_[topic]));
    }
    for (const auto& topic : image_topics_) {
        image_publishers_[topic] = it_.advertise(topic, 10);
        camera_info_publishers_[topic] = this->create_publisher<sensor_msgs::msg::CameraInfo>(topic + "/camera_info", 10);
        visualkit->source().install(topic, std::make_unique<vkc::ImageReceiver>(
            *this, image_publishers_[topic], camera_info_publishers_[topic]));
    }
    for (const auto& topic : disparity_topics_) {
        disparity_publishers_[topic] = it_.advertise(topic, 10);
        visualkit->source().install(topic, std::make_unique<vkc::DisparityReceiver>(
            *this, disparity_publishers_[topic]));
    }
    visualkit->source().start();
}

vkc::VkRos2Driver::~VkRos2Driver() {
    visualkit->source().stop();
}

void vkc::VkRos2Driver::log_cb(vkc::LogLevel level, std::string_view message) {
    switch (level) {
        case vkc::LogLevel::TRACE: {
            RCLCPP_DEBUG(get_logger(), message.data());
            break;
        };
        case vkc::LogLevel::DEBUG: {
            RCLCPP_DEBUG(get_logger(), message.data());
            break;
        };
        case vkc::LogLevel::INFO: {
            RCLCPP_INFO(get_logger(), message.data());
            break;
        };
        case vkc::LogLevel::WARN: {
            RCLCPP_WARN(get_logger(), message.data());
            break;
        };
        case vkc::LogLevel::ERROR: {
            RCLCPP_ERROR(get_logger(), message.data());
            break;
        };
        default:
            RCLCPP_ERROR(get_logger(), "Invalid log level");
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(vkc::VkRos2Driver)
