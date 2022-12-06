/*
* 在ros中发布里程计信息
*/

#include "lidar_localization/publisher/odometry_publisher.hpp"

namespace lidar_localization {
OdometryPublisher::OdometryPublisher(ros::NodeHandle& nh,
                          std::string topic_name,
                          std::string base_frame_id,
                          std::string child_frame_id,
                          int buff_size)
    : nh_(nh) {
        this->publisher_ = nh_.advertise<nav_msgs::Odometry>(topic_name, buff_size);
        this->odometry_.header.frame_id = base_frame_id;
        this->odometry_.child_frame_id = child_frame_id;
    }

void OdometryPublisher::Publish(const Eigen::Matrix4f& transform_matrix) {
    this->odometry_.header.stamp = ros::Time::now();

    // 设置position
    this->odometry_.pose.pose.position.x = transform_matrix(0, 3);
    this->odometry_.pose.pose.position.y = transform_matrix(1, 3);
    this->odometry_.pose.pose.position.z = transform_matrix(2, 3);

    Eigen::Quaternionf q;
    q = transform_matrix.block<3, 3>(0, 0);
    this->odometry_.pose.pose.orientation.x = q.x();
    this->odometry_.pose.pose.orientation.y = q.y();
    this->odometry_.pose.pose.orientation.z = q.z();
    this->odometry_.pose.pose.orientation.w = q.w();
    
    this->publisher_.publish(this->odometry_);
}
}
