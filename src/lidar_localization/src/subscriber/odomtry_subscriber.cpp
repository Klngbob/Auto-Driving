#include "lidar_localization/subscriber/odometry_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization {
OdometrySubscriber::OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
        this->subscriber_ = this->nh_.subscribe(topic_name, buff_size, &OdometrySubscriber::msg_callback, this);
    }

void OdometrySubscriber::msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr) {
    PoseData pose_data;
    pose_data.time = odom_msg_ptr->header.stamp.toSec();
    
    pose_data.pose(0, 3) = odom_msg_ptr->pose.pose.position.x;
    pose_data.pose(1, 3) = odom_msg_ptr->pose.pose.position.y;
    pose_data.pose(2, 3) = odom_msg_ptr->pose.pose.position.z;

    Eigen::Quaternionf q;
    q.x() = odom_msg_ptr->pose.pose.orientation.x;
    q.y() = odom_msg_ptr->pose.pose.orientation.y;
    q.z() = odom_msg_ptr->pose.pose.orientation.z;
    q.w() = odom_msg_ptr->pose.pose.orientation.w;
    pose_data.pose.block<3, 3>(0, 0) = q.matrix();

    this->new_pose_data_.push_back(pose_data);
}

void OdometrySubscriber::ParseData(std::deque<PoseData>& deque_pose_data) {
    if(this->new_pose_data_.size() > 0) {
        deque_pose_data.insert(deque_pose_data.end(), this->new_pose_data_.begin(), this->new_pose_data_.end());
        this->new_pose_data_.clear();
    }
}
}