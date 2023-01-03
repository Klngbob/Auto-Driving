#include "lidar_localization/subscriber/key_frame_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization {
KeyFrameSubscriber::KeyFrameSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
        this->subscriber_ = this->nh_.subscribe(topic_name, buff_size, &KeyFrameSubscriber::msg_callback, this);
    }

void KeyFrameSubscriber::msg_callback(const geometry_msgs::PoseStampedConstPtr& pose_msg_ptr) {
    KeyFrameData key_frame;
    key_frame.time = pose_msg_ptr->header.stamp.toSec();
    key_frame.index = pose_msg_ptr->header.seq;

    key_frame.pose(0, 3) = pose_msg_ptr->pose.position.x;
    key_frame.pose(1, 3) = pose_msg_ptr->pose.position.y;
    key_frame.pose(2, 3) = pose_msg_ptr->pose.position.z;

    Eigen::Quaternionf q;
    q.x() = pose_msg_ptr->pose.orientation.x;
    q.y() = pose_msg_ptr->pose.orientation.y;
    q.z() = pose_msg_ptr->pose.orientation.z;
    q.w() = pose_msg_ptr->pose.orientation.w;
    key_frame.pose.block<3, 3>(0, 0) = q.matrix();
    
    this->new_key_frame_.push_back(key_frame);
}

void KeyFrameSubscriber::ParseData(std::deque<KeyFrameData>& key_frame_buff) {
    if(this->new_key_frame_.size() > 0) {
        key_frame_buff.insert(key_frame_buff.end(), this->new_key_frame_.begin(), this->new_key_frame_.end());
        this->new_key_frame_.clear();
    }
}
}