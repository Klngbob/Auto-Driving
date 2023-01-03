#include "lidar_localization/subscriber/key_frames_subscriber.hpp"
#include "glog/logging.h"

namespace lidar_localization {
KeyFramesSubscriber::KeyFramesSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
        this->subscriber_ = this->nh_.subscribe(topic_name, buff_size, &KeyFramesSubscriber::msg_callback, this);
}

void KeyFramesSubscriber::msg_callback(const nav_msgs::PathConstPtr& key_frames_msg_ptr) {
    this->new_key_frames_.clear();
    for(size_t i = 0; i < key_frames_msg_ptr->poses.size(); ++i) {
        KeyFrameData key_frame;
        key_frame.time = key_frames_msg_ptr->poses[i].header.stamp.toSec();
        key_frame.index = key_frames_msg_ptr->poses[i].header.seq;

        key_frame.pose(0, 3) = key_frames_msg_ptr->poses[i].pose.position.x;
        key_frame.pose(1, 3) = key_frames_msg_ptr->poses[i].pose.position.y;
        key_frame.pose(2, 3) = key_frames_msg_ptr->poses[i].pose.position.z;

        Eigen::Quaternionf q;
        q.x() = key_frames_msg_ptr->poses[i].pose.orientation.x;
        q.y() = key_frames_msg_ptr->poses[i].pose.orientation.y;
        q.z() = key_frames_msg_ptr->poses[i].pose.orientation.z;
        q.w() = key_frames_msg_ptr->poses[i].pose.orientation.w;
        key_frame.pose.block<3, 3>(0, 0) = q.matrix();

        this->new_key_frames_.push_back(key_frame);
    }
}

void KeyFramesSubscriber::ParseData(std::deque<KeyFrameData>& key_frame_buff) {
    if(this->new_key_frames_.size() > 0) {
        key_frame_buff = this->new_key_frames_;
        this->new_key_frames_.clear();
    }
}

}