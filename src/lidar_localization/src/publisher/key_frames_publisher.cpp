#include "lidar_localization/publisher/key_frames_publisher.hpp"

#include <Eigen/Dense>

namespace lidar_localization {
KeyFramesPublisher::KeyFramesPublisher(ros::NodeHandle& nh,
                                     std::string topic_name,
                                     std::string frame_id,
                                     size_t buff_size)
    : nh_(nh), frame_id_(frame_id) {
        this->publisher_ = nh_.advertise<nav_msgs::Path>(topic_name, buff_size);
    }

void KeyFramesPublisher::Publish(const std::deque<KeyFrameData>& key_frames) {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = this->frame_id_;

    for(size_t i = 0; i < key_frames.size(); ++i) {
        geometry_msgs::PoseStamped pose_stamped;
        KeyFrameData key_frame = key_frames.at(i);
        ros::Time ros_time((float) key_frame.time);
        pose_stamped.header.stamp = ros_time;
        pose_stamped.header.frame_id = this->frame_id_;
        pose_stamped.header.seq = key_frame.index;
        pose_stamped.pose.position.x = key_frame.pose(0, 3);
        pose_stamped.pose.position.y = key_frame.pose(1, 3);
        pose_stamped.pose.position.z = key_frame.pose(2, 3);

        Eigen::Quaternionf q;
        q = key_frame.pose.block<3, 3>(0, 0);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        path.poses.push_back(pose_stamped);
    }
    
    this->publisher_.publish(path);
}

bool KeyFramesPublisher::HasSubscribers() {
    return this->publisher_.getNumSubscribers() != 0;
}
}