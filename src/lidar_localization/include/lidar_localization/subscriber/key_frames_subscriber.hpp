/**
 * 订阅key frames数据
*/
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_KEY_FRAMES_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_KEY_FRAMES_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "lidar_localization/sensor_data/key_frame_data.hpp"

namespace lidar_localization {
class KeyFramesSubscriber {
    public:
        KeyFramesSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        KeyFramesSubscriber() = default;
        void ParseData(std::deque<KeyFrameData>& key_frame_buff);

    private:
        void msg_callback(const nav_msgs::PathConstPtr& key_frames_msg_ptr);
    
    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<KeyFrameData> new_key_frames_;
};
}

#endif