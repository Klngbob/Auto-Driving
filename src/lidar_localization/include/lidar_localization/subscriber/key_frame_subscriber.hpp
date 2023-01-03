/**
 * 订阅key frame数据
*/
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "lidar_localization/sensor_data/key_frame_data.hpp"

namespace lidar_localization {
class KeyFrameSubscriber {
    public:
        KeyFrameSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        KeyFrameSubscriber() = default;
        void ParseData(std::deque<KeyFrameData>& key_frame_buff);

    private:
        void msg_callback(const geometry_msgs::PoseStampedConstPtr& pose_msg_ptr);
    
    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;

        std::deque<KeyFrameData> new_key_frame_;
};
}

#endif