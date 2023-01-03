/**
 * key frames信息发布
*/
#ifndef LIDAR_LOCALIZATION_PUBLISHER_KEY_FRAMES_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_KEY_FRAMES_PUBLISHER_HPP_

#include <string>
#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include "lidar_localization/sensor_data/key_frame_data.hpp"

namespace lidar_localization {
class KeyFramesPublisher {
    public:
        KeyFramesPublisher(ros::NodeHandle& nh,
                          std::string topic_name,
                          std::string frame_id,
                          size_t buff_size);
        KeyFramesPublisher() = default;

        void Publish(const std::deque<KeyFrameData>& key_frames);

        bool HasSubscribers();

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_ = "";
};
}
#endif