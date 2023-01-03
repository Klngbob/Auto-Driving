/**
 * 单个key frame信息发布
*/
#ifndef LIDAR_LOCALIZATION_PUBLISHER_KEY_FRAME_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_KEY_FRAME_PUBLISHER_HPP_

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "lidar_localization/sensor_data/key_frame_data.hpp"

namespace lidar_localization {
class KeyFramePublisher {
    public:
        KeyFramePublisher(ros::NodeHandle& nh,
                          std::string topic_name,
                          std::string frame_id,
                          size_t buff_size);
        KeyFramePublisher() = default;

        void Publish(const KeyFrameData& key_frame);

        bool HasSubscribers();

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_ = "";
};
}
#endif