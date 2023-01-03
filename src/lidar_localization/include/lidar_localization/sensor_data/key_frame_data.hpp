/**
 * 关键帧，在各个模块之间传递数据
*/
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_KEYFRAME_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_KEYFRAME_DATA_HPP_

#include <Eigen/Dense>

namespace lidar_localization {
class KeyFrameData {
    public:
        double time = 0.0;
        unsigned int index = 0;
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        
    private:
        Eigen::Quaternionf GetQuaternion();
};
}

#endif