/**
 * 存放处理后的IMU姿态以及GNSS位置
*/
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_

#include <Eigen/Dense>

namespace lidar_localization {
class PoseData {
    public:
        double time = 0.0;
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    private:
        Eigen::Quaternionf GetQuaternion();
};
}

#endif