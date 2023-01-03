/**
 * 存放处理后的IMU姿态以及GNSS位置
*/
#include "lidar_localization/sensor_data/key_frame_data.hpp"

namespace lidar_localization {
Eigen::Quaternionf KeyFrameData::GetQuaternion() {
    Eigen::Quaternionf q;
    q = this->pose.block<3, 3>(0, 0);

    return q;
}
}