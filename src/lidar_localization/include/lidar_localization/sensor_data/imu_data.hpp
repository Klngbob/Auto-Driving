#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_IMU_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_IMU_DATA_HPP_

#include <Eigen/Dense>
#include <deque>
#include <cmath>

namespace lidar_localization {
class IMUData {
    public:
        struct LinearAcceleration {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };

        struct AngularVelocity {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };

        class Orientation {
            public:
                double x = 0.0;
                double y = 0.0;
                double z = 0.0;
                double w = 0.0;
            public:
                void Normalize() {
                    double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
                    x /= norm;
                    y /= norm;
                    z /= norm;
                    w /= norm;
                }
        };

        double time = 0.0;
        LinearAcceleration linear_acceleration;
        AngularVelocity angular_velocity;
        Orientation orientation;

    public:
        // 把四元数转换成旋转矩阵送出去
        Eigen::Matrix3f GetOrientationMatrix();
        static bool SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time);
};
}

#endif