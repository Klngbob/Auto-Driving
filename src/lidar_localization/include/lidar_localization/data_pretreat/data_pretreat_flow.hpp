/**
 * 数据预处理模块，时间同步、点云运动畸变补偿
*/
#ifndef LIDAR_LOCALIZATION_DATA_PRETREAT_FLOW_HPP_
#define LIDAR_LOCALIZATION_DATA_PRETREAT_FLOW_HPP_

#include <ros/ros.h>
// subscriber，原始数据的订阅处理
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/velocity_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/tf_listener/tf_listener.hpp"
// publisher，同步数据的发布
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"
// models，原始点云畸变补偿
#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"

namespace lidar_localization {
class DataPretreatFlow {
    public:
        DataPretreatFlow(ros::NodeHandle& nh);

        bool Run();
    
    private:
        bool ReadData();
        bool InitCalibration(); // 初始化标定文件
        bool InitGNSS();
        bool HasData();
        bool ValidData();
        bool TransformData();
        bool PublishData(); // 发布同步后的点云和GNSS数据

    private:
        // subscriber
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
        std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
        std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
        std::shared_ptr<TFListener> lidar_to_imu_ptr_;
        // publisher
        std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
        std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
        // models
        std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;

        Eigen::Matrix4f imu_to_lidar_ = Eigen::Matrix4f::Identity(); // imu到lidar的变化矩阵

        // 接收原始输入数据的缓冲区
        std::deque<CloudData> cloud_data_buff_;
        std::deque<IMUData> imu_data_buff_;
        std::deque<VelocityData> velocity_data_buff_;
        std::deque<GNSSData> gnss_data_buff_;

        // 当前读取的原始数据
        CloudData current_cloud_data_;
        IMUData current_imu_data_;
        VelocityData current_velocity_data_;
        GNSSData current_GNSS_data_;

        Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity(); // 当前gnss位姿（在lidar坐标系下）
};
}

#endif