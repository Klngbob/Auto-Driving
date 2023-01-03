/**
 * 数据预处理模块，时间同步、点云运动畸变补偿
*/
#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_definition/global_definition.h"

namespace lidar_localization {
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh) {
    // subscriber
    this->cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    this->imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    this->velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
    this->gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    this->lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "velo_link", "imu_link");
    // publisher
    this->cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "synced_cloud", "velo_link", 100);
    this->gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "synced_gnss", "map", "velo_link", 100);

    this->distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
}

bool DataPretreatFlow::Run() {
    if(!this->ReadData())
        return false;
    
    if(!this->InitCalibration())
        return false;
    
    if(!this->InitGNSS())
        return false;
    
    while(this->HasData()) {
        if(!this->ValidData())
            continue;
        
        this->TransformData();
        this->PublishData();
    }

    return true;
}

bool DataPretreatFlow::ReadData() {
    // 订阅并解析数据集中的数据
    this->cloud_sub_ptr_->ParseData(this->cloud_data_buff_);

    static std::deque<IMUData> unsynced_imu_;
    static std::deque<VelocityData> unsynced_vel_;
    static std::deque<GNSSData> unsynced_gnss_;


    this->imu_sub_ptr_->ParseData(unsynced_imu_);
    this->velocity_sub_ptr_->ParseData(unsynced_vel_);
    this->gnss_sub_ptr_->ParseData(unsynced_gnss_);

    if(this->cloud_data_buff_.size() == 0)
        return false;
    
    double cloud_time = this->cloud_data_buff_.front().time;
    bool valid_imu = IMUData::SyncData(unsynced_imu_, this->imu_data_buff_, cloud_time);
    bool valid_velocity = VelocityData::SyncData(unsynced_vel_, this->velocity_data_buff_, cloud_time);
    bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, this->gnss_data_buff_, cloud_time);

    static bool sensor_inited = false;
    if(!sensor_inited) {
        if(!valid_imu || !valid_gnss || !valid_velocity) {
            this->cloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }
    return true;
}

bool DataPretreatFlow::InitCalibration() {
    // 标定lidar和imu位置
    static bool calibration_received = false;
    if(!calibration_received) {
        if(this->lidar_to_imu_ptr_->LookupData(this->imu_to_lidar_)) {
            calibration_received = true;
        }
    }

    return calibration_received;
}

bool DataPretreatFlow::InitGNSS() {
    static bool gnss_inited = false;
    if(!gnss_inited && this->gnss_data_buff_.size() > 0) {
        GNSSData gnss_data = this->gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}

bool DataPretreatFlow::HasData() {
    if(this->cloud_data_buff_.size() == 0 ||
       this->gnss_data_buff_.size() == 0  ||
       this->imu_data_buff_.size() == 0   ||
       this->velocity_data_buff_.size() == 0) {
        return false;
    }
    return true;
}

bool DataPretreatFlow::ValidData() {
    // 剔除无效数据
    this->current_cloud_data_ = this->cloud_data_buff_.front();
    this->current_imu_data_ = this->imu_data_buff_.front();
    this->current_GNSS_data_ = this->gnss_data_buff_.front();
    this->current_velocity_data_ = this->velocity_data_buff_.front();

    double diff_imu_time = this->current_cloud_data_.time - this->current_imu_data_.time;
    double diff_velocity_time = this->current_cloud_data_.time - this->current_velocity_data_.time;
    double diff_gnss_time = this->current_cloud_data_.time - this->current_GNSS_data_.time;
    if(diff_imu_time < -0.05 || diff_velocity_time < -0.05 || diff_gnss_time < -0.05) {
        this->cloud_data_buff_.pop_front();
        return false;
    }
    if(diff_imu_time > 0.05) {
        this->imu_data_buff_.pop_front();
        return false;
    }
    if(diff_velocity_time > 0.05) {
        this->velocity_data_buff_.pop_front();
        return false;
    }
    if(diff_gnss_time > 0.05) {
        this->gnss_data_buff_.pop_front();
        return false;
    }
    this->cloud_data_buff_.pop_front();
    this->imu_data_buff_.pop_front();
    this->gnss_data_buff_.pop_front();
    this->velocity_data_buff_.pop_front();

    return true;
}

bool DataPretreatFlow::TransformData() {
    // gnss坐标系转换到lidar坐标系
    this->gnss_pose_ = Eigen::Matrix4f::Identity();
    this->current_GNSS_data_.UpdateXYZ();
    this->gnss_pose_(0, 3) = this->current_GNSS_data_.local_E;
    this->gnss_pose_(1, 3) = this->current_GNSS_data_.local_N;
    this->gnss_pose_(2, 3) = this->current_GNSS_data_.local_U;
    this->gnss_pose_.block<3, 3>(0, 0) = this->current_imu_data_.GetOrientationMatrix();
    this->gnss_pose_ *= this->imu_to_lidar_;

    // 点云运动畸变补偿
    this->current_velocity_data_.TransformCoordinate(this->imu_to_lidar_);
    this->distortion_adjust_ptr_->SetMotionInfo(0.1, this->current_velocity_data_);
    this->distortion_adjust_ptr_->AdjustCloud(this->current_cloud_data_.cloud_ptr, this->current_cloud_data_.cloud_ptr);
    LOG(INFO) << "Transforming Data " << std::endl;
    return true;
}

bool DataPretreatFlow::PublishData() {
    LOG(WARNING) << "Publishing Data -1" << std::endl;
    this->cloud_pub_ptr_->Publish(this->current_cloud_data_.cloud_ptr, this->current_cloud_data_.time);
    LOG(WARNING) << "Publishing Data -2" << std::endl;
    this->gnss_pub_ptr_->Publish(this->gnss_pose_, this->current_GNSS_data_.time);
    LOG(WARNING) << "Published Data " << std::endl;

    return true;
}
}