/**
 * 前端任务管理，放在类里使得代码更清晰
*/

#include "lidar_localization/front_end/front_end_flow.hpp"

#include "glog/logging.h"

namespace lidar_localization {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {
    this->cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    this->imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    // this->velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
    this->gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    this->lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "velo_link", "imu_link");

    this->cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "map");
    this->local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "local_map", 100, "map");
    this->global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "global_map", 100, "map");
    this->laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "laser_odom", "map", "lidar", 100);
    this->gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "gnss", "map", "lidar", 100);

    this->front_end_ptr_ = std::make_shared<FrontEnd>();

    this->local_map_ptr_.reset(new CloudData::CLOUD());
    this->global_map_ptr_.reset(new CloudData::CLOUD());
    this->current_scan_ptr_.reset(new CloudData::CLOUD());
}

bool FrontEndFlow::Run() {
    this->ReadData();

    if(!this->InitCalibration())
        return false;
    
    if(!this->InitGNSS())
        return false;

    while(this->HasData()) {
        if(!ValidData())
            continue;
        this->UpdateGNSSOdometry();
        if(this->UpdateLaserOdometry())
            this->PublishData();
    }

    return true;
}

bool FrontEndFlow::ReadData() {
    // 订阅并解析数据集中的数据
    this->cloud_sub_ptr_->ParseData(this->cloud_data_buff_);

    static std::deque<IMUData> unsynced_imu_;
    static std::deque<VelocityData> unsynced_vel_;
    static std::deque<GNSSData> unsynced_gnss_;


    this->imu_sub_ptr_->ParseData(unsynced_imu_);
    // this->velocity_sub_ptr_->ParseData(unsynced_vel_);
    this->gnss_sub_ptr_->ParseData(unsynced_gnss_);

    if(this->cloud_data_buff_.size() == 0)
        return false;
    
    double cloud_time = this->cloud_data_buff_.front().time;
    bool valid_imu = IMUData::SyncData(unsynced_imu_, this->imu_data_buff_, cloud_time);
    bool valid_velocity;
    // bool valid_velocity = VelocityData::SyncData(unsynced_vel_, this->velocity_data_buff_, cloud_time);
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

bool FrontEndFlow::InitCalibration() {
    static bool calibration_received = false;
    if(!calibration_received) {
        if(this->lidar_to_imu_ptr_->LookupData(this->imu_to_lidar_)) {
            calibration_received = true;
        }
    }

    return calibration_received;
}

bool FrontEndFlow::InitGNSS() {
    static bool gnss_inited = false;
    if(!gnss_inited && this->gnss_data_buff_.size() > 0) {
        GNSSData gnss_data = this->gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}

bool FrontEndFlow::HasData() {
    if(this->cloud_data_buff_.size() == 0 ||
       this->gnss_data_buff_.size() == 0  ||
       this->imu_data_buff_.size() == 0   ||
       this->velocity_data_buff_.size() == 0) {
        return false;
    }
    return true;
}

bool FrontEndFlow::ValidData() {
    this->current_cloud_data_ = this->cloud_data_buff_.front();
    this->current_imu_data_ = this->imu_data_buff_.front();
    this->current_GNSS_data_ = this->gnss_data_buff_.front();
    this->current_velocity_data_ = this->velocity_data_buff_.front();

    double d_time = this->current_cloud_data_.time - this->current_imu_data_.time;
    if(d_time < -0.05) {
        this->cloud_data_buff_.pop_front();
        return false;
    }
    if(d_time > 0.05) {
        this->imu_data_buff_.pop_front();
        this->gnss_data_buff_.pop_front();
        this->velocity_data_buff_.pop_front();
        return false;
    }

    this->cloud_data_buff_.pop_front();
    this->imu_data_buff_.pop_front();
    this->gnss_data_buff_.pop_front();
    this->velocity_data_buff_.pop_front();

    return true;
}

bool FrontEndFlow::UpdateGNSSOdometry() {
    this->gnss_odometry_ = Eigen::Matrix4f::Identity();
    this->current_GNSS_data_.UpdateXYZ();
    this->gnss_odometry_(0, 3) = this->current_GNSS_data_.local_E;
    this->gnss_odometry_(1, 3) = this->current_GNSS_data_.local_N;
    this->gnss_odometry_(2, 3) = this->current_GNSS_data_.local_U;
    this->gnss_odometry_.block<3, 3>(0, 0) = this->current_imu_data_.GetOrientationMatrix();
    this->gnss_odometry_ *= this->imu_to_lidar_;

    return true;
}

bool FrontEndFlow::UpdateLaserOdometry() {
    static bool front_end_pose_inited = false;
    if(!front_end_pose_inited) {
        front_end_pose_inited = true;
        this->front_end_ptr_->SetInitPose(this->gnss_odometry_);
        this->laser_odometry_ = this->gnss_odometry_;
        return true;
    }

    this->laser_odometry_ = Eigen::Matrix4f::Identity();
    if(this->front_end_ptr_->Update(this->current_cloud_data_, this->laser_odometry_))
        return true;
    else
        return false;
}
bool FrontEndFlow::PublishData() {
    this->gnss_pub_ptr_->Publish(this->gnss_odometry_);
    this->laser_odom_pub_ptr_->Publish(this->laser_odometry_);

    this->front_end_ptr_->GetCurrentScan(this->current_scan_ptr_);
    this->cloud_pub_ptr_->Publish(this->current_scan_ptr_);

    if(this->front_end_ptr_->GetNewLocalMap(this->local_map_ptr_)) {
        this->local_map_pub_ptr_->Publish(this->local_map_ptr_);
    }

    return true;
}

bool FrontEndFlow::SaveMap() {
    return this->front_end_ptr_->SaveMap();
}

bool FrontEndFlow::PublishGlobalMap() {
    if(this->front_end_ptr_->GetNewGlobalMap(this->global_map_ptr_)) {
        this->global_map_pub_ptr_->Publish(this->global_map_ptr_);
        this->global_map_ptr_.reset(new CloudData::CLOUD());
    }

    return true;
}

}