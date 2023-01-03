/**
 * 前端任务管理，放在类里使代码更清晰
*/
#include "lidar_localization/mapping/front_end/front_end_flow.hpp"
#include "glog/logging.h"
#include "lidar_localization/global_definition/global_definition.h"

namespace lidar_localization {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {
    this->cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
    this->laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "laser_odom", "map", "lidar", 100);

    this->front_end_ptr_ = std::make_shared<FrontEnd>();

}

bool FrontEndFlow::Run() {
    if(!this->ReadData())
        return false;

    while(this->HasData()) {
        if(!ValidData())
            continue;

        if(this->UpdateLaserOdometry()) {
            this->PublishData();
        }
    }

    return true;
}

bool FrontEndFlow::ReadData() {
    // 订阅已经同步并补偿畸变过的点云数据
    this->cloud_sub_ptr_->ParseData(this->cloud_data_buff_);

    return true;
}

bool FrontEndFlow::HasData() {
    return this->cloud_data_buff_.size() > 0;
}

bool FrontEndFlow::ValidData() {
    this->current_cloud_data_ = this->cloud_data_buff_.front();
    this->cloud_data_buff_.pop_front();

    return true;
}

bool FrontEndFlow::UpdateLaserOdometry() {
    static bool odometry_inited = false;
    if(!odometry_inited) {
        odometry_inited = true;
        this->front_end_ptr_->SetInitPose(Eigen::Matrix4f::Identity());
        return this->front_end_ptr_->Update(this->current_cloud_data_, this->laser_odometry_);
    }

    return this->front_end_ptr_->Update(this->current_cloud_data_, this->laser_odometry_);
}

bool FrontEndFlow::PublishData() {
    this->laser_odom_pub_ptr_->Publish(this->laser_odometry_, this->current_cloud_data_.time);

    return true;
}

}
