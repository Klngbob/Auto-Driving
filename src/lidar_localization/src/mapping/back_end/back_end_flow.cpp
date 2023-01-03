/**
 * 后端任务管理，放在类里使代码更清晰
*/
#include "lidar_localization/mapping/back_end/back_end_flow.hpp"

#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_definition/global_definition.h"

namespace lidar_localization {
BackEndFlow::BackEndFlow(ros::NodeHandle& nh) {
    // subscriber
    this->cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
    this->laser_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/laser_odom", 100000);
    this->gnss_pose_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);
    // publisher
    this->transformed_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "transformed_odom", "map", "lidar", 100);
    this->key_frame_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh, "key_frame", "map", 100);
    this->key_frames_pub_ptr_ = std::make_shared<KeyFramesPublisher>(nh, "optimized_key_frames", "map", 100);

    this->back_end_ptr_ = std::make_shared<BackEnd>();
}

bool BackEndFlow::Run() {
    if(!this->ReadData())
        return false;
    
    while(this->HasData()) {
        if(!this->ValidData())
            continue;
        
        this->UpdateBackEnd();

        this->PublishData();
    }
    
    return true;
}

bool BackEndFlow::ReadData() {
    this->cloud_sub_ptr_->ParseData(this->cloud_data_buff_);
    this->laser_odom_sub_ptr_->ParseData(this->laser_odom_buff_);
    this->gnss_pose_sub_ptr_->ParseData(this->gnss_pose_data_buff_);
    
    return true;
}

bool BackEndFlow::HasData() {
    if(this->cloud_data_buff_.size() == 0 ||
       this->laser_odom_buff_.size() == 0 ||
       this->gnss_pose_data_buff_.size() == 0) {
        return false;
    }

    return true;
}

bool BackEndFlow::ValidData() {
    this->current_cloud_data_ = this->cloud_data_buff_.front();
    this->current_gnss_pose_data_ = this->gnss_pose_data_buff_.front();
    this->current_laser_odom_data_ = this->laser_odom_buff_.front();

    double diff_gnss_time = this->current_cloud_data_.time - this->current_gnss_pose_data_.time;
    double diff_laser_time = this->current_cloud_data_.time - this->current_laser_odom_data_.time;

    if(diff_gnss_time < -0.05 || diff_laser_time < -0.05) {
        this->cloud_data_buff_.pop_front();
        return false;
    }
    if(diff_gnss_time > 0.05) {
        this->gnss_pose_data_buff_.pop_front();
        return false;
    }
    if(diff_laser_time > 0.05) {
        this->laser_odom_buff_.pop_front();
        return false;
    }

    this->cloud_data_buff_.pop_front();
    this->gnss_pose_data_buff_.pop_front();
    this->laser_odom_buff_.pop_front();

    return true;
}

bool BackEndFlow::UpdateBackEnd() {
    static bool odometry_inited = false;
    static Eigen::Matrix4f odom_init_pose = Eigen::Matrix4f::Identity();

    if(!odometry_inited) {
        odometry_inited = true;
        odom_init_pose = this->current_gnss_pose_data_.pose * this->current_laser_odom_data_.pose.inverse();
    }
    this->current_laser_odom_data_.pose = odom_init_pose * this->current_laser_odom_data_.pose;

    return this->back_end_ptr_->Update(this->current_cloud_data_, this->current_laser_odom_data_, this->current_gnss_pose_data_);
}

bool BackEndFlow::PublishData() {
    this->transformed_odom_pub_ptr_->Publish(this->current_laser_odom_data_.pose, this->current_laser_odom_data_.time);
    
    if(this->back_end_ptr_->HasNewKeyFrame()) {
        KeyFrameData key_frame;
        this->back_end_ptr_->GetLatestKeyFrame(key_frame);
        this->key_frame_pub_ptr_->Publish(key_frame);
    }
    if(this->back_end_ptr_->HasNewOptimized()) {
        std::deque<KeyFrameData> optimized_key_frames;
        this->back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
        this->key_frames_pub_ptr_->Publish(optimized_key_frames);
    }

    return true;
}

}