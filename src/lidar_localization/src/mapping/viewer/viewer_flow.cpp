#include "lidar_localization/mapping/viewer/viewer_flow.hpp"
#include "glog/logging.h"
#include "lidar_localization/global_definition/global_definition.h"

namespace lidar_localization {
ViewerFlow::ViewerFlow(ros::NodeHandle& nh) {
    // subscriber
    this->cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000); // 当前帧点云
    this->key_frame_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(nh, "/key_frame", 100000); // 当前关键帧
    this->transformed_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/transformed_odom", 100000); // 当前帧位姿
    this->optimized_key_frames_sub_ptr_ = std::make_shared<KeyFramesSubscriber>(nh, "/optimized_key_frames", 100000); // 历史关键帧
    // publisher
    this->optimized_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "optimized_odom", "map", "lidar", 100);
    this->current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "current_scan", "map", 100);
    this->global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "global_map", "map", 100);
    this->local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "local_map", "map", 100);
    // viewer
    this->viewer_ptr_ = std::make_shared<Viewer>();
}

bool ViewerFlow::Run() {
    if(!this->ReadData())
        return false;
    
    while (this->HasData())
    {
        if(!this->ValidData())
            continue;
        
        this->UpdateViewer();

        this->PublishData();
    }

    return true;
}

bool ViewerFlow::ReadData() {
    this->cloud_sub_ptr_->ParseData(this->cloud_data_buff_);
    this->key_frame_sub_ptr_->ParseData(this->key_frame_buff_);
    this->transformed_odom_sub_ptr_->ParseData(this->transformed_odom_buff_);
     this->optimized_key_frames_sub_ptr_->ParseData(this->all_key_frames_);

    return true;
}

bool ViewerFlow::HasData() {
    if(this->cloud_data_buff_.size() == 0 ||
       this->transformed_odom_buff_.size() == 0) {
        return false;
    }

    return true;
}

bool ViewerFlow::ValidData() {
    this->current_cloud_data_ = this->cloud_data_buff_.front();
    this->current_transformed_odom_ = this->transformed_odom_buff_.front();

    double diff_odom_time = this->current_cloud_data_.time - this->current_transformed_odom_.time;

    if(diff_odom_time < -0.05) {
        this->cloud_data_buff_.pop_front();
        return false;
    }

    if(diff_odom_time > 0.05) {
        this->transformed_odom_buff_.pop_front();
        return false;
    }

    this->cloud_data_buff_.pop_front();
    this->transformed_odom_buff_.pop_front();

    return true;
}

bool ViewerFlow::UpdateViewer() {
    return this->viewer_ptr_->Update(this->key_frame_buff_,
                                     this->optimized_key_frames_,
                                     this->current_transformed_odom_,
                                     this->current_cloud_data_);
}

bool ViewerFlow::PublishData() {
    this->optimized_odom_pub_ptr_->Publish(this->viewer_ptr_->GetCurrentPose());
    this->current_scan_pub_ptr_->Publish(this->viewer_ptr_->GetCurrentScan());

    if(this->viewer_ptr_->HasNewLocalMap() && this->local_map_pub_ptr_->HasSubscribers()) {
        CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
        this->viewer_ptr_->GetLocalMap(cloud_ptr);
        this->local_map_pub_ptr_->Publish(cloud_ptr);
    }

    if(this->viewer_ptr_->HasNewGlobalMap() && this->global_map_pub_ptr_->HasSubscribers()) {
        CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
        this->viewer_ptr_->GetGlobalMap(cloud_ptr);
        this->global_map_pub_ptr_->Publish(cloud_ptr);
    }

    return true;
}

bool ViewerFlow::SaveMap() {
    return this->viewer_ptr_->SaveMap();
}

}