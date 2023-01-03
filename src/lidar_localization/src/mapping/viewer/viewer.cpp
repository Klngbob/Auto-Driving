#include "lidar_localization/mapping/viewer/viewer.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_definition/global_definition.h"

namespace lidar_localization {
Viewer::Viewer() {
    this->InitWithConfig();
}

bool Viewer::InitWithConfig() {
    std::string config_path_file = WORK_SPACE_PATH + "/config/viewer/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_path_file);

    this->InitParam(config_node);
    this->InitDataPath(config_node);
    this->InitFilter("frame", this->frame_filter_ptr_, config_node);
    this->InitFilter("local_map", this->local_map_filter_ptr_, config_node);
    this->InitFilter("global_map", this->global_map_filter_ptr_, config_node);

    return true;
}

bool Viewer::InitParam(const YAML::Node& config_node) {
    this->local_frame_num_ = config_node["local_frame_num"].as<int>();

     return true;
}

bool Viewer::InitDataPath(const YAML::Node& config_node) {
    this->data_path_ = config_node["data_path"].as<std::string>();
    if(this->data_path_ == "./") {
        this->data_path_ = WORK_SPACE_PATH;
    }

    this->key_frames_path_ = this->data_path_ + "/slam_data/key_frames";
    this->map_path_ = this->data_path_ + "/slam_data/map";

    if(!FileManager::InitDirectory(this->map_path_, "点云地图文件"))
        return false;

    return true;
}

bool Viewer::InitFilter(std::string filter_user,
                        std::shared_ptr<CloudFilterInterface>& filter_ptr,
                        const YAML::Node& config_node) {

    std::string filter_method = config_node[filter_user + "_filter"].as<std::string>();
    LOG(INFO) << "viewer_" << filter_user << "选择的滤波方法为：" << filter_method;

    if(filter_method == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
    } else {
        LOG(ERROR) << "没有为  " << filter_user << "找到与" << filter_method << "对应的滤波方式！";
        return false;
    }
    
    return true;
}

bool Viewer::Update(std::deque<KeyFrameData>& new_key_frames,
                    std::deque<KeyFrameData>& optimized_key_frames,
                    PoseData transformed_data,
                    CloudData cloud_data) {

    this->ResetParam();

    if(optimized_key_frames.size() > 0) {
        this->optimized_key_frames_ = optimized_key_frames;
        optimized_key_frames.clear();
        this->OptimizeKeyFrames();
        this->has_new_global_map_ = true;
    }

    if(new_key_frames.size() > 0) {
        this->all_key_frames_.insert(this->all_key_frames_.end(), new_key_frames.begin(), new_key_frames.end());
        new_key_frames.clear();
        this->has_new_local_map_ = true;
    }

    this->optimized_odom_ = transformed_data;
    this->optimized_odom_.pose = this->pose_to_optimzie_ * this->optimized_odom_.pose;

    this->optimized_cloud_ = cloud_data;
    pcl::transformPointCloud(*cloud_data.cloud_ptr, *this->optimized_cloud_.cloud_ptr, this->optimized_odom_.pose);

    return true;
}

void Viewer::ResetParam() {
    this->has_new_global_map_ = false;
    this->has_new_local_map_ = false;
}

bool Viewer::OptimizeKeyFrames() {
    size_t optimized_index = 0;
    size_t all_index = 0;
    while (optimized_index < this->optimized_key_frames_.size() && all_index < this->all_key_frames_.size()) {
        if (this->optimized_key_frames_.at(optimized_index).index < this->all_key_frames_.at(all_index).index) {
            optimized_index++;
        } else if (this->optimized_key_frames_.at(optimized_index).index > this->all_key_frames_.at(all_index).index) {
            all_index++;
        } else {
            this->pose_to_optimzie_ = this->optimized_key_frames_.at(optimized_index).pose * this->all_key_frames_.at(all_index).pose.inverse();
            this->all_key_frames_.at(all_index) = this->optimized_key_frames_.at(optimized_index);
            optimized_index++;
            all_index++;
        }
    }

    while (all_index < this->all_key_frames_.size()) {
        this->all_key_frames_.at(all_index).pose = this->pose_to_optimzie_ * this->all_key_frames_.at(all_index).pose;
        all_index++;
    }

    return true;
}

bool Viewer::JointGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) {
    this->JointCloudMap(this->optimized_key_frames_, global_map_ptr);

    return true;
}

bool Viewer::JointLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
    size_t begin_index = 0;
    if(this->all_key_frames_.size() > (size_t)this->local_frame_num_)
        begin_index = this->all_key_frames_.size() - (size_t)this->local_frame_num_;
    
    std::deque<KeyFrameData> local_key_frames;
    for(size_t i = begin_index; i < this->all_key_frames_.size(); ++i)
        local_key_frames.push_back(this->all_key_frames_.at(i));

    this->JointCloudMap(local_key_frames, local_map_ptr);
}

bool Viewer::JointCloudMap(const std::deque<KeyFrameData>& key_frames, CloudData::CLOUD_PTR& map_cloud_ptr) {
    map_cloud_ptr.reset(new CloudData::CLOUD());

    CloudData::CLOUD_PTR cloud_ptr(new CloudData::CLOUD());
    std::string file_path = "";

    for(size_t i = 0; i < key_frames.size(); ++i) {
        file_path = this->key_frames_path_ + "/key_frame_" + std::to_string(key_frames.at(i).index) + ".pcd";
        pcl::io::loadPCDFile(file_path, *cloud_ptr);
        pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, key_frames.at(i).pose);
        *map_cloud_ptr += *cloud_ptr;
    }

    return true;
}

bool Viewer::SaveMap() {
    if(this->optimized_key_frames_.size() == 0)
        return false;

    CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
    this->JointCloudMap(this->optimized_key_frames_, global_map_ptr);

    std::string map_file_path = this->map_path_ + "/map.pcd";
    pcl::io::savePCDFileBinary(map_file_path, *global_map_ptr);

    LOG(INFO) << "地图保存完成，地址是: " << std::endl << map_file_path << std::endl << std::endl;

    return true;
}

Eigen::Matrix4f& Viewer::GetCurrentPose() {
    return this->optimized_odom_.pose;
}

CloudData::CLOUD_PTR& Viewer::GetCurrentScan() {
    this->frame_filter_ptr_->Filter(this->optimized_cloud_.cloud_ptr, this->optimized_cloud_.cloud_ptr);
    return this->optimized_cloud_.cloud_ptr;
}

bool Viewer::GetLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
    this->JointLocalMap(local_map_ptr);
    this->local_map_filter_ptr_->Filter(local_map_ptr, local_map_ptr);
    return true;
}

bool Viewer::GetGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) {
    this->JointGlobalMap(global_map_ptr);
    this->global_map_filter_ptr_->Filter(global_map_ptr, global_map_ptr);
    return true;
}

bool Viewer::HasNewLocalMap() {
    return this->has_new_local_map_;
}

bool Viewer::HasNewGlobalMap() {
    return this->has_new_global_map_;
}

}