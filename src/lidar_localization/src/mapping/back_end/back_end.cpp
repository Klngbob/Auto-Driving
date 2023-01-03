/**
 * 后端算法具体实现
*/
#include "lidar_localization/mapping/back_end/back_end.hpp"

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lidar_localization/global_definition/global_definition.h"
#include "lidar_localization/tools/file_manager.hpp"

namespace lidar_localization {
BackEnd::BackEnd() {
    this->InitWithConfig();
}

bool BackEnd::InitWithConfig() {
    std::string config_path_file = WORK_SPACE_PATH + "/config/back_end/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_path_file);

    this->InitParam(config_node);
    this->InitDataPath(config_node);

    return true;
}

bool BackEnd::InitParam(const YAML::Node& config_node) {
    this->key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    this->optimize_step_with_none_ = config_node["optimize_step_with_none"].as<int>();
    this->optimize_step_with_gnss_ = config_node["optimize_step_with_gnss"].as<int>();
    this->optimize_step_with_loop_ = config_node["optimize_step_with_loop"].as<int>();

    return true;
}

bool BackEnd::InitDataPath(const YAML::Node& config_node) {
    std::string data_path = config_node["data_path"].as<std::string>();
    if(data_path == "./") {
        data_path = WORK_SPACE_PATH;
    }
    
    if(!FileManager::CreateDirectory(data_path + "/slam_data"))
        return false;
    
    this->key_frames_path_ = data_path + "/slam_data/key_frames";
    this->trajectory_path_ = data_path + "/slam_data/trajectory";

    if(!FileManager::InitDirectory(key_frames_path_, "关键帧点云"))
        return false;

    if(!FileManager::InitDirectory(trajectory_path_, "轨迹文件"))
        return false;
    
    if(!FileManager::CreateFile(this->ground_truth_ofs_, this->trajectory_path_ + "/ground_truth.txt"))
        return false;
    
    if(!FileManager::CreateFile(this->laser_odom_ofs_, this->trajectory_path_ + "/laser_odom.txt"))
        return false;

    return true;
}

void BackEnd::ResetParam() {
    this->has_new_key_frame_ = false;
    this->has_new_optimized_ = false;

}

bool BackEnd::Update(const CloudData& cloud_data, const PoseData& laser_odom, const PoseData& gnss_pose) {
    this->ResetParam();

    this->SaveTrajectory(laser_odom, gnss_pose);

    if(this->MaybeNewFrame(cloud_data, laser_odom)) {
        this->MaybeOptimized();
    }

    return true;
}

bool BackEnd::SaveTrajectory(const PoseData& laser_odom, const PoseData& gnss_pose) {
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 4; ++j) {
            this->ground_truth_ofs_ << gnss_pose.pose(i, j);
            this->laser_odom_ofs_ << laser_odom.pose(i, j);
            if(i == 2 && j == 3) {
                this->ground_truth_ofs_ << std::endl;
                this->laser_odom_ofs_ << std::endl;
            } else {
                this->ground_truth_ofs_ << " ";
                this->laser_odom_ofs_ << " ";
            }
        }
    }

    return true;
}

bool BackEnd::MaybeNewFrame(const CloudData& cloud_data, const PoseData& laser_odom) {
    if(this->key_frames_deque_.size() == 0) {
        this->has_new_key_frame_ = true;
        this->last_key_pose_ = laser_odom.pose;
    }

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应的更新
    if(fabs(laser_odom.pose(0, 3) - this->last_key_pose_(0, 3)) +
       fabs(laser_odom.pose(1, 3) - this->last_key_pose_(1, 3)) +
       fabs(laser_odom.pose(2, 3) - this->last_key_pose_(2, 3)) > this->key_frame_distance_) {
        this->has_new_key_frame_ = true;
        this->last_key_pose_ = laser_odom.pose;
    }

    if(this->has_new_key_frame_) {
    // 把关键帧存入硬盘中，节省内存
        std::string file_path = this->key_frames_path_ + "/key_frame_" + std::to_string(this->key_frames_deque_.size()) + ".pcd";
        pcl::io::savePCDFileBinary(file_path, *cloud_data.cloud_ptr);

        KeyFrameData key_frame;
        key_frame.time = laser_odom.time;
        key_frame.index = (unsigned int)this->key_frames_deque_.size();
        key_frame.pose = laser_odom.pose;
        this->key_frames_deque_.push_back(key_frame);

        this->last_key_frame_ = key_frame;
    }

    return this->has_new_key_frame_;
}

bool BackEnd::MaybeOptimized() {
    static int unoptimized_cnt = 0;
    if(++unoptimized_cnt > this->optimize_step_with_none_) {
        unoptimized_cnt = 0;
        this->has_new_optimized_ = true;
    }

    return true;
}

void BackEnd::GetOptimizedKeyFrames(std::deque<KeyFrameData>& key_frames_deque) {
    key_frames_deque = this->key_frames_deque_;
}

bool BackEnd::HasNewKeyFrame() {
    return this->has_new_key_frame_;
}

bool BackEnd::HasNewOptimized() {
    return this->has_new_optimized_;
}

void BackEnd::GetLatestKeyFrame(KeyFrameData& key_frame) {
    key_frame = this->last_key_frame_;
}


}