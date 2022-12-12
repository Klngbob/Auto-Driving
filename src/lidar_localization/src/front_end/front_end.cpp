/**
 * 前端里程计算法
*/
#include "lidar_localization/front_end/front_end.hpp"

#include <fstream>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <glog/logging.h>

#include "lidar_localization/global_definition/global_definition.h"

namespace lidar_localization {
FrontEnd::FrontEnd()
    : local_map_ptr_(new CloudData::CLOUD()),
    global_map_ptr_(new CloudData::CLOUD()),
    result_cloud_ptr_(new CloudData::CLOUD()) {
          this->InitWithConfig();
    }

bool FrontEnd::InitWithConfig() {
    std::string config_path_file = WORK_SPACE_PATH + "/config/front_end/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_path_file);

    this->InitDataPath(config_node);
    this->InitRegistration(this->registration_ptr_, config_node);
    this->InitFilter("local_map", this->local_map_filter_ptr_, config_node);
    this->InitFilter("frame", this->frame_filter_ptr_, config_node);
    this->InitFilter("display", this->display_filter_ptr_, config_node);

    return true;
}

bool FrontEnd::InitParam(const YAML::Node& config_node) {
    this->key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    this->local_frame_num_ = config_node["local_frame_num"].as<int>();

    return true;
}

bool FrontEnd::InitDataPath(const YAML::Node& config_node) {
    this->data_path_ = config_node["data_path"].as<std::string>();
    if(this->data_path_ == "./") {
        this->data_path_ = WORK_SPACE_PATH;
    }
    this->data_path_ += "/slam_data";

    if(boost::filesystem::is_directory(this->data_path_)) {
        boost::filesystem::remove_all(this->data_path_);
    }

    boost::filesystem::create_directory(this->data_path_);
    if(!boost::filesystem::is_directory(this->data_path_)) {
        LOG(WARNING) << "文件夹" << this->data_path_ << "未创建成功!";
        return false;
    } else {
        LOG(INFO) << "地图点云存放地址："  << this->data_path_;
    }

    std::string key_frame_path = this->data_path_ + "/key_frames";
    boost::filesystem::create_directory(key_frame_path);
    if(!boost::filesystem::is_directory(key_frame_path)) {
        LOG(WARNING) << "文件夹" << key_frame_path << "未创建成功!";
        return false;
    } else {
        LOG(INFO) << "关键帧点云存放地址："  << key_frame_path << std::endl << std::endl;
    }
    
    return true;
}

bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    LOG(INFO) << "点云匹配方式为：" << registration_method;

    if(registration_method == "NDT") {
        this->registration_ptr_ = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else {
        LOG(ERROR) << "没找到与 " << registration_method << "对应的点云匹配方式！";
        return false;
    }

    return true;
}

bool FrontEnd::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_method = config_node[filter_user + "_filter"].as<std::string>();
    LOG(INFO) << filter_user << "选择的滤波方法为：" << filter_method;

    if(filter_method == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
    } else {
        LOG(ERROR) << "没有为  " << filter_user << "找到与" << filter_method << "对应的滤波方式！";
        return false;
    }
    
    return true;
}

bool FrontEnd::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {
    this->current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *this->current_frame_.cloud_data.cloud_ptr, indices);

    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    this->frame_filter_ptr_->Filter(this->current_frame_.cloud_data.cloud_ptr, filtered_cloud_ptr);

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = this->init_pose_;
    static Eigen::Matrix4f predict_pose = this->init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = this->init_pose_;

    // 局部地图里没有关键帧，说明这是第一帧
    // 此时把当前帧作为第一个关键帧，并更新局部地图和全局地图
    if(this->local_map_frames_.size() == 0) {
        this->current_frame_.pose = this->init_pose_;
        this->UpdateWithNewFrame(this->current_frame_);
        cloud_pose = this->current_frame_.pose;
        return true;
    }

    // 如果不是第一帧，则正常匹配
    // 设置ndt匹配的源点云
    /*
    align()函数是pcl库中的一个函数，主要用于对两个点云进行配准（点云对齐）。它接收两个参数，分别是待配准的源点云和目标点云，
    并返回一个配准后的点云对象。
    getFinalTransformation()函数是pcl库中的一个函数，主要用于获取配准过程中最后一次迭代计算得到的变换矩阵。
    它接收一个参数，即待配准的源点云，并返回一个Eigen::Matrix4f类型的变换矩阵。
    两个函数的区别在于，align()函数是用于实现点云配准的核心函数，它在运行过程中会自动计算出变换矩阵，并将配准后的点云作为返回值返回。
    而getFinalTransformation()函数则是用于获取配准过程中最后一次迭代计算得到的变换矩阵，它并不会对点云进行配准，只返回变换矩阵。
    实际应用中，通常是先使用align()函数对点云进行配准，然后再使用getFinalTransformation()函数获取配准过程中计算得到的变换矩阵。
    */
    this->registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, this->result_cloud_ptr_, this->current_frame_.pose);
    cloud_pose = this->current_frame_.pose;
    // 更新相邻两帧的相对运动，用k-2 和 k-1帧预测当前第k帧的位姿
    step_pose = last_pose.inverse() * this->current_frame_.pose;
    predict_pose = this->current_frame_.pose * step_pose;
    last_pose = this->current_frame_.pose;

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应的更新
    if(fabs(last_key_frame_pose(0, 3) - this->current_frame_.pose(0, 3)) +
       fabs(last_key_frame_pose(1, 3) - this->current_frame_.pose(1, 3)) +
       fabs(last_key_frame_pose(2, 3) - this->current_frame_.pose(2, 3)) > this->key_frame_distance_) {
        this->UpdateWithNewFrame(this->current_frame_);
        last_key_frame_pose = this->current_frame_.pose;
    }

    return true;
}

// 设置里程计初始位姿
bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
    this->init_pose_ = init_pose;
    return true;
}


// 添加新的关键帧
bool FrontEnd::UpdateWithNewFrame(const Frame& new_key_frame) {
    // 把关键帧存入硬盘中，节省内存
    std::string file_path = this->data_path_ + "/key_frames/key_frame_" + std::to_string(this->global_map_frames_.size()) + ".pcd";
    pcl::io::savePCDFileBinary(file_path, *new_key_frame.cloud_data.cloud_ptr);

    Frame key_frame = new_key_frame;
    // 这一步的目的是为了把关键帧的点云保存下来
    // 由于用的是shared_ptr，所以直接复制只是复制了一个指针而已
    // 此时无论放多少个关键帧在容器里，这些关键帧点云指针都指向同一个点云
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());

    // 更新局部地图
    this->local_map_frames_.push_back(key_frame);
    // 滑动窗口数为20
    while(this->local_map_frames_.size() > static_cast<size_t>(this->local_frame_num_)) {
        this->local_map_frames_.pop_front();
    }
    this->local_map_ptr_.reset(new CloudData::CLOUD());
    for(size_t i = 0; i < this->local_map_frames_.size(); ++i) {
        // 用at(i)有边界检查且速度不慢
        pcl::transformPointCloud(*this->local_map_frames_.at(i).cloud_data.cloud_ptr,
                                 *transformed_cloud_ptr,
                                 this->local_map_frames_.at(i).pose);
        *this->local_map_ptr_ += *transformed_cloud_ptr;
    }
    this->has_new_local_map_ = true;

    // 更新ndt匹配的目标点云
    // 关键帧少的时候还不能滤波，因为点云太稀疏影响匹配效果
    if(this->local_map_frames_.size() < 10) {
        this->registration_ptr_->SetInputTarget(this->local_map_ptr_);
    }
    else {
        CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
        this->local_map_filter_ptr_->Filter(this->local_map_ptr_, filtered_local_map_ptr);
        this->registration_ptr_->SetInputTarget(filtered_local_map_ptr);
    }

    // 更新全局地图
    // 保存所有关键帧信息在容器里
    // 存储之前，点云要先释放，因为已经存入了硬盘里，不释放也达不到节省内存的目的
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD());
    this->global_map_frames_.push_back(key_frame);

    return true;
}

bool FrontEnd::SaveMap() {
    this->global_map_ptr_.reset(new CloudData::CLOUD());

    std::string key_frame_path = "";
    CloudData::CLOUD_PTR key_frame_cloud_ptr(new CloudData::CLOUD());
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());

    for(size_t i = 0; i < this->global_map_frames_.size(); ++i) {
        key_frame_path = this->data_path_ + "/key_frames/key_frame_" + std::to_string(i) + ".pcd";
        pcl::io::loadPCDFile(key_frame_path, *key_frame_cloud_ptr);

        pcl::transformPointCloud(*key_frame_cloud_ptr,
                                 *transformed_cloud_ptr,
                                  global_map_frames_.at(i).pose);
        *this->global_map_ptr_ += *transformed_cloud_ptr;
    }

    std::string map_file_path = this->data_path_ + "/map.pcd";
    pcl::io::savePCDFileBinary(map_file_path, *this->global_map_ptr_);
    this->has_new_global_map_ = true;

    return true;
}

bool FrontEnd::GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
    if(this->has_new_local_map_) {
        this->display_filter_ptr_->Filter(this->local_map_ptr_, local_map_ptr);
        return true;
    }
    return false;
}

bool FrontEnd::GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) {
    if(this->has_new_global_map_) {
        this->has_new_global_map_ = false;
        this->display_filter_ptr_->Filter(this->global_map_ptr_, global_map_ptr);
        this->global_map_ptr_.reset(new CloudData::CLOUD()); // 释放内存吧
        return true;
    }
    return false;
}

bool FrontEnd::GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr) {
    this->display_filter_ptr_->Filter(this->result_cloud_ptr_, current_scan_ptr);
    return true;
}

}