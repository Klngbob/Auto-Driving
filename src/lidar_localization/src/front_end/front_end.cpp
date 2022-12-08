/**
 * 前端里程计算法
*/
#include "lidar_localization/front_end/front_end.hpp"

#include <cmath>
#include <pcl/common/transforms.h>
#include <glog/logging.h>

namespace lidar_localization {
FrontEnd::FrontEnd()
    :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()),
    local_map_ptr_(new CloudData::CLOUD()),
    global_map_ptr_(new CloudData::CLOUD()),
    result_cloud_ptr_(new CloudData::CLOUD()) {
    // 设置默认参数，以免类的使用者在匹配之前忘了设置参数
        this->cloud_filter_.setLeafSize(1.3, 1.3, 1.3);
        this->local_map_filter_.setLeafSize(0.6, 0.6, 0.6);
        this->display_filter_.setLeafSize(0.5, 0.5, 0.5);
        this->ndt_ptr_->setResolution(1.0);
        this->ndt_ptr_->setStepSize(0.1);
        this->ndt_ptr_->setTransformationEpsilon(0.01); // 收敛条件
        this->ndt_ptr_->setMaximumIterations(30);
    }

Eigen::Matrix4f FrontEnd::Update(const CloudData& cloud_data) {
    this->current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *this->current_frame_.cloud_data.cloud_ptr, indices);

    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    this->cloud_filter_.setInputCloud(this->current_frame_.cloud_data.cloud_ptr);
    this->cloud_filter_.filter(*filtered_cloud_ptr);

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = this->init_pose_;
    static Eigen::Matrix4f predict_pose = this->init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = this->init_pose_;

    // 局部地图里没有关键帧，说明这是第一帧
    // 此时把当前帧作为第一个关键帧，并更新局部地图和全局地图
    if(this->local_map_frames_.size() == 0) {
        this->current_frame_.pose = this->init_pose_;
        this->UpdateNewFrame(this->current_frame_);
        return this->current_frame_.pose;
    }

    // 如果不是第一帧，则正常匹配
    this->ndt_ptr_->setInputSource(filtered_cloud_ptr); // 设置ndt匹配的源点云
    /*
    align()函数是pcl库中的一个函数，主要用于对两个点云进行配准（点云对齐）。它接收两个参数，分别是待配准的源点云和目标点云，
    并返回一个配准后的点云对象。
    getFinalTransformation()函数是pcl库中的一个函数，主要用于获取配准过程中最后一次迭代计算得到的变换矩阵。
    它接收一个参数，即待配准的源点云，并返回一个Eigen::Matrix4f类型的变换矩阵。
    两个函数的区别在于，align()函数是用于实现点云配准的核心函数，它在运行过程中会自动计算出变换矩阵，并将配准后的点云作为返回值返回。
    而getFinalTransformation()函数则是用于获取配准过程中最后一次迭代计算得到的变换矩阵，它并不会对点云进行配准，只返回变换矩阵。
    实际应用中，通常是先使用align()函数对点云进行配准，然后再使用getFinalTransformation()函数获取配准过程中计算得到的变换矩阵。
    */
    this->ndt_ptr_->align(*this->result_cloud_ptr_, predict_pose); // 
    this->current_frame_.pose = this->ndt_ptr_->getFinalTransformation();

    // 更新相邻两帧的相对运动，用k-2 和 k-1帧预测当前第k帧的位姿
    step_pose = last_pose.inverse() * this->current_frame_.pose;
    predict_pose = this->current_frame_.pose * step_pose;
    last_pose = this->current_frame_.pose;

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应的更新
    if(fabs(last_key_frame_pose(0, 3) - this->current_frame_.pose(0, 3)) +
       fabs(last_key_frame_pose(1, 3) - this->current_frame_.pose(1, 3)) +
       fabs(last_key_frame_pose(2, 3) - this->current_frame_.pose(2, 3)) > 2.0) {
        this->UpdateNewFrame(this->current_frame_);
        last_key_frame_pose = this->current_frame_.pose;
    }

    return this->current_frame_.pose;
}

// 设置里程计初始位姿
bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
    this->init_pose_ = init_pose;
    return true;
}

// 设置预测位姿
bool FrontEnd::SetPredictPose(const Eigen::Matrix4f& predict_pose) {
    this->predict_pose_ = predict_pose;
    return true;
}

// 添加新的关键帧
void FrontEnd::UpdateNewFrame(const Frame& new_key_frame) {
    Frame key_frame = new_key_frame;
    // 这一步的目的是为了把关键帧的点云保存下来
    // 由于用的是shared_ptr，所以直接复制只是复制了一个指针而已
    // 此时无论放多少个关键帧在容器里，这些关键帧点云指针都指向同一个点云
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());

    // 更新局部地图
    this->local_map_frames_.push_back(key_frame);
    // 滑动窗口数为20
    while(this->local_map_frames_.size() > 20) {
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
    if(this->local_map_frames_.size() < 10) {
        this->ndt_ptr_->setInputTarget(this->local_map_ptr_);
    }
    else {
        CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
        this->local_map_filter_.setInputCloud(this->local_map_ptr_);
        this->local_map_filter_.filter(*filtered_local_map_ptr);
        this->ndt_ptr_->setInputTarget(filtered_local_map_ptr);
    }

    // 更新全局地图
    this->global_map_frames_.push_back(key_frame);
    if(this->global_map_frames_.size() % 100 != 0) {
        return;
    } else {
        this->global_map_ptr_.reset(new CloudData::CLOUD());
        for(size_t i = 0; i < this->global_map_frames_.size(); ++i) {
            pcl::transformPointCloud(*this->global_map_frames_.at(i).cloud_data.cloud_ptr,
                                     *transformed_cloud_ptr,
                                     this->global_map_frames_.at(i).pose);
            *this->global_map_ptr_ += *transformed_cloud_ptr;
        }
        this->has_new_global_map_ = true;
    }
}

bool FrontEnd::GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
    if(this->has_new_local_map_) {
        this->display_filter_.setInputCloud(this->local_map_ptr_);
        this->display_filter_.filter(*local_map_ptr);
        return true;
    }
    return false;
}

bool FrontEnd::GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) {
    if(this->has_new_global_map_) {
        this->display_filter_.setInputCloud(this->global_map_ptr_);
        this->display_filter_.filter(*global_map_ptr);
        return true;
    }
    return false;
}

bool FrontEnd::GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr) {
    this->display_filter_.setInputCloud(this->result_cloud_ptr_);
    this->display_filter_.filter(*current_scan_ptr);
    return true;
}

}