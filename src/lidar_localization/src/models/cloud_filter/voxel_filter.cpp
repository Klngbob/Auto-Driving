/**
 * voxel filter 模块
*/
#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "glog/logging.h"

namespace lidar_localization {
VoxelFilter::VoxelFilter(const YAML::Node& node) {
    float leaf_size_x = node["leaf_size"][0].as<float>();
    float leaf_size_y = node["leaf_size"][1].as<float>();
    float leaf_size_z = node["leaf_size"][2].as<float>();

    this->SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

VoxelFilter::VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    this->SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

bool VoxelFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
    this->voxel_filter_.setInputCloud(input_cloud_ptr);
    this->voxel_filter_.filter(*filtered_cloud_ptr);

    return true;
}

bool VoxelFilter::SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    this->voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);

    LOG(INFO) << "Voxel Filter的参数为: " <<std::endl
              << leaf_size_x << ", "
              << leaf_size_y << ", "
              << leaf_size_z
              << std::endl << std::endl;

    return true;
}
}