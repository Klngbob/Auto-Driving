#include "lidar_localization/tf_listener/tf_listener.hpp"

#include <Eigen/Geometry>

namespace lidar_localization {
TFListener::TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id)
    :nh_(nh), base_frame_id_(base_frame_id), child_frame_id_(child_frame_id) {}

// 读取tf转换矩阵至transform_matrix
bool TFListener::LookupData(Eigen::Matrix4f& transform_matrix) {
    try
    {
        tf::StampedTransform transform;
        // 读取child_frame到base_frame的坐标系转换
        this->listener_.lookupTransform(this->base_frame_id_, this->child_frame_id_, ros::Time(0), transform);
        this->TranformToMatrix(transform, transform_matrix);
        return true;
    }
    catch(tf::TransformException& ex)
    {
        return false;
    }
    
}

bool TFListener::TranformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix) {
    Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());

    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());

    // 变换矩阵：child_frame到base_frame的坐标系转换
    transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

    return true;
}
}