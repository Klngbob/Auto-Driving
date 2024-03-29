/**
 * tf监听模块
*/
#ifndef LIDAR_LOCALIZATION_TF_LISTENER_TF_LISTENER_HPP_
#define LIDAR_LOCALIZATION_TF_LISTENER_TF_LISTENER_HPP_

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace lidar_localization {
class TFListener {
    public:
        TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id);
        TFListener() = default;

        bool LookupData(Eigen::Matrix4f& tranform_matrix);

    private:
        bool TranformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix);

    private:
        ros::NodeHandle nh_;
        tf::TransformListener listener_;
        std::string base_frame_id_;
        std::string child_frame_id_;
};
}

#endif