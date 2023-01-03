/**
 * 后端算法具体实现
*/
#ifndef LIDAR_LOCALIZATION_MAPPING_BACK_END_BACK_END_HPP_
#define LIDAR_LOCALIZATION_MAPPING_BACK_END_BACK_END_HPP_

#include <string>
#include <deque>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/sensor_data/pose_data.hpp"
#include "lidar_localization/sensor_data/key_frame_data.hpp"

namespace lidar_localization {
class BackEnd {
    public:
        BackEnd();

        bool Update(const CloudData& cloud_data, const PoseData& laser_odom, const PoseData& gnss_pose);

        void GetOptimizedKeyFrames(std::deque<KeyFrameData>& key_frames_deque);
        bool HasNewKeyFrame();
        bool HasNewOptimized();
        void GetLatestKeyFrame(KeyFrameData& key_frame);

    private:
        bool InitWithConfig();
        bool InitParam(const YAML::Node& config_node);
        bool InitDataPath(const YAML::Node& config_node);

        void ResetParam();
        bool SaveTrajectory(const PoseData& laser_odom, const PoseData& gnss_pose); // 用于里程计精度对比
        bool MaybeNewFrame(const CloudData& cloud_data, const PoseData& laser_odom);
        bool MaybeOptimized();

    private:
        std::string key_frames_path_ = "";
        std::string trajectory_path_ = "";

        std::ofstream ground_truth_ofs_;
        std::ofstream laser_odom_ofs_;

        float key_frame_distance_ = 2.0;
        int optimize_step_with_none_ = 100;
        int optimize_step_with_gnss_ = 100;
        int optimize_step_with_loop_ = 10;

        bool has_new_key_frame_ = false;
        bool has_new_optimized_ = false;

        Eigen::Matrix4f last_key_pose_ = Eigen::Matrix4f::Identity();
        KeyFrameData last_key_frame_;
        std::deque<KeyFrameData> key_frames_deque_;

};
}

#endif