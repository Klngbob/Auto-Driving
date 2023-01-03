/**
 * 前端任务管理，放在类里使代码更清晰
*/
#ifndef LIDAR_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_
#define LIDAR_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_

#include <ros/ros.h>
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/mapping/front_end/front_end.hpp"

namespace lidar_localization {
class FrontEndFlow {
    public:
        FrontEndFlow(ros::NodeHandle& nh);

        bool Run();
        // bool SaveMap();
        // bool PublishGlobalMap();
    
    private:
        bool ReadData();
        // bool InitCalibration(); // 初始化标定文件
        // bool InitGNSS();
        bool HasData();
        bool ValidData();
        // bool UpdateGNSSOdometry();
        bool UpdateLaserOdometry();
        bool PublishData();
        // bool SaveTrajectory();
    
    private:
        //只订阅点云数据，发布里程计信息
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
        std::shared_ptr<FrontEnd> front_end_ptr_;

        std::deque<CloudData> cloud_data_buff_;

        CloudData current_cloud_data_;

        Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();

};
}

#endif