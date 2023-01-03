/**
 * 前端里程计node文件
*/

#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_localization/global_definition/global_definition.h"
#include "lidar_localization/mapping/front_end/front_end_flow.hpp"

using namespace lidar_localization;

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log"; // 日志输出目录
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "frone_end_node");
    ros::NodeHandle nh;

    std::shared_ptr<FrontEndFlow> _front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        _front_end_flow_ptr->Run();

        rate.sleep();
    }
    return 0; 
}