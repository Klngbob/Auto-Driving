/**
 * 显示模块的node文件
*/
#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_localization/global_definition/global_definition.h"
#include <lidar_localization/saveMap.h>
#include "lidar_localization/mapping/viewer/viewer_flow.hpp"

using namespace lidar_localization;

std::shared_ptr<ViewerFlow> viewer_flow_ptr;
bool _need_save_map = false;

bool save_map_callback(saveMap::Request& request, saveMap::Response& response) {
    _need_save_map = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log"; // 日志输出目录
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "viewer_node");
    ros::NodeHandle nh;

    viewer_flow_ptr = std::make_shared<ViewerFlow>(nh);
    ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);

    ros::Rate rate(100);
    while(ros::ok()) {
        ros::spinOnce();

        viewer_flow_ptr->Run();
        if(_need_save_map) {
            _need_save_map = false;
            viewer_flow_ptr->SaveMap();
        }

        rate.sleep();
    }

    return 0;
}