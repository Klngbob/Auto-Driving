#include "lidar_localization/sensor_data/gnss_data.hpp"

#include "glog/logging.h"

// 静态成员变量必须在类外初始化!!!
bool lidar_localization::GNSSData::origin_position_ininted = false;
GeographicLib::LocalCartesian lidar_localization::GNSSData::geo_converter;

namespace lidar_localization {

void GNSSData::InitOriginPosition() {
    geo_converter.Reset(this->latitude, this->longitude, this->altitude); // 经纬度初始化原点
    this->origin_position_ininted = true; // 完成初始化
}

// 经纬度转ENU坐标系
void GNSSData::UpdateXYZ() {
    if(!origin_position_ininted) {
        LOG(WARNING) << "GeoConverter has not set origin position";
    }
    
    geo_converter.Forward(this->latitude, this->longitude, this->altitude, this->local_E, this->local_N, this->local_U);
}

}