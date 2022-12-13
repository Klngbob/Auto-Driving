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

// GNSS传感器时间同步
bool GNSSData::SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time) {
    // 传感器数据按时间序排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意：如果左右两个数据有里同步时间差值比较大的，则说明数据有丢失，时间离得太远不适合做插值
    while(UnsyncedData.size() >= 2) {
        if(UnsyncedData.front().time > sync_time) // 如果第一个数据比同步数据更后则直接退出，无法插值
            return false;
        if(UnsyncedData.at(1).time < sync_time) { // 如果第二个数据时间也在同步时间之前，则舍弃第一个数据
            UnsyncedData.pop_front();
            continue;
        }
        if(sync_time - UnsyncedData.front().time > 0.2) { // 如果同步时间离第一个数据太远了则数据丢失，
            UnsyncedData.pop_front();
            break;
        }
        if(UnsyncedData.at(1).time - sync_time > 0.2) {
            UnsyncedData.pop_front();
            break;
        }
        break;
    }
    if(UnsyncedData.size() < 2)
        return false;
    
    GNSSData front_data = UnsyncedData.at(0);
    GNSSData back_data = UnsyncedData.at(1);
    GNSSData synced_data;
    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.status = back_data.status;
    synced_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
    synced_data.latitude = front_data.latitude * front_scale + back_data.latitude * back_scale;
    synced_data.altitude = front_data.altitude * front_scale + back_data.altitude * back_scale;

    synced_data.local_E = front_data.local_E * front_scale + back_data.local_E * back_scale;
    synced_data.local_N = front_data.local_N * front_scale + back_data.local_N * back_scale;
    synced_data.local_U = front_data.local_U * front_scale + back_data.local_U * back_scale;

    SyncedData.push_back(synced_data);
    
    return true;
}
}