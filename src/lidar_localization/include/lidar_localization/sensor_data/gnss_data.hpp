#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_

#include <vector>
#include <string>
#include <deque>

#include "Geocentric/LocalCartesian.hpp"

using std::vector;
using std::string;

namespace lidar_localization {
class GNSSData {
    public:
        double time = 0.0;
        double longitude = 0.0;
        double latitude = 0.0;
        double altitude = 0.0;
        double local_E = 0.0; // 东
        double local_N = 0.0; // 北
        double local_U = 0.0; // 天 坐标系，即笛卡尔坐标系
        int status = 0; // 卫星定位状态消息
        int service = 0; // 使用哪个全球卫星导航系统，比如GPS、北斗
    
    private:
        static GeographicLib::LocalCartesian geo_converter; // 局部坐标系转换
        static bool origin_position_ininted;
    
    public:
        void InitOriginPosition();
        void UpdateXYZ();
        static bool SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time);
};
}

#endif