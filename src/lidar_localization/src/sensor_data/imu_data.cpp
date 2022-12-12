#include "lidar_localization/sensor_data/imu_data.hpp"

#include<cmath>
#include "glog/logging.h"

namespace lidar_localization {
Eigen::Matrix3f IMUData::GetOrientationMatrix() {
    Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
    Eigen::Matrix3f matrix = q.matrix().cast<float>();
            
    return matrix;
}

bool IMUData::SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time) {
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
    }
    if(UnsyncedData.size() < 2)
        return false;
    
    IMUData front_data = UnsyncedData.at(0);
    IMUData back_data = UnsyncedData.at(1);
    IMUData synced_data;
    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
    synced_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
    synced_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
    synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
    synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
    synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
    // 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
    // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
    synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
    synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
    synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
    synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
    // 线性插值之后要归一化
    synced_data.orientation.Normalize();
    SyncedData.push_back(synced_data);
    return true;
}

}