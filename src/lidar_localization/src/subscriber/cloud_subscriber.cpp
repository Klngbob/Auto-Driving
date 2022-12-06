/*
* 订阅激光点云信息，并解析数据
*/
#include "lidar_localization/subscriber/cloud_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization {
CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
        // 第四个参数表示调用this里的callback函数
        this->subscriber_ = this->nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
    }

// 每次调用回调函数，将接收到的点云数据插入到点云订阅者类中定义的缓冲区
void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    CloudData cloud_data;
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));

    this->new_cloud_data_.push_back(cloud_data);
}

// 将订阅到的点云缓冲区数据都拼接起来
void CloudSubscriber::ParseData(std::deque<CloudData>& deque_cloud_data) {
    if(this->new_cloud_data_.size() > 0) {
        deque_cloud_data.insert(deque_cloud_data.end(), this->new_cloud_data_.begin(), this->new_cloud_data_.end());
        this->new_cloud_data_.clear();
    }
}
}