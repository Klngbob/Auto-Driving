/*
* 在ros中发布点云
*/

#include "lidar_localization/publisher/cloud_publisher.hpp"

namespace lidar_localization {
CloudPublisher::CloudPublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               std::string frame_id,
                               size_t buff_size)
    : nh_(nh), frame_id_(frame_id) {
        this->publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
    }

void CloudPublisher::Publish(CloudData::CLOUD_PTR cloud_ptr_input) {
    ros::Time time = ros::Time::now();
    this->PublishData(cloud_ptr_input, time);
}

void CloudPublisher::Publish(CloudData::CLOUD_PTR cloud_ptr_input, double time) {
    ros::Time ros_time((float) time);
    this->PublishData(cloud_ptr_input, ros_time);
}

void CloudPublisher::PublishData(CloudData::CLOUD_PTR& cloud_ptr_input, ros::Time time) {
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    printf("toROSMsg...\n");
    printf("点云宽度width: %u", cloud_ptr_input->width);
    printf("点云高度height: %u", cloud_ptr_input->height);
    printf("点云size: %u", cloud_ptr_input->size());
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output); // 转换成ros中的点云数据格式
    printf("toROSMsg done\n");
    cloud_ptr_output->header.stamp = time;
    cloud_ptr_output->header.frame_id = frame_id_;
    publisher_.publish(*cloud_ptr_output);
}

bool CloudPublisher::HasSubscribers() {
    return this->publisher_.getNumSubscribers() != 0;
}
}