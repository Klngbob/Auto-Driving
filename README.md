# Auto-Driving

## test_frame_node
接收数据集中激光雷达、IMU、GNSS数据，将其转存至相应类型的deque容器内，再经过处理、坐标变换，通过`/current_scan`发布点云数据，通过`/lidar_odom`发布雷达和`map`坐标转换。

## front_end_node
激光雷达前端里程计，