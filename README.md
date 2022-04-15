# PR033
costmap文件夹是核心功能模块，实现小车运动时累计多帧激光雷达点云并转换成深度图像和检测深度图像上的异常，接口在costmap/src/costmap_ros.cpp。

hunter_ros是小车驱动模块，负责接收hunter的里程计。

livox_ros_driver是激光雷达驱动模块，负责发布点云数据。

imu是imu驱动模块，负责发布加速度和角速度数据。

robot_pose_ekf是拓展卡尔曼滤波模块，负责融合imu和hunter里程计。

livox_cloud_undistortion是点云运动去畸变模块。

pcl_image是上位机负责在可见光图像上框出深度图像上对应目标的模块。
