#pragma once

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
class PclFilterCore
{

  private:
    ros::Subscriber sub_point_cloud_;

    ros::Publisher pub_ror_;
    ros::Publisher pub_vg_;
    ros::Publisher pub_filtered_;
    ros::Publisher pub_;

    void lidarCbk(const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr);
    void radiusOutlierRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, 
                                const pcl::PointCloud<pcl::PointXYZI>::Ptr output);
    void voxelGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, 
                                const pcl::PointCloud<pcl::PointXYZI>::Ptr output);

  public:
    double clip_height = 0.3;
    PclFilterCore(ros::NodeHandle &nh);
    ~PclFilterCore();
    void Spin();
    double tolerance_;
    double min_height_far_=-0.7, min_height_close_=-0.66, x_distance=10, x_distort_distance=10, max_height_=0.3, angle_min_=-0.615, angle_max_=0.615;
    //double angle_min_=-0.61, angle_max_=0.325;-0.25
    double angle_increment_=0.003, scan_time_=0.1, range_min_=1, range_max_=100;
    bool use_inf_=true;
    double inf_epsilon_=1;
   // -1.2217305
   //0.03
};

 
