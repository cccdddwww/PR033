#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include"costmap.h"
#include"costmap_2d_publisher.h"


class Costmap_ROS
{
private:

    void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message);
    bool getLidarPose(geometry_msgs::PoseStamped& global_pose) const; //< @brief get the global pose of the lidar

    geometry_msgs::PoseStamped old_pose_;
    ros::Subscriber sub_lidar_;
    ros::Publisher globl_lidar_pub_;
    ros::Publisher matrix_pub_;
    image_transport::Publisher img_pub_;
    std::string global_frame_;  //< @brief The global frame for the costmap
    std::string robot_base_frame_;  //< @brief The frame_id of the robot base
    std::string lidar_frame_;  //< @brief The frame_id of the lidar frame

    tf2_ros::Buffer& tf_;  //< @brief Used for transforming point clouds
    double transform_tolerance_;  //< @brief timeout before transform errors
    bool stop_updates_;
    double offset_;//< @brief (-a,a) to (0, 2a)
    int cnt_;//< @brief count for conversion
    int image_trans_speed_;//< @brief how many pointcloud frames convert to one image
    size_t x_size_;//< @brief map size x
    size_t y_size_;//< @brief map size y

    costmap_2d::Costmap2DPublisher* publisher_;
    unsigned char* costmap_;

public:
    Costmap_ROS(tf2_ros::Buffer& tf);
    ~Costmap_ROS();

    costmap_2d::Costmap2D* costmap;
};
