#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver/CustomMsg.h"

typedef pcl::PointXYZINormal PointType;

ros::Publisher pub_pcl_out;

void LivoxMsgCbk1(const livox_ros_driver::CustomMsgConstPtr& livox_msg) {

  pcl::PointCloud<PointType> pcl_in;
 
  auto time_end = livox_msg->points.back().offset_time;
  for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
    PointType pt;
    pt.x = livox_msg->points[i].x;
    pt.y = livox_msg->points[i].y;
    pt.z = livox_msg->points[i].z;
    float s = livox_msg->points[i].offset_time / (float)time_end;
    pt.intensity = livox_msg->points[i].line + s*0.1; // The integer part is line number and the decimal part is timestamp
    pt.curvature = livox_msg->points[i].reflectivity * 0.1;
    pcl_in.push_back(pt);
  }

  /// timebase 5ms ~ 50000000, so 10 ~ 1ns

  //unsigned long timebase_ns = livox_msg->timebase;
  //std::cout << "timebase_ns: " << timebase_ns << std::endl;

  sensor_msgs::PointCloud2 pcl_ros_msg;
  pcl::toROSMsg(pcl_in, pcl_ros_msg);
  pcl_ros_msg.header.stamp.fromNSec(livox_msg->timebase);
  pcl_ros_msg.header.frame_id = "livox";
  pub_pcl_out.publish(pcl_ros_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_repub_node");
  ros::NodeHandle nh;

  ROS_INFO("start livox_repub");

  ros::Subscriber sub_livox_msg1 = nh.subscribe<livox_ros_driver::CustomMsg>(
      "/livox/lidar", 100, LivoxMsgCbk1);
  pub_pcl_out = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_repub", 100);

  ros::spin();
}