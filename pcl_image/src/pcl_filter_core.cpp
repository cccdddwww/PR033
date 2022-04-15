#include "pcl_filter_core.h"

PclFilterCore::PclFilterCore(ros::NodeHandle &nh){
    sub_point_cloud_ = nh.subscribe("/livox_origin",10, &PclFilterCore::lidarCbk, this);

    //pub_ror_ = nh.advertise<sensor_msgs::PointCloud2>("/ror", 10);
    //pub_vg_ = nh.advertise<sensor_msgs::PointCloud2>("/vg", 10);
    pub_filtered_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered", 10);
    pub_ = nh.advertise<sensor_msgs::LaserScan>("/scan_0", 10);
    ros::spin();
}

PclFilterCore::~PclFilterCore(){}

void PclFilterCore::Spin(){
    
}

void PclFilterCore::lidarCbk(const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr){
  //ROS_INFO("got cloud_msg!");
  /*pcl::PointCloud<pcl::PointXYZ>::Ptr init_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ror_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr vg_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr3(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pc_ptr11(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*in_cloud_ptr, *init_pc_ptr);

  double begin = ros::Time::now().toSec();
  //voxelGrid(init_pc_ptr, vg_ptr);
  pcl::VoxelGrid<pcl::PointXYZ> vgfilter;//对点云稀疏化,降采样
  vgfilter.setInputCloud(init_pc_ptr);
  vgfilter.setLeafSize(0.1f, 0.1f, 0.1f);//0.1m*0.1m*0.1m
  vgfilter.filter(*vg_ptr);
  double voxel = ros::Time::now().toSec();
  //radiusOutlierRemoval(vg_ptr, ror_ptr);
  //double radius = ros::Time::now().toSec();
  
  ///pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond11(new pcl::ConditionAnd<pcl::PointXYZ>());   //创建条件定义对象
  //range_cond11->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, -0.5)));
  //range_cond11->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.2)));

  pcl::ConditionOr<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionOr<pcl::PointXYZ>());   //创建条件定义对象
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, -0.5)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, 1.0)));

  pcl::ConditionOr<pcl::PointXYZ>::Ptr range_cond2(new pcl::ConditionOr<pcl::PointXYZ>());
  //range_cond2->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -0.25)));
  range_cond2->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, 3.0)));
  range_cond2->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, -0.4)));
  pcl::ConditionOr<pcl::PointXYZ>::Ptr range_cond3(new pcl::ConditionOr<pcl::PointXYZ>());
  //range_cond3->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 0.25)));
  range_cond3->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, -0.4)));
  range_cond3->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, 3.0)));
  //创建滤波器并用条件定义对象初始化
  //pcl::ConditionalRemoval<pcl::PointXYZ> condrem11;
  pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
  pcl::ConditionalRemoval<pcl::PointXYZ> condrem2;
  pcl::ConditionalRemoval<pcl::PointXYZ> condrem3;
  condrem2.setInputCloud(vg_ptr);
  condrem2.setCondition(range_cond2);
  condrem2.setKeepOrganized(false);
  condrem2.filter(*filtered_pc_ptr2);
  condrem3.setCondition(range_cond3);
  condrem3.setInputCloud(filtered_pc_ptr2);
  condrem3.setKeepOrganized(false);
  condrem3.filter(*filtered_pc_ptr3);
  condrem.setCondition(range_cond);
  condrem.setInputCloud(filtered_pc_ptr3);
  condrem.setKeepOrganized(false);
  condrem.filter(*filtered_pc_ptr);
  //condrem11.setCondition(range_cond11);
  //condrem11.setInputCloud(filtered_pc_ptr);
  //condrem11.setKeepOrganized(false);
  //condrem11.filter(*filtered_pc_ptr11);

  //std::cout << "cloud_after_Condition后点云数据点数：" << white_ground_after_Condition->points.size() + red_unground_after_Condition->points.size() << std::endl;
  double cond = ros::Time::now().toSec();
  //ROS_INFO("voxel time = %f", voxel-begin);
  //ROS_INFO("radius time = %f", radius-voxel);
  //ROS_INFO("cond time = %f", cond-voxel);
  //sensor_msgs::PointCloud2Ptr cloud_msg;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*filtered_pc_ptr, cloud_msg);*/
  //pcl_conversions::moveFromPCL(*filtered_pc_ptr, *cloud_msg);
  double begin = ros::Time::now().toSec();
  sensor_msgs::PointCloud2 cloud_msg;
  cloud_msg.header = in_cloud_ptr->header;
  // build laserscan output  
  sensor_msgs::LaserScan output;
  //sensor_msgs::PointCloud2Ptr *cloud_msg;
  //cloud_msg = cloud_msgg.makeShared();
  output.header = cloud_msg.header;
  output.angle_min = angle_min_;
  output.angle_max = angle_max_;
  output.angle_increment = angle_increment_;
  output.time_increment = 0.0;
  output.scan_time = scan_time_;
  output.range_min = range_min_;
  output.range_max = range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_)
  {
    //把inf赋给所有点
    output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  }
  else
  {
    output.ranges.assign(ranges_size, output.range_max + inf_epsilon_);
  }

  //sensor_msgs::PointCloud2ConstPtr cloud_out;
  sensor_msgs::PointCloud2Ptr cloud;
  //cloud_out = cloud_msg;
  //ROS_INFO("1111");
  // Iterate through pointcloud

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*in_cloud_ptr, "x"), iter_y(*in_cloud_ptr, "y"),
       iter_z(*in_cloud_ptr, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    //ROS_INFO("converting!");
    //ROS_INFO("point(%f, %f, %f, %f, %d, %d)\n", *iter_x, *iter_y, *iter_z, *iter_intensity, *iter_tag, *iter_line);
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
    {
      //NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
      //ROS_INFO("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
      continue;
    }

    if(*iter_x < x_distort_distance){
      if (*iter_z > max_height_ || *iter_z < min_height_close_)
      {
        //NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
        continue;
      }
    }

    if(*iter_x >= x_distort_distance){
      if (*iter_z > max_height_ || *iter_z < min_height_far_)
      {
        //NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
        continue;
      }
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_)
    {
      //NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_)
    {
      //NODELET_DEBUG("rejected for range %f above maximum value %f. Point: (%f, %f, %f)", range, range_max_, *iter_x, *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < output.angle_min || angle > output.angle_max)
    {
      //NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    int index = (angle - output.angle_min) / output.angle_increment;
    if (range < output.ranges[index])
    {
      output.ranges[index] = range;
    }
  }
  for(size_t i = 1; i < output.ranges.size()-1; i++){
    //if(!std::isinf(output.ranges[i-1]) && std::isinf(output.ranges[i]) && !std::isinf(output.ranges[i+1])){
    //  output.ranges[i] = (output.ranges[i-1] + output.ranges[i+1]) / 2;
    //  continue;
    //}
    if(fabs(output.ranges[i] - output.ranges[i-1])>1.0 && fabs(output.ranges[i] - output.ranges[i+1])>1.0){
      output.ranges[i] = std::numeric_limits<double>::infinity();
    }
  }
  double end = ros::Time::now().toSec();
  //ROS_INFO("trans time = %f", end - begin);
  pub_.publish(output);

  /*sensor_msgs::PointCloud2 pub_pc;
  pcl::toROSMsg(*filtered_pc_ptr, pub_pc);
  pub_pc.header = in_cloud_ptr->header;
  pub_filtered_.publish(pub_pc);*/
    /*pcl::PointCloud<pcl::PointXYZI>::Ptr init_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ror_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr vg_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr2(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr3(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*in_cloud_ptr, *init_pc_ptr);

    double begin = ros::Time::now().toSec();
    voxelGrid(init_pc_ptr, vg_ptr);
    double voxel = ros::Time::now().toSec();
    //radiusOutlierRemoval(vg_ptr, ror_ptr);
    double radius = ros::Time::now().toSec();
    
    pcl::ConditionOr<pcl::PointXYZI>::Ptr range_cond(new pcl::ConditionOr<pcl::PointXYZI>());   //创建条件定义对象
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("z", pcl::ComparisonOps::GT, -0.4)));
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, 1.0)));

    pcl::ConditionOr<pcl::PointXYZI>::Ptr range_cond2(new pcl::ConditionOr<pcl::PointXYZI>());
    range_cond2->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT, -0.3)));
    range_cond2->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, 2.5)));
    
    pcl::ConditionOr<pcl::PointXYZI>::Ptr range_cond3(new pcl::ConditionOr<pcl::PointXYZI>());
    range_cond3->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT, 0.3)));
    range_cond3->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, 2.5)));
    //创建滤波器并用条件定义对象初始化
    pcl::ConditionalRemoval<pcl::PointXYZI> condrem;
    pcl::ConditionalRemoval<pcl::PointXYZI> condrem2;
    pcl::ConditionalRemoval<pcl::PointXYZI> condrem3;
    condrem2.setInputCloud(vg_ptr);
    condrem2.setCondition(range_cond2);
    condrem2.setKeepOrganized(false);
    condrem2.filter(*filtered_pc_ptr2);
    condrem3.setCondition(range_cond3);
    condrem3.setInputCloud(filtered_pc_ptr2);
    condrem3.setKeepOrganized(false);
    condrem3.filter(*filtered_pc_ptr3);
    condrem.setCondition(range_cond);
    condrem.setInputCloud(filtered_pc_ptr3);
    condrem.setKeepOrganized(false);
    condrem.filter(*filtered_pc_ptr);
    //std::cout << "cloud_after_Condition后点云数据点数：" << white_ground_after_Condition->points.size() + red_unground_after_Condition->points.size() << std::endl;
    double cond = ros::Time::now().toSec();
    ROS_INFO("hello");
    ROS_INFO("voxel time = %f", voxel-begin);
    ROS_INFO("radius time = %f", radius-voxel);
    ROS_INFO("cond time = %f", cond-radius);

    sensor_msgs::PointCloud2 pub_ror;
    sensor_msgs::PointCloud2 pub_vg;
    sensor_msgs::PointCloud2 pub_pc;
    pcl::toROSMsg(*ror_ptr, pub_ror);
    pcl::toROSMsg(*vg_ptr, pub_vg);
    pcl::toROSMsg(*filtered_pc_ptr, pub_pc);

    pub_ror.header = in_cloud_ptr->header;
    pub_vg.header = in_cloud_ptr->header;
    pub_pc.header = in_cloud_ptr->header;

    pub_ror_.publish(pub_ror);
    pub_vg_.publish(pub_vg);
    pub_filtered_.publish(pub_pc);*/
}

void PclFilterCore::radiusOutlierRemoval(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, 
                                const pcl::PointCloud<pcl::PointXYZI>::Ptr output)
{
    //创建滤波器
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> rorfilter;
    //输入点云
    rorfilter.setInputCloud(input);
    //在0.1m为半径的范围内搜寻邻居点
    rorfilter.setRadiusSearch(0.1);
    //邻居少于2个的点认为是离群点
    rorfilter.setMinNeighborsInRadius (2);
    //获取滤波结果
    rorfilter.filter(*output);
}

void PclFilterCore::voxelGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr input, 
                                const pcl::PointCloud<pcl::PointXYZI>::Ptr output)
{
    pcl::VoxelGrid<pcl::PointXYZI> vgfilter;//对点云稀疏化,降采样

    vgfilter.setInputCloud(input);
    vgfilter.setLeafSize(0.1f, 0.1f, 0.1f);//0.1m*0.1m*0.1m
    vgfilter.filter(*output);
}
