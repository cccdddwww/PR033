#include "costmap_ros.h"
#include "costmap/Point.h"
#include "costmap/pointmatrix.h"

#include "detect2.h"
#include "detect2_terminate.h"
#include "coder_array.h"

Costmap_ROS::Costmap_ROS(tf2_ros::Buffer& tf):
    tf_(tf),
    transform_tolerance_(0.3),
    stop_updates_(false),
    offset_(0.4),
    cnt_(0),
    image_trans_speed_(10),
    //x_size_(200),
    //y_size_(160)
    x_size_(200),
    y_size_(160)
{
    //Initialize old pose with something
    tf2::toMsg(tf2::Transform::getIdentity(), old_pose_.pose);

    ros::NodeHandle nh("~/costmap");
    //get global and robot base frame names
    nh.param("global_frame", global_frame_, std::string("odom_ekf"));//odom_ekf
    nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
    nh.param("lidar_frame", lidar_frame_, std::string("livox_frame"));

    ros::Time last_error = ros::Time::now();
    std::string tf_error;
    //we need to make sure that the transform
    // between the robot base frame and the global frame is available
    while(ros::ok() && !tf_.canTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(0.1), &tf_error)){
        ros::spinOnce();
        if (last_error + ros::Duration(5.0) < ros::Time::now())
    {
        ROS_WARN("Timed out waiting for transform from %s to %s to become available before running costmap, tf error: %s",
                robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
        last_error = ros::Time::now();
    }
    // The error string will accumulate and errors will typically be the same, so the last
    // will do for the warning above. Reset the string here to avoid accumulation.
    tf_error.clear();
    }

    costmap = new costmap_2d::Costmap2D(x_size_, y_size_, 0.01, 0, 0, 0);

    publisher_ = new costmap_2d::Costmap2DPublisher(&nh, costmap, robot_base_frame_, "costmap", true);

    globl_lidar_pub_ = nh.advertise<sensor_msgs::PointCloud2>("global_lidar", 64);

    matrix_pub_ = nh.advertise<costmap::Point>("/matrix",10);

    image_transport::ImageTransport it(nh);
    img_pub_ = it.advertise("depth_image", 10);

    sub_lidar_ = nh.subscribe("/livox/lidar", 1, &Costmap_ROS::pointCloud2Callback, this);
    ros::spin();
}

Costmap_ROS::~Costmap_ROS()
{ 
    if (publisher_ != NULL)
        delete publisher_;

    delete costmap;
}

void Costmap_ROS::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message){
    //std::cout << "callback" << std::endl;
    if(!stop_updates_){
        // get global pose
        geometry_msgs::PoseStamped pose;
        if (getLidarPose (pose)){
            //double yaw = tf2::getYaw(pose.pose.orientation);
            double new_origin_x = pose.pose.position.x + 0.9;
            double new_origin_y = pose.pose.position.y - costmap->getSizeInMetersY() / 2;
            costmap->updateOrigin(new_origin_x, new_origin_y);
            //std::cout << "costmap_new_origin: (" << new_origin_x << ", " << new_origin_y << ")" << std::endl;
            //std::cout << "pose: (" << pose.pose.position.x << ", " << pose.pose.position.y  - costmap->getSizeInMetersY() / 2<< ")" << std::endl;
            geometry_msgs::TransformStamped transformStamped;
            try{
                // transform the point cloud
                sensor_msgs::PointCloud2 global_frame_cloud;
                transformStamped = tf_.lookupTransform(global_frame_, lidar_frame_, ros::Time(0));
                tf2::doTransform(*message, global_frame_cloud, transformStamped);
                //tf_.transform(*message, global_frame_cloud, global_frame_);
                global_frame_cloud.header.stamp = message->header.stamp;
                globl_lidar_pub_.publish(global_frame_cloud);
                
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
                pcl::fromROSMsg(global_frame_cloud, *cloud);
                double px,py,pz;
                uint8_t cost_new;
                for(size_t i = 0; i < cloud->points.size(); ++i){
                    px = cloud->points[i].x;
                    py = cloud->points[i].y;
                    pz = cloud->points[i].z;
                    unsigned int mx, my;

                    if (!costmap->worldToMap(px, py, mx, my))
                    {
                    ROS_DEBUG("Computing map coords failed");
                    continue;
                    }  
                    
                    if(pz < -1*offset_) costmap->setCost(mx, my, 255);
                    else if(pz > offset_) costmap->setCost(mx, my, 255);
                    else {
                        cost_new = 1+ 253 * std::fabs(pz)/offset_;
                        costmap->setCost(mx, my, cost_new);
                    }
                } 

                if(cnt_ == image_trans_speed_ - 1){
                    cv::Mat depth_img(cv::Size(y_size_,x_size_),CV_8UC1); // 单通道
                    u_int32_t u,v;
                    for(size_t i = 0; i < x_size_*y_size_; i++){
                        costmap->indexToCells(i, u, v);
                        depth_img.at<uint8_t>(x_size_ - u - 1, y_size_ - v - 1) = costmap->getCost(u, v);
                    }
                    ros::Time time_img = ros::Time::now();
                    uint64_t img_time = time_img.toNSec();
                    std::string path = "/home/ubuntu/Project/img/";
                    path += std::to_string(img_time);
                    path += ".png";
                    cv::imwrite(path, depth_img);

                    coder::array<double, 2U> Result;
                    double t;
                    unsigned char uv[32000];

                    for (int i=0; i < 200; i++) {
                        for (int j=0; j < 160; j++) {
                        // Set the value of the array element.
                        // Change this value to the value that the application requires.
                        uv[i + 200 * j] = depth_img.at<uchar>(i,j);
                        }
                    }
                    // printf("size_a = %d\n", a.cols);
                    // printf("size_a = %d\n", a.rows);
                    // printf("size_a = %d\n", a.channels());
                    //ros::time start = ros::Time::now();
                    ros::Time start_time = ros::Time::now();
                    detect2(uv, Result, &t);
                    ros::Time end_time = ros::Time::now();
                    //ROS_INFO("start time is %ld", start_time.toNSec());
                    //ROS_INFO("end time is %ld", end_time.toNSec());
                    //double startt = 
                    ROS_INFO("cost time is %lf", end_time.toNSec()/1000000000.0 - start_time.toNSec()/1000000000.0);

                    //ros::time end = ros::Time::now();
                    //uint64_t time_detect = end.toNSec() - start.toNsec();
                    //ROS_INFO("detect time is ");
                    //printf("t = %lf\n",t); 

                    if(t != 1){
                        cv::cvtColor(depth_img,depth_img,CV_GRAY2BGR);
                        int size= *(Result.size());
                        //printf("size= %d\n",size); 

                        costmap::Point Point;
                        costmap::pointmatrix maxtrix;
                        
                        Point.header.stamp = time_img;
                        double obj_rx, obj_ry, obj_lx,obj_ly;
                        geometry_msgs::PointStamped right_point, left_point;
                        geometry_msgs::PointStamped new_right_point, new_left_point;
                        right_point.header.frame_id = global_frame_;
                        right_point.header.stamp = time_img;
                        left_point.header.frame_id = global_frame_;
                        left_point.header.stamp = time_img;
 
                        for (int i = 1; i < t; i++)
                        {
                            int lx= Result[i-1];
                            int ly= Result[i+size -1];
                            int w= Result[i+2*size -1];
                            int h= Result[i+3*size -1];
                            int rx = lx + w;
                            int ry = ly + h;
                            //printf("(%3d,%3d) (%3d,%3d)\n",lx,ly,rx,ry);
                            cv::rectangle(depth_img,cv::Point(lx,ly),cv::Point(rx,ry),cv::Scalar(0,0,255),2);
                            //ROS_INFO("lx = %d, ly = %d, rx =%d, ry =%d\n", lx, ly, rx, ry);
                            costmap->mapToWorld(x_size_ - ly - 1, y_size_ - lx -1, obj_lx, obj_ly);
                            if(costmap->getCost(x_size_ - ly - 1, y_size_ - lx -1) == 255) left_point.point.z = 0.5;
                            else if(costmap->getCost(x_size_ - ly - 1, y_size_ - lx -1) == 0) left_point.point.z = 0;
                            else left_point.point.z = (costmap->getCost(x_size_ - ly - 1, y_size_ - lx -1) - 1)*offset_/253;
                            costmap->mapToWorld(x_size_ - ry - 1, y_size_ - rx - 1, obj_rx, obj_ry);
                            if(costmap->getCost(x_size_ - ry - 1, y_size_ - rx - 1) == 255) right_point.point.z = 0.5;
                            else if(costmap->getCost(x_size_ - ry - 1, y_size_ - rx - 1) == 0) right_point.point.z = 0;
                            else right_point.point.z = (costmap->getCost(x_size_ - ry - 1, y_size_ - rx - 1) - 1)*offset_/253;
                            right_point.point.x = obj_rx;
                            right_point.point.y = obj_ry;
                            left_point.point.x = obj_lx;
                            left_point.point.y = obj_ly;

                            //ROS_INFO("right_point = (%lf, %lf, %lf)\n", obj_rx, obj_ry, right_point.point.z);
                            //ROS_INFO("left_point = (%lf, %lf, %lf)\n", obj_lx, obj_ly, left_point.point.z);

                            

                            //right_point = tf_.transform(right_point, lidar_frame_, ros::Duration(0));
                            //left_point = tf_.transform(left_point, lidar_frame_, ros::Duration(0));
                            tf_.transform(right_point, new_right_point, lidar_frame_, ros::Duration(0));
                            tf_.transform(left_point, new_left_point, lidar_frame_, ros::Duration(0));

                            //ROS_INFO("new_right_point = (%lf, %lf, %lf)\n", new_right_point.point.x, new_right_point.point.y , new_right_point.point.z);
                            //ROS_INFO("new_left_point = (%lf, %lf, %lf)\n", new_left_point.point.x, new_left_point.point.y , new_left_point.point.z);

                            maxtrix.p1 = new_left_point.point;
                            maxtrix.p2 = new_right_point.point;
                            Point.points.push_back(maxtrix);
                        }
                        matrix_pub_.publish(Point);
                        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", depth_img).toImageMsg();
                        img_pub_.publish(img_msg);
                        
                    }
                    else{
                        costmap::Point Point;
                        costmap::pointmatrix maxtrix;
                        Point.header.stamp = time_img;
                        maxtrix.p1.x = 3.0;
                        maxtrix.p1.y = -5.0;
                        maxtrix.p1.z = 5.0;
                        maxtrix.p2.x = 3.0;
                        maxtrix.p2.y = -4.9;
                        maxtrix.p2.z = 5.1;
                        Point.points.push_back(maxtrix);
                        matrix_pub_.publish(Point);
                        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", depth_img).toImageMsg();
                        img_pub_.publish(img_msg);
                    }
                    //ROS_INFO("publish img");
                    cnt_ = 0;
                    
                }
            
            
                //publisher_->updateBounds(0, 100, 0, 100);
                publisher_->publishCostmap();
                //std::cout << "callback4" << std::endl;
                cnt_++;

            }

            catch (tf2::TransformException& ex){
                ROS_ERROR("TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s", lidar_frame_.c_str(),
                        message->header.frame_id.c_str(), ex.what());
            }
        }
    }
    
}

bool Costmap_ROS::getLidarPose(geometry_msgs::PoseStamped& global_pose) const
{
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped lidar_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), lidar_pose.pose);
    lidar_pose.header.frame_id = lidar_frame_;
    lidar_pose.header.stamp = ros::Time();
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get the global pose of the lidar
    try
    {
        // use current time if possible (makes sure it's not in the future)
        if (tf_.canTransform(global_frame_, lidar_frame_, current_time))
        {
        geometry_msgs::TransformStamped transform = tf_.lookupTransform(global_frame_, lidar_frame_, current_time);
        tf2::doTransform(lidar_pose, global_pose, transform);
        }
        // use the latest otherwise
        else
        {
        tf_.transform(lidar_pose, global_pose, global_frame_);
        }
    }
    catch (tf2::LookupException& ex)
    {
        ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up lidar pose: %s\n", ex.what());
        return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
        ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up lidar pose: %s\n", ex.what());
        return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
        ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up lidar pose: %s\n", ex.what());
        return false;
    }
    // check global_pose timeout
    if (current_time.toSec() - global_pose.header.stamp.toSec() > transform_tolerance_)
    {
        ROS_WARN_THROTTLE(1.0,
                                "Costmap transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                                current_time.toSec(), global_pose.header.stamp.toSec(), transform_tolerance_);
        return false;
    }
    return true;
}
