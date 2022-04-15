#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "pcl_image/Point.h"
#include "pcl_image/pointmatrix.h"
using namespace std;
using namespace cv;
using namespace message_filters;

float theoryUV[2]={0, 0};
vector<float> intrinsic{1257.971355837851,0,675.0608944447513,0,1259.185163234034,329.6319412921005,0,0,1};
vector<float> distortion{-0.066483467712754,-0.062067208747137,0,0,0};
vector<float> extrinsic{-0.0579608,-0.998266,0.01026,0.0704277,0.448227,-0.0352051,-0.893226,-0.0392181,0.892039,-0.0471733,0.44949,-0.18914,0,0,0,1};
cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
vector<pcl_image::pointmatrixConstPtr> p;
cv_bridge::CvImagePtr cv_ptr;
cv::Mat src_img;
ros::Publisher img_pub_;


void getTheoreticalUV(float* theoryUV, const vector<float> &intrinsic, const vector<float> &extrinsic,const vector<float> &cordinate,int res[4]){
    //theoryUV[2] = 
    int U1,V1,U2,V2;
    // set the intrinsic and extrinsic matrix
    double matrix1[3][3] = {{intrinsic[0], intrinsic[1], intrinsic[2]}, {intrinsic[3], intrinsic[4], intrinsic[5]}, {intrinsic[6], intrinsic[7], intrinsic[8]}}; 
    double matrix2[3][4] = {{extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3]}, {extrinsic[4], extrinsic[5], extrinsic[6], extrinsic[7]}, {extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11]}};
    double matrix3[4][2] = {{cordinate[0], cordinate[3]},{cordinate[1],cordinate[4]},{cordinate[2],cordinate[5]}, {1, 1}};
    
    
    // transform into the opencv matrix
    cv::Mat matrixIn(3, 3, CV_64F, matrix1);
    cv::Mat matrixOut(3, 4, CV_64F, matrix2);
    cv::Mat coordinate(4, 2, CV_64F, matrix3);
 
    
    // calculate the result of u and v
    cv::Mat result = matrixIn*matrixOut*coordinate;
    float u1 = result.at<double>(0, 0);
    float v1 = result.at<double>(1, 0);
    float depth1 = result.at<double>(2, 0);

    float u2 = result.at<double>(0, 1);
    float v2 = result.at<double>(1, 1);
    float depth2 = result.at<double>(2, 1);

    theoryUV[0] = u1 / depth1;
    theoryUV[1] = v1 / depth1;
    
    U1 = floor(theoryUV[0] + 0.5);
    V1 = floor(theoryUV[1] + 0.5);
    
    theoryUV[0] = u2 / depth2;
    theoryUV[1] = v2 / depth2;
    
    U2 = floor(theoryUV[0] + 0.5);
    V2 = floor(theoryUV[1] + 0.5);

    res[0]=U1;
    res[1]=V1;

    res[2]=U2;
    res[3]=V2;
    //printf(" %d,%d,%d,%d\n",res[0],res[1],res[2],res[3]);

 

}

void callback(const sensor_msgs::ImageConstPtr& image_msg, const pcl_image::PointConstPtr& point_msg){
    ROS_INFO("image_time :%lu\n",image_msg->header.stamp.toNSec());
    ROS_INFO("matrix_time:%lu\n",point_msg->header.stamp.toNSec());

    cv_ptr=cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

    src_img=cv_ptr->image;

    vector<float> coord;
    int result[4];

    cv::Scalar colorRectangle1(0, 0, 255);
    int thicknessRectangle1 = 2;
    cv::Mat view, rview, map1, map2;
    cv::Size imageSize = src_img.size();
    cv::Point p3, p4;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
    cv::remap(src_img, src_img, map1, map2, cv::INTER_LINEAR);
    for(int i = 0; i < point_msg->points.size(); i++){
        coord.push_back(point_msg->points[i].p1.x);
        coord.push_back(point_msg->points[i].p1.y);   
        coord.push_back(point_msg->points[i].p1.z);   
        coord.push_back(point_msg->points[i].p2.x);   
        coord.push_back(point_msg->points[i].p2.y);   
        coord.push_back(point_msg->points[i].p2.z);

        getTheoreticalUV(theoryUV, intrinsic, extrinsic, coord,result);
        coord.clear();
        cv::Point p3(result[0]*800.0/1280-10, result[1]*600.0/960-10), p4(result[2]*800.0/1280+10, result[3]*600.0/960+10);
        printf("p1.x =%f,p1.y =%f,p1.z =%f\n",point_msg->points[i].p1.x,point_msg->points[i].p1.y,point_msg->points[i].p1.z);
        printf("p2.x =%f,p2.y =%f,p2.z =%f\n",point_msg->points[i].p2.x,point_msg->points[i].p2.y,point_msg->points[i].p2.z);
        printf("rx =%d,ry =%d,lx =%d,ly =%d\n",result[0],result[1],result[2],result[3]);
        //cv::Point p3(200, 300), p4(100, 150);
        
        cv::rectangle(src_img,p3,p4,colorRectangle1,thicknessRectangle1); 
        
    };
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", src_img).toImageMsg();
    
    img_pub_.publish(img_msg);
    
}

int main(int argc,char** argv){
    ros::init(argc,argv,"main");
    ros::NodeHandle n;
    // set intrinsic parameters of the camera
    cameraMatrix.at<double>(0, 0) = intrinsic[0];
    cameraMatrix.at<double>(0, 2) = intrinsic[2];
    cameraMatrix.at<double>(1, 1) = intrinsic[4];
    cameraMatrix.at<double>(1, 2) = intrinsic[5];

    // set radial distortion and tangential distortion
    distCoeffs.at<double>(0, 0) = distortion[0];
    distCoeffs.at<double>(1, 0) = distortion[1];
    distCoeffs.at<double>(2, 0) = distortion[2];
    distCoeffs.at<double>(3, 0) = distortion[3];
    distCoeffs.at<double>(4, 0) = distortion[4]; 
    img_pub_ = n.advertise<sensor_msgs::Image>("/image1", 10);
    message_filters::Subscriber<sensor_msgs::Image> sub_image_(n, "/usb_cam/image_raw", 1);
    message_filters::Subscriber<pcl_image::Point> sub_matrix_(n, "/matrix", 1); 
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, pcl_image::Point> MySyncPolicy;
    //message_filters::TimeSynchronizer<sensor_msgs::Image, pcl_image::Point> sync(sub_image_, sub_matrix_, 300); 
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_image_, sub_matrix_); //queue size=10

    sync.registerCallback(boost::bind(&callback, _1, _2));
    
    ros::spin();
    return 0;  
}
/*void projection(float x,float y,float z,int &u,int &v){
        u = floor(theoryUV[0] + 0.5);
        v = floor(theoryUV[1] + 0.5);   
}*/