#include <iostream>
#include <vector>
#include <ros/ros.h>
#include<sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace std;
using namespace cv;



float theoryUV[2];
void projection(float x,float y,float z,int &u,int &v);
void callback(const sensor_msgs::ImageConstPtr &msg);
cv_bridge::CvImagePtr cv_ptr;
cv::Mat src_img;
void getTheoreticalUV(float* theoryUV, const vector<float> &intrinsic, const vector<float> &extrinsic, double x, double y, double z);
int main(int argc,char** argv){
    ros::init(argc,argv,"main");
    ros::NodeHandle n;
    ros::Subscriber sub=n.subscribe("usb_cam/image_raw",1,callback);

    ros::spin();
    return 0;
      
}
/*void projection(float x,float y,float z,int &u,int &v){
        u = floor(theoryUV[0] + 0.5);
        v = floor(theoryUV[1] + 0.5);   
}*/
void getTheoreticalUV(float* theoryUV, const vector<float> &intrinsic, const vector<float> &extrinsic, double x, double y, double z) {
    
    float theoryUV[2] = {0, 0};

    // set the intrinsic and extrinsic matrix
    double matrix1[3][3] = {{intrinsic[0], intrinsic[1], intrinsic[2]}, {intrinsic[3], intrinsic[4], intrinsic[5]}, {intrinsic[6], intrinsic[7], intrinsic[8]}}; 
    double matrix2[3][4] = {{extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3]}, {extrinsic[4], extrinsic[5], extrinsic[6], extrinsic[7]}, {extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11]}};
    double matrix3[4][1] = {x, y, z, 1};
    
    // transform into the opencv matrix
    cv::Mat matrixIn(3, 3, CV_64F, matrix1);
    cv::Mat matrixOut(3, 4, CV_64F, matrix2);
    cv::Mat coordinate(4, 1, CV_64F, matrix3);
    
    // calculate the result of u and v
    cv::Mat result = matrixIn*matrixOut*coordinate;
    float u = result.at<double>(0, 0);
    float v = result.at<double>(1, 0);
    float depth = result.at<double>(2, 0);

    theoryUV[0] = u / depth;
    theoryUV[1] = v / depth;
}
void callback(const sensor_msgs::ImageConstPtr &msg){
    cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    src_img=cv_ptr->image;
    vector<float> intrinsic{1257.971355837851,0,675.0608944447513,0,1259.185163234034,329.6319412921005,0,0,1};
    vector<float> distortion{-0.066483467712754,-0.062067208747137,0,0,0};
    vector<float> extrinsic{-0.00637662,-0.999867,0.015002,-0.0448843,-0.176276,-0.0136435,-0.984246,0.0352824,0.98432,-0.00892066,-0.176165,-0.3629,0,0,0,1};
    
    // set intrinsic parameters of the camera
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = intrinsic[0];
    cameraMatrix.at<double>(0, 2) = intrinsic[2];
    cameraMatrix.at<double>(1, 1) = intrinsic[4];
    cameraMatrix.at<double>(1, 2) = intrinsic[5];

	// set radial distortion and tangential distortion
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    distCoeffs.at<double>(0, 0) = distortion[0];
    distCoeffs.at<double>(1, 0) = distortion[1];
    distCoeffs.at<double>(2, 0) = distortion[2];
    distCoeffs.at<double>(3, 0) = distortion[3];
    distCoeffs.at<double>(4, 0) = distortion[4]; 


    float x1=3.189,y1=0.446000,z1=0.032000;

    float x2=3.14000,y2=-0.438000,z2= -0.572000;
    int u1,v1,u2,v2;
    getTheoreticalUV(theoryUV, intrinsic, extrinsic, x1, y1, z1);

    //projection(x1, y1, z1,u1,v1);
    //projection(x2, y2, z2,u2,v2);
    u1 = floor(theoryUV[0] + 0.5);
    v1 = floor(theoryUV[1] + 0.5);
    
    getTheoreticalUV(theoryUV, intrinsic, extrinsic, x2, y2, z2);
    u2 = floor(theoryUV[0] + 0.5);
    v2 = floor(theoryUV[1] + 0.5);
    
    cout<<u1<<","<<v1 << endl;
    cout<<u2<<"," <<v2 << endl;
    cv::Mat view, rview, map1, map2;
    cv::Size imageSize = src_img.size();
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
    cv::remap(src_img, src_img, map1, map2, cv::INTER_LINEAR);
    cv::Point p3(u1, v1), p4(u2, v2);
    cv::Scalar colorRectangle1(0, 0, 255);
	int thicknessRectangle1 = 2;
    cv::rectangle(src_img,p3,p4,colorRectangle1,thicknessRectangle1);
    cv::imshow("source", src_img);
    cv::waitKey(3);
    
}


