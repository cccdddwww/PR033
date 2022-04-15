#include<ros/ros.h>
#include "pcl_image/Point.h"
#include "pcl_image/pointmatrix.h"

int main(int argc,char** argv){
    ros::init(argc,argv,"publisher");
    ros::NodeHandle n;
    ros::Publisher pub=n.advertise<pcl_image::Point>("/matrix",10);
    ros::Rate loop_rate(1);
    while(ros::ok()){
        pcl_image::Point Point;
        pcl_image::pointmatrix maxtrix1;
        pcl_image::pointmatrix maxtrix2;

        Point.header.stamp=ros::Time::now();
        maxtrix1.p1.x=1.972;
        maxtrix1.p1.y=0.69;
        maxtrix1.p1.z=0.834;
        maxtrix1.p2.x=1.574;
        maxtrix1.p2.y=-0.015;
        maxtrix1.p2.z=0.428;
        /*maxtrix2.p1.x=1;
        maxtrix2.p1.y=0.5;
        maxtrix2.p1.z=0.5;
        maxtrix2.p2.x=1;
        maxtrix2.p2.y=-0.5;
        maxtrix2.p2.z=-0.5;    */    
        Point.points.push_back(maxtrix1);
        //Point.points.push_back(maxtrix2);

        pub.publish(Point);
        loop_rate.sleep();
        

    }
    return 0;


}