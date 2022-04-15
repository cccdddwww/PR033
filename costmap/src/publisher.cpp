#include<ros/ros.h>
#include "pcl_image/Point.h"
#include "pcl_image/pointmatrix.h"

int main(int argc,char** argv){
    ros::init(argc,argv,"publisher");
    ros::NodeHandle n;
    ros::Publisher pub=n.advertise<pcl_image::Point>("/matrix",10);
    ros::Rate loop_rate(30);
    while(ros::ok()){
        pcl_image::Point Point;
        pcl_image::pointmatrix maxtrix1;
        pcl_image::pointmatrix maxtrix2;

        Point.header.stamp=ros::Time::now();
        maxtrix1.p1.x=2.189000;
        maxtrix1.p1.y=0.446000;
        maxtrix1.p1.z=0.032000;
        maxtrix1.p2.x=2.140000;
        maxtrix1.p2.y=-0.438000;
        maxtrix1.p2.z=-0.572000;

        maxtrix2.p1.x=3.189000;
        maxtrix2.p1.y=0.446000;
        maxtrix2.p1.z=0.032000;
        maxtrix2.p2.x=3.140000;
        maxtrix2.p2.y=-0.438000;
        maxtrix2.p2.z=-0.572000;        
        Point.points.push_back(maxtrix1);
        Point.points.push_back(maxtrix2);

        pub.publish(Point);
        loop_rate.sleep();
        

    }
    return 0;


}
