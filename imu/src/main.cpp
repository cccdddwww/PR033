#include "imu_ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <chrono>
#include <string.h>
 
#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "imu_node");
    Imu_ros imu_uart;
    if(imu_uart.Start()){
        printf("Imu_uart start success\n");
    } 
    else {
        printf("Imu_uart start failed\n");
        return 0;
    }
    //std::this_thread::sleep_for(std::chrono::seconds(10000));
    while(ros::ok()){
        sleep(1);
    }
    imu_uart.Stop();
    printf("Imu_uart demo end!\n");
}
