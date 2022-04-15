#ifndef IMU_ROS_H_
#define IMU_ROS_H_

#include <stdint.h>
#include <memory>
#include <thread>
#include <vector>
#include <string>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

//Operations on file opening, data writing, data reading, and file closing.
#include<sys/types.h>
#include<sys/stat.h>
#include <fcntl.h>

#include <termios.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>


#define READ_BUF (256)
#define Euler_Angles (0)
#define Raw_Data (1)


enum Parity {
    P_8N1,    /* No parity (8N1)	*/     
    P_7E1,    /* Even parity (7E1)*/
    P_7O1,    /* Odd parity (7O1)	*/
    P_7S1     /* Space parity is setup the same as no parity (7S1)	*/
};

enum BaudRate {
    BR2400,
    BR4800,
    BR9600,
    BR19200,
    BR38400,
    BR57600,
    BR115200,
    BR230400,
    BR460800,
    BR500000,
    BR576000,
    BR921600,
    BR1152000,
    BR1500000,
    BR2000000,
    BR2500000,
    BR3000000,
    BR3500000,
    BR4000000,
};

class Imu_ros
{
private:
    void Close();
    bool Open();
    int Setup(enum BaudRate baud, enum Parity parity);
    void ImuIOLoop();
    size_t Read(uint8_t *buffer, size_t size);
    void ImuDecode(uint8_t* encode_buff, size_t size);
    bool handleSerialData(uint8_t buf);
    bool CRCcheck(std::vector<uint8_t> data_buff, std::vector<uint8_t> check_buff);
    std::vector<float> hex_to_ieee(std::vector<uint8_t> hex_data);
    void ClearImuBuff();

    enum BaudRate baudrate_;
    enum Parity parity_;
    std::string port_name_;
    int fd_;
    bool is_quite_;
    std::shared_ptr<std::thread> listener_;
    std::vector<uint8_t> imu_buff_;
    int key_;
    bool pub_flag[2];    
    float acc_k_;

    ros::Publisher imu_pub;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

    sensor_msgs::Imu imu_data;

public:
    Imu_ros();
    ~Imu_ros();
    
    bool Start();
    void Stop();
    void SetBaudRate(BaudRate baudrate);
    void SetPortName(std::string port_name);
    void SetParity(Parity parity);

    float acceleration[3] = {0, 0, 0};
    float angularVelocity[3] = {0, 0, 0};
    float magnetometer[3] = {0, 0, 0};
    float angle_degree[3] = {0, 0, 0};
};
#endif
