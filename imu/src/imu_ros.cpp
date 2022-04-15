#include "imu_ros.h"

Imu_ros::Imu_ros():
    fd_(0),
    is_quite_(false),
    key_(0)
{
    ros::NodeHandle nh("~/imu_uart");
    nh.param("port_name", port_name_, std::string("/dev/ttyUSB0"));
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 10);
    SetBaudRate(BR921600);
    SetParity(P_8N1);
    pub_flag[Euler_Angles] = true;
    pub_flag[Raw_Data] = true;

    imu_buff_.resize(256, 0);
}

Imu_ros::~Imu_ros()
{
    Stop();
}

bool Imu_ros::Start() {
    if (!Open()) {
    return false;
    }
    is_quite_ = false;
    listener_ = std::make_shared<std::thread>(&Imu_ros::ImuIOLoop, this);
    return true;
}

void Imu_ros::ImuIOLoop() {
    uint8_t buff[READ_BUF];
    size_t size = 0;
    while (!is_quite_) {
        if (0 != (size = Read(buff, READ_BUF))) {
            ImuDecode(buff, size);
        }
        /*if (0 != (size = Read(buff, 1))) {
            uint8_t a = buff[0];
            handleSerialData(a);
        }*/
    }
}

size_t Imu_ros::Read(uint8_t *buffer, size_t size) {
    if (fd_ > 0) {
        return read(fd_, buffer, size);
    } else {
        return 0;
    }
}

void Imu_ros::ImuDecode(uint8_t * encode_buff, size_t size) {
    for (size_t i = 0; i < size; i++) {
        //if (handleSerialData(encode_buff[i])){
          //  printf("handle imu data succeed!\n");
        //}
        handleSerialData(encode_buff[i]);
    }
}

//hfi-a9 serial communication protocol
bool Imu_ros::handleSerialData(uint8_t buf){
    imu_buff_[key_] = buf;
    key_++;
    if(imu_buff_[0] != 0xaa){
        key_ = 0;
        return false;
    }
    if(key_ < 3){
        return false;
    }
    if(imu_buff_[1] != 0x55){
        key_ = 0;
        return false;
    }
    //According to the judgment of the data length bit,
    // to obtain the corresponding length data
    if(key_ < imu_buff_[2] + 5){
        return false;
    }
    else{
        std::vector<uint8_t> check_data(2,0);//校验码两个字节
        if(imu_buff_[2] == 0x14 && pub_flag[Euler_Angles]){
            std::vector<uint8_t> list_data_euler_angle(21, 0);//角度数据除去包头和校验码有21个字节
            for(uint8_t i = 0; i < 21; i++){
                list_data_euler_angle[i] = imu_buff_[i + 2];
            }
            check_data[0] = imu_buff_[23];
            check_data[1] = imu_buff_[24];
            if(CRCcheck(list_data_euler_angle, check_data)){
                std::vector<uint8_t> valid_Euler_angles_data(16, 0);
                for(uint8_t j = 0; j < 16; j++){
                    valid_Euler_angles_data[j] = list_data_euler_angle[j+5];
                }
                std::vector<float> convert_euler_data;
                convert_euler_data = hex_to_ieee(valid_Euler_angles_data);
                angle_degree[0] = convert_euler_data[1];
                angle_degree[1] = convert_euler_data[2];
                angle_degree[2] = convert_euler_data[3];
            }
            else{
                printf("check method_one failed!\n");
            }
            pub_flag[Euler_Angles] = false;
        }
        else if(imu_buff_[2] == 0x2c && pub_flag[Raw_Data]){
            std::vector<uint8_t> list_data_raw(45, 0);//角度数据除去包头和校验码有45个字节
            for(uint8_t i = 0; i < 45; i++){
                list_data_raw[i] = imu_buff_[i + 2];
            }
            check_data[0] = imu_buff_[47];
            check_data[1] = imu_buff_[48];
            if(CRCcheck(list_data_raw, check_data)){
                std::vector<uint8_t> valid_raw_data(40, 0);
                for(uint8_t j = 0; j < 40; j++){
                    valid_raw_data[j] = list_data_raw[j+5];
                }
                std::vector<float> convert_raw_data;
                convert_raw_data = hex_to_ieee(valid_raw_data);
                angularVelocity[0] = convert_raw_data[1];
                angularVelocity[1] = convert_raw_data[2];
                angularVelocity[2] = convert_raw_data[3];
                acceleration[0] = convert_raw_data[4];
                acceleration[1] = convert_raw_data[5];
                acceleration[2] = convert_raw_data[6];
                magnetometer[0] = convert_raw_data[7];
                magnetometer[1] = convert_raw_data[8];
                magnetometer[2] = convert_raw_data[9];
            }
            else{
                printf("check method_two failed!\n");
            }
            pub_flag[Raw_Data] = false;
        }
        else{
            printf("data error! no such type\n");
            key_ = 0;
            ClearImuBuff();
            return false;
        }
        key_ = 0;
        ClearImuBuff();
        if(pub_flag[Euler_Angles] == true || pub_flag[Raw_Data] == true){
            return false;
        }
        pub_flag[Euler_Angles] = true;
        pub_flag[Raw_Data] = true;
        acc_k_ = sqrt(acceleration[0]*acceleration[0] + acceleration[1]*acceleration[1]
                                                        + acceleration[2]*acceleration[2]);
        imu_data.header.frame_id = "imu_link";
        imu_data.header.stamp = ros::Time::now();
        float angle_radian[3] = {0, 0, 0};
        angle_radian[0] = angle_degree[0] * M_PI / 180;
        angle_radian[1] = angle_degree[1] * M_PI / 180;
        angle_radian[2] = angle_degree[2] * M_PI / 180;

        imu_data.orientation = tf::createQuaternionMsgFromRollPitchYaw(angle_radian[0], -angle_radian[1], -angle_radian[2]);
        imu_data.angular_velocity.x = angularVelocity[0];
        imu_data.angular_velocity.y = angularVelocity[1];
        imu_data.angular_velocity.z = angularVelocity[2];

        imu_data.linear_acceleration.x = acceleration[0] * -9.8 / acc_k_;
        imu_data.linear_acceleration.y = acceleration[1] * -9.8 / acc_k_;
        imu_data.linear_acceleration.z = acceleration[2] * -9.8 / acc_k_;

          // publish tf transformation
        /*    geometry_msgs::Quaternion imu_quat = tf::createQuaternionMsgFromYaw(0);

        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = imu_data.header.stamp;
        tf_msg.header.frame_id = "base_link";
        tf_msg.child_frame_id = "imu_link";

        tf_msg.transform.translation.x = 0.56;
        tf_msg.transform.translation.y = -0.064;
        tf_msg.transform.translation.z = 0.675;
        tf_msg.transform.rotation = imu_quat;

        tf_broadcaster_.sendTransform(tf_msg);*/

        imu_pub.publish(imu_data);



/*        printf("加速度(m/s²) : x轴: %6.2f y轴: %6.2f z轴: %6.2f\n", 
        acceleration[0] * -9.8 / acc_k_, acceleration[1] * -9.8 / acc_k_, acceleration[2] * -9.8 / acc_k_);
        printf("角速度(rad/s): x轴: %6.2f y轴: %6.2f z轴: %6.2f\n", 
        angularVelocity[0], angularVelocity[1], angularVelocity[2]);
        printf("欧拉角(°)    : x轴: %6.2f y轴: %6.2f z轴: %6.2f\n", 
        angle_degree[0], angle_degree[1], angle_degree[2]);
        printf("磁场         : x轴: %6.2f y轴: %6.2f z轴: %6.2f\n", 
        magnetometer[0], magnetometer[1], magnetometer[2]);
	    printf("\033[1A");
	    printf("\033[1A");
        printf("\033[1A");
        printf("\033[1A");*/
    }
    return true;
}

//CRC check
bool Imu_ros::CRCcheck(std::vector<uint8_t> data_buff, std::vector<uint8_t> check_buff){
    uint16_t crc = 0xffff;
    for(unsigned int i = 0; i < data_buff.size(); i++){
        crc ^= data_buff[i];
        for(unsigned int j = 0; j < 8; j++){
            if((crc & 1) != 0){
                crc >>= 1;
                crc ^= 0xa001;
            }
            else crc >>= 1;
        }
    }
    return (((crc & 0xff) << 8) + (crc >> 8)) == (check_buff[0] << 8 | check_buff[1]);
}

std::vector<float> Imu_ros::hex_to_ieee(std::vector<uint8_t> hex_data){
    uint8_t data_unit[4];
    std::vector<float> ieee_results;
    for(unsigned int i = 0; i < hex_data.size(); i+=4){
        data_unit[0] = hex_data[i];
        data_unit[1] = hex_data[i+1];
        data_unit[2] = hex_data[i+2];
        data_unit[3] = hex_data[i+3];
        ieee_results.push_back(*(float*)data_unit);
    }
    return ieee_results;
}

void Imu_ros::ClearImuBuff() {
    std::fill(imu_buff_.begin(), imu_buff_.end(), 0);
}

void Imu_ros::Stop() {
    is_quite_ = true;
    if (listener_) {
    listener_->join();
    listener_ = nullptr;
    }
    Close();
}

bool Imu_ros::Open() {
    fd_ = open(port_name_.c_str(), O_RDONLY | O_NOCTTY);
    if (fd_ < 0) {
    printf("Open %s serials fail!\n", port_name_.c_str());
    return false;
    }
    /* Set baudrate and parity,etc. */
    Setup(baudrate_, parity_);
    printf("Set baudrate success.\n");
    return true;
}

void Imu_ros::Close() {
    if (fd_ > 0) {
    /* Flush the port */
    tcflush(fd_,TCOFLUSH); 
    tcflush(fd_,TCIFLUSH); 

    close(fd_);
    }
    fd_ = 0;
}

void Imu_ros::SetPortName(std::string port_name) {
    port_name_ = port_name;
}

void Imu_ros::SetBaudRate(BaudRate baudrate) {
    baudrate_ = baudrate;
}

void Imu_ros::SetParity(Parity parity) {
    parity_ = parity;
}

/* Sets up the port parameters */
int Imu_ros::Setup(enum BaudRate baud, enum Parity parity) {
    static uint32_t baud_map[19] = {
        B2400, B4800, B9600, B19200, B38400, B57600,B115200, B230400,B460800, B500000, B576000,B921600,
        B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
        };
    tcflag_t baudrate;
    struct termios options;

    /* Clear old setting completely,must add here for A3 CDC serial */
    tcgetattr(fd_, &options);
    memset(&options, 0, sizeof(options));
    tcflush(fd_, TCIOFLUSH);
    tcsetattr(fd_, TCSANOW, &options);
    usleep(10000);

    /* Minimum number of characters to read */
    options.c_cc[VMIN] = 0;
    /* Time to wait for data */
    options.c_cc[VTIME] = 100;

    /* Enable the receiver and set local mode... */
    options.c_cflag |= (CLOCAL | CREAD);

    /* Set boadrate */
    baudrate = baud_map[baud];
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
    printf("[Baudrate]: %d %u\r\n", baud, baudrate);

    switch (parity) {
    case P_8N1:
        /* No parity (8N1)  */
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
    break;
    case P_7E1:
        /* Even parity (7E1) */
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS7;
    break;
    case P_7O1:
        /* Odd parity (7O1) */
        options.c_cflag |= PARENB;
        options.c_cflag |= PARODD;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS7;
    break;
    case P_7S1:
        /* Space parity is setup the same as no parity (7S1)  */
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
    break;
    default:
        return -1;
    } 
    /* flush the port */
    tcflush(fd_, TCIOFLUSH);
    
    /* send new config to the port */
    tcsetattr(fd_, TCSANOW, &options);

    return 0;
}
