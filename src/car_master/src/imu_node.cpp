// Copyright (c) 2024 stark
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "serial/serial.h"
#include <thread>

/**
 * @brief 校验数据
 * @param[in] data_buff 待校验数据
 * @return true 校验成功
 *         false 校验失败 
*/
bool checkSum(const std::vector<uint8_t> &data_buff)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < data_buff.size() - 1; i++)
    {
        sum += data_buff[i];
    }

    // 计算结果是否与输出一致
    return sum == data_buff[data_buff.size() - 1];
}


/// @brief 解析数据
/// @param hex_data 输入原始数据
/// @return 返回解析完的数据
std::vector<int16_t> hexToShort(const std::vector<uint8_t> &hex_data)
{
    uint8_t rawData[] = {hex_data[0], hex_data[1], hex_data[2], hex_data[3], hex_data[4], hex_data[5], hex_data[6], hex_data[7]};
    // 创建一个存储解析后整数的数组
    short result[4];
    // 使用memcpy将字节数组解析为整数数组
    memcpy(result, rawData, sizeof(result));

    std::vector<int16_t> short_data = {result[0], result[1], result[2], result[3]};
    return short_data;
}

void imu_thread_func(serial::Serial &serialport,std::string &frame_id,ros::Publisher &imu_pub)
{
    std::string Head = "[node:" + ros::this_node::getName() + "]:";
    // 打开串口
retry_open:    
    try{
        serialport.open();
        ROS_INFO("%s open serial port success",Head.c_str());
    }
    catch (const serial::IOException &e)
    {
        ROS_ERROR("%s open serial port error",Head.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(1));
        goto retry_open;
    }
    // 初始化IMU参数
    std::vector<uint8_t> buff;
    std::vector<int16_t> acceleration(4, 0);
    std::vector<int16_t> angularVelocity(4, 0);
    std::vector<int16_t> angle_degree(4, 0);
    std::vector<int16_t> quaternion(4,0);
    sensor_msgs::Imu imu_msg;
    imu_msg.header.seq = 0;
    imu_msg.header.frame_id = frame_id;
    bool get_acceleration = false;
    bool get_angularVelocity = false;
    bool get_angle_degree = false;
    bool get_quaternion = false;
    // 清除缓冲区
    serialport.flush();
    while(ros::ok()){
        if(serialport.available()){
            uint8_t data;
            serialport.read(&data,1);
            buff.push_back(data);
            // 接收到11个数据时且为第一个数据为0x55时
            if(buff.size() >= 11 && buff[0] == 0x55){
                // 获取11位数据
                std::vector<uint8_t> data_buff(buff.begin(), buff.begin() + 11);
                // 获取中间数字位
                std::vector<uint8_t> data(buff.begin() + 2, buff.begin() + 10);
                // 加速度数据
                if(data_buff[1] == 0x51){
                    if(checkSum(data_buff)){
                        acceleration = hexToShort(data);
                        get_acceleration = true;
                    }
                    else
                        ROS_WARN("%s 0x51 Check failure",Head.c_str());
                }
                // 角速度数据
                else if(data_buff[1] == 0x52){
                    if(checkSum(data_buff)){
                        angularVelocity = hexToShort(data);
                        get_angularVelocity = true;
                    }
                    else
                        ROS_WARN("%s 0x52 Check failure",Head.c_str());
                }
                // 欧拉角数据
                else if (data_buff[1] == 0x53){
                    if(checkSum(data_buff)){
                        angle_degree = hexToShort(data);
                        get_angle_degree = true;
                    }
                    else
                        ROS_WARN("%s 0x53 Check failure",Head.c_str());
                }
                // 四元数数据
                else if(data_buff[1] == 0x59){
                    if(checkSum(data_buff)){
                        quaternion = hexToShort(data);
                        get_quaternion = true;
                        if(get_acceleration && get_angle_degree && get_angularVelocity && get_quaternion){
                            // 载入加速度（m/s^2)
                            imu_msg.linear_acceleration.x = static_cast<double>(acceleration[0]) / 32768.0 * 16 * 9.8;
                            imu_msg.linear_acceleration.y = static_cast<double>(acceleration[1]) / 32768.0 * 16 * 9.8;
                            imu_msg.linear_acceleration.z = static_cast<double>(acceleration[2]) / 32768.0 * 16 * 9.8;

                            // 载入角速度（rad/s)
                            imu_msg.angular_velocity.x = static_cast<double>(angularVelocity[0]) / 32768.0 * 2000 * M_PI / 180;
                            imu_msg.angular_velocity.y = static_cast<double>(angularVelocity[1]) / 32768.0 * 2000 * M_PI / 180;
                            imu_msg.angular_velocity.z = static_cast<double>(angularVelocity[2]) / 32768.0 * 2000 * M_PI / 180;

                            // 转换四元数
                            imu_msg.orientation.x = static_cast<double>(quaternion[0])/32768.0;
                            imu_msg.orientation.y = static_cast<double>(quaternion[1])/32768.0;
                            imu_msg.orientation.z = static_cast<double>(quaternion[2])/32768.0;
                            imu_msg.orientation.w = static_cast<double>(quaternion[3])/32768.0;
                            imu_msg.header.seq++;
                            imu_msg.header.stamp = ros::Time::now();
                            imu_pub.publish(imu_msg);
                            get_acceleration = false;
                            get_angle_degree = false;
                            get_angularVelocity = false;
                            get_quaternion = false;
                        }
                    }
                    else
                        ROS_WARN("%s 0x59 Check failure",Head.c_str());
                }
                // 刷新数据
                buff.clear();
            }
            else if(buff[0] != 0x55){
                buff.clear();
            }
        }
        else{
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"imu_node");
    ros::NodeHandle nh;    
    ros::NodeHandle nh_self("~");
    std::string port_name;
    int baud;
    std::string frame_id;
    std::string imu_topic;
    // 初始化参数
    nh_self.param<std::string>("port_name",port_name,"/dev/ttyUSB0");
    nh_self.param<int>("baud",baud,115200);
    nh_self.param<std::string>("frame_id",frame_id,"Imu");
    nh_self.param<std::string>("imu_topic",imu_topic,"imu_data");
    ROS_INFO("[node:%s]: start [port_name:%s, baud:%d, frame_id:%s]",ros::this_node::getName().c_str(),port_name.c_str(),baud,frame_id.c_str());
    // 初始化串口参数
    serial::Serial serialport;
    serialport.setPort(port_name);
    serialport.setBaudrate(baud);
    serial::Timeout _timeout = serial::Timeout::simpleTimeout(200);
    serialport.setTimeout(_timeout);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic,100);
    std::thread imu_thead(imu_thread_func,std::ref(serialport),std::ref(frame_id),std::ref(imu_pub));
    imu_thead.detach();

    ros::spin();
    return 0;
}
