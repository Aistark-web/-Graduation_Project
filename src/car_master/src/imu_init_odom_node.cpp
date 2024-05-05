// Copyright (c) 2024 stark
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

bool get_first = false;
void imu_sub_callback(const sensor_msgs::Imu::ConstPtr &msg,nav_msgs::Odometry &odom)
{
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation.x = msg->orientation.x;
    odom.pose.pose.orientation.y = msg->orientation.y;
    odom.pose.pose.orientation.z = msg->orientation.z;
    odom.pose.pose.orientation.w = msg->orientation.w;
    if(!get_first)
        get_first = true;
}

void pub_timer_callback(const ros::TimerEvent &event,ros::Publisher &pub,nav_msgs::Odometry &odom)
{
    if(get_first){
        pub.publish(odom);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"imu_init_odom_node");
    ros::NodeHandle nh;
    std::string imu_topic;
    std::string imu_init_odom_topic;
    ros::NodeHandle nh_self("~");
    nh_self.param<std::string>("imu_topic",imu_topic,"imu_data");
    nh_self.param<std::string>("imu_init_odom_topic",imu_init_odom_topic,"imu_init_odom");
    
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>(imu_init_odom_topic,100);
    ROS_INFO("[node:%s]: start",ros::this_node::getName().c_str());
    nav_msgs::Odometry odom;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Imu>(imu_topic,100,boost::bind(imu_sub_callback,_1,boost::ref(odom)));
    ros::Timer timer = nh.createTimer(ros::Duration(0.5),boost::bind(pub_timer_callback,_1,boost::ref(pub),boost::ref(odom)));

    ros::spin();
    return 0;
}
