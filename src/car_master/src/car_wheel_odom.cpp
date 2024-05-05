// Copyright (c) 2024 cat
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
void callback(const geometry_msgs::Twist::ConstPtr &msg){
    
}

int main(int argc,char *argv[])
{
    ros::init(argc,argv,"car_wheel_odom");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("wheel_odom",100);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("current_cmd_vel",100,boost::bind(callback,_1));
    ros::spin();
    return 0;
}