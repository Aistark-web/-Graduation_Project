#include "ros/ros.h"
#include "xboxseries_joy/xboxseries_joy_controller.hpp"
#include "xboxseries_joy/joy.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include <thread>

xboxseries_joy_controller controller;
float Vx = 0.3;
float Vy = 0.3;
float Az = 0.5;

void pub_joy_thread(ros::Publisher &joy_pub,xboxseries_joy::joy &joy){
    joy.LX = controller.LX;
    joy.LY = controller.LY;
    joy.RX = controller.RX;
    joy.RY = controller.RY;
    joy.LT = controller.LT;
    joy.RT = controller.RT;
    joy.DIR_X = controller.DIR_X;
    joy.DIR_Y = controller.DIR_Y;
    joy.LS = controller.LS;
    joy.RS = controller.RS;
    joy.LB = controller.LB;
    joy.RB = controller.RB;
    joy.A = controller.A;
    joy.B = controller.B;
    joy.X = controller.X;
    joy.Y = controller.Y;
    joy.VIEW = controller.VIEW;
    joy.MENU = controller.MENU;
    joy.XBOX = controller.XBOX;
    joy_pub.publish(joy);
}

void joy_callback(const sensor_msgs::Joy::ConstPtr &msg,ros::Publisher &cmd_vel_pub,geometry_msgs::Twist &cmd_vel,
            ros::Publisher &joy_pub,xboxseries_joy::joy &joy){
    controller.transfer(msg);
    std::thread t1(pub_joy_thread,std::ref(joy_pub),std::ref(joy));
    t1.detach();
    if(controller.LX > 0.5f){
        cmd_vel.linear.x = Vx;
    }
    else if(controller.LX < -0.5f){
        cmd_vel.linear.x = -Vx;
    }
    else{
        cmd_vel.linear.x = 0;
    }

    if(controller.RY > 0.5f){
        cmd_vel.linear.y = Vy;
    }
    else if(controller.RY < -0.5f){
        cmd_vel.linear.y = -Vy;
    }
    else{
        cmd_vel.linear.y = 0;
    }

    if(controller.LB && !controller.RB){
        cmd_vel.angular.z = Az;
    }
    else if(controller.RB && !controller.LB){
        cmd_vel.angular.z = -Az;
    }
    else{
        cmd_vel.angular.z = 0;
    }
    cmd_vel_pub.publish(cmd_vel);
}

int main(int argc,char *argv[])
{
    ros::init(argc,argv,"car_joy");
    ros::NodeHandle nh;
    ROS_INFO("Node:[%s] start",ros::this_node::getName().c_str());
    geometry_msgs::Twist cmd_vel;
    xboxseries_joy::joy joy;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",100);
    ros::Publisher joy_pub = nh.advertise<xboxseries_joy::joy>("xboxseries_joy",100);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Joy>("joy",100,boost::bind(joy_callback,_1,cmd_vel_pub,cmd_vel,joy_pub,joy));
    ros::spin();
    return 0;
}