#ifndef _ODOM_FUSION_H_
#define _ODOM_FUSION_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <thread>
#include <serial/serial.h>



class ODOM_FUSION{
private:

    double altitude;
    ros::Subscriber odom_sub;
    ros::Publisher odom_fusion_pub;
    nav_msgs::Odometry odom_data;
    bool flag;
    bool have_altitude;
    void readLaser();
    void OdomCallback(const nav_msgs::OdometryConstPtr msg);
public:
    ODOM_FUSION(){
        have_altitude = false;
    }
    ~ODOM_FUSION(){}
    void init(ros::NodeHandle& nh);
};



#endif