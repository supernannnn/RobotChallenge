#ifndef _ODOM_FUSION_H_
#define _ODOM_FUSION_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <vector>
#include <thread>
#include <serial/serial.h>


#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>


#include <std_msgs/Float64.h>


class ODOM_FUSION{
private:

    ros::Timer test_odom_timer; 
    double altitude;
    ros::Subscriber odom_sub, px4_imu_sub;
    ros::Publisher odom_fusion_pub, test_odom_pub;

    ros::Publisher PX4_yaw_pub, T265_yaw_pub;

    nav_msgs::Odometry odom_data;
    bool flag;
    bool have_altitude;

    double tmp_altitude;

    void readLaser();
    void OdomCallback(const nav_msgs::OdometryConstPtr msg);
    void PX4IMUCallback(const sensor_msgs::ImuConstPtr msg);
    void test_odomCallback(const ros::TimerEvent &e);
public:
    ODOM_FUSION(){
        have_altitude = false;
        tmp_altitude = 0;
    }
    ~ODOM_FUSION(){}
    void init(ros::NodeHandle& nh);
};



#endif