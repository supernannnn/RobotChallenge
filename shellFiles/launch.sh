#! /bin/bash

sudo chmod 777 /dev/ttyACM0 &sleep 3;
sudo chmod 777 /dev/ttyUSB0 &sleep 1;
roslaunch mavros px4.launch & sleep 10;
roslaunch realsense2_camera rs_t265.launch &sleep 10;
roslaunch odom_fusion odom_fusion.launch &sleep 5
wait;


