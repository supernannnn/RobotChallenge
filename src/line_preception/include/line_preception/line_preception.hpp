#ifndef LINE_PRECEPTION_HPP
#define LINE_PRECEPTION_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>


class LINE_PRECEPTION{
private:

    //控制指令发布
    ros::Publisher cmd_pub;
    ros::Publisher cam_pub; //just for rosbag record


    //相机流
    cv::Mat frame;

    void readCam();

public:
    LINE_PRECEPTION(/* args */){

    }
    ~LINE_PRECEPTION(){

    }
    void init(ros::NodeHandle &nh);

};



#endif 