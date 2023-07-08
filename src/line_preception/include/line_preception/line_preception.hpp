#ifndef LINE_PRECEPTION_HPP
#define LINE_PRECEPTION_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class LINE_PRECEPTION{
private:

    //控制指令发布
    ros::Publisher cmd_pub;

    /***just for rosbag record***/
    image_transport::Publisher cam_pub; 
    sensor_msgs::ImagePtr image;
    ros::Timer imageTimer;
    void imageCallback(const ros::TimerEvent &e);
    /****************************/

    //相机流
    cv::VideoCapture cap;
    cv::Mat frame;

    //相机初始化
    void camInit();

public:
    LINE_PRECEPTION(/* args */){

    }
    ~LINE_PRECEPTION(){

    }
    void init(ros::NodeHandle &nh);

};



#endif 