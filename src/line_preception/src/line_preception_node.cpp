#include <line_preception/line_preception.hpp>


int main(int argc, char **argv){
    ros::init(argc, argv, "line_preception_node");
    ros::NodeHandle nh("~");
    LINE_PRECEPTION line_pre;
    line_pre.init(nh);
    ros::spin();
    return 0;
}

void LINE_PRECEPTION::init(ros::NodeHandle &nh){

  imageTimer = nh.createTimer(ros::Duration(0.1), &LINE_PRECEPTION::imageCallback, this);
  image_transport::ImageTransport it(nh);
  cam_pub = it.advertise("camera", 10);

  std::thread rtk_thread(&LINE_PRECEPTION::readCam, this);
  rtk_thread.detach();  
}


void LINE_PRECEPTION::imageCallback(const ros::TimerEvent &e){
  cam_pub.publish(image);
}


void LINE_PRECEPTION::readCam(){
  cv::VideoCapture cap(0);//open video from the path
  if(!cap.isOpened()){
    std::cout<<"open video failed!"<<std::endl;
    return;
  }
  else{
    std::cout<<"open camera success!"<<std::endl;
  }  
  bool isSuccess = true;
  //ros::Rate loop_rate(30);
  while(ros::ok()){
    isSuccess = cap.read(frame);
    if(!isSuccess)//if the video ends, then break
    {
        std::cout<<"video ends"<<std::endl;
        break;
    }
    image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    // cv::imshow("Video Frame", frame);
    // cv::waitKey(0);
    //loop_rate.sleep();
  }
}


