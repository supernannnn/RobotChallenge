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
  camInit();
  imageTimer = nh.createTimer(ros::Duration(0.1), &LINE_PRECEPTION::imageCallback, this);
  image_transport::ImageTransport it(nh);
  cam_pub = it.advertise("image", 10);
}


void LINE_PRECEPTION::imageCallback(const ros::TimerEvent &e){
  
  bool isSuccess = cap.read(frame);
  if(!isSuccess)//if the video ends, then break
  {
      std::cout<<"video ends"<<std::endl;
  }
  image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
  cam_pub.publish(image);
}



void LINE_PRECEPTION::camInit(){
  cap.open(0);//open video from the path
  if(!cap.isOpened()){
    std::cout<<"open video failed!"<<std::endl;
    return;
  }
  else{
    std::cout<<"open camera success!"<<std::endl;
  }  
}


