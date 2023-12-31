#include <odom_fusion/odom_fusion.h>


void ODOM_FUSION::init(ros::NodeHandle& nh){

    odom_fusion_pub = nh.advertise<nav_msgs::Odometry>("/fusion/odom", 50);
    test_odom_pub   = nh.advertise<nav_msgs::Odometry>("/fusion/test_odom", 50);

    PX4_yaw_pub = nh.advertise<std_msgs::Float64>("/px4_debug_yaw", 10);
    T265_yaw_pub = nh.advertise<std_msgs::Float64>("/t265_debug_yaw", 10);

    odom_sub        = nh.subscribe("odom", 100, &ODOM_FUSION::OdomCallback, this, ros::TransportHints().tcpNoDelay());

    px4_imu_sub     = nh.subscribe("/mavros/imu/data", 100, &ODOM_FUSION::PX4IMUCallback, this, ros::TransportHints().tcpNoDelay());


    test_odom_timer = nh.createTimer(ros::Duration(0.5), &ODOM_FUSION::test_odomCallback, this);
    std::thread rtk_thread(&ODOM_FUSION::readLaser, this);
    rtk_thread.detach();                                            //分离线程在后台运行
}

void ODOM_FUSION::PX4IMUCallback(const sensor_msgs::ImuConstPtr msg){
    Eigen::Quaterniond q;

    q.x() = msg->orientation.x;
    q.y() = msg->orientation.y;
    q.z() = msg->orientation.z;
    q.w() = msg->orientation.w;
    double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());

    std_msgs::Float64 yaw_msg;
    yaw_msg.data = yaw;

    PX4_yaw_pub.publish(yaw_msg);
    
}


void ODOM_FUSION::test_odomCallback(const ros::TimerEvent &e){
    test_odom_pub.publish(odom_data);
}  

void ODOM_FUSION::OdomCallback(const nav_msgs::OdometryConstPtr msg){

    Eigen::Quaterniond q;

    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    q.w() = msg->pose.pose.orientation.w;
    double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z()); 

    std_msgs::Float64 yaw_msg;
    yaw_msg.data = yaw;

    T265_yaw_pub.publish(yaw_msg);

    if (have_altitude){
        static bool flag = true;
        if (flag){
            ROS_WARN("Get Altitude!");
            flag = false;
        }
        odom_data = *msg;
        odom_data.header.frame_id = "world";
        if (altitude == 0){
            odom_data.pose.pose.position.z = tmp_altitude;
        }else{
            odom_data.pose.pose.position.z = altitude;
            tmp_altitude = altitude;
        }
        odom_fusion_pub.publish(odom_data);
    }
}


void ODOM_FUSION::readLaser(){

    serial::Serial serial_port;
    try
    {
        // 打开串口设备
        serial_port.setPort("/dev/ttyUSB0");
        serial_port.setBaudrate(115200);
        serial_port.setBytesize(serial::eightbits);  // 设置数据位为8
        serial_port.setStopbits(serial::stopbits_one);  // 设置停止位为1
        serial_port.setParity(serial::parity_none);  // 设置奇偶校验位为None
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100000);
        serial_port.setTimeout(timeout);
        serial_port.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Failed to open the serial port: " << e.what());
        return ;
    }

    if(serial_port.isOpen())
    {
        ROS_INFO_STREAM("Serial Port opened");
    }

    //经过实测发现，前41个字节没有用，所以先读取进行过滤
    serial_port.read(41);       

    // // 解析强度值
    // uint16_t strength;

    // // 解析温度值
    // float temperature_c;

    ros::Rate loop_time(100);
    while(ros::ok()){
        
        if(serial_port.available()){
            uint8_t buffer[9];
            size_t bytes_read = serial_port.read(buffer, 9);
            // 检查是否成功读取到完整的数据帧
            if (bytes_read != 9)
            {
                ROS_ERROR_STREAM("Failed to read complete data frame");
                continue;
            }

            if (buffer[0] == 0x59 && buffer[1] == 0x59)
            {
                // 解析距离值
                uint16_t distance = (buffer[3] << 8) | buffer[2];
                altitude = static_cast<float>(distance) / 100.0;  // 距离值以米为单位
                have_altitude = true;
                // 解析强度值
                // strength = (buffer[5] << 8) | buffer[4];
                // // 解析温度值
                // int16_t temperature = (buffer[7] << 8) | buffer[6];
                // temperature_c = static_cast<float>(temperature) / 8.0 - 256.0;  // 温度值以摄氏度为单位
                // std::cout << "Distance: " << altitude << " m" << std::endl;
                // std::cout << "Strength: " << strength << std::endl;
                // std::cout << "Temperature: " << temperature_c << " °C" << std::endl;           
            }
        }

    }    
}