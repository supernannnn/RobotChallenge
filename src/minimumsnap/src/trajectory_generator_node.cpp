#include "minimumsnap/trajectory_generator.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "trajectory_generator_node");
    ros::NodeHandle nh("~");
    TRAJECTORY_GENERATOR trajectory_generator;
    trajectory_generator.init(nh);
    ros::spin();
    return 0;
}

void TRAJECTORY_GENERATOR::init(ros::NodeHandle& nh){
    nh.param("planning/vel", _Vel, 1.0);
    nh.param("planning/acc", _Acc, 1.0);
    nh.param("planning/dev_order", _dev_order, 3);  // the order of derivative, _dev_order = 3->minimum jerk, _dev_order = 4->minimum snap
    nh.param("planning/min_order", _min_order, 3);
    nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);

    _poly_num1D = 2 * _dev_order;

    nh.param("pillar1_x", pillar1(0), 0.0);
    nh.param("pillar1_y", pillar1(1), 0.0);
    nh.param("pillar1_z", pillar1(2), 0.0);

    nh.param("pillar2_x", pillar2(0), 0.0);
    nh.param("pillar2_y", pillar2(1), 0.0);
    nh.param("pillar2_z", pillar2(2), 0.0);

    nh.param("time_forward", time_forward_, -1.0);

    //调用PlanningVisualization构造函数进行初始化
    trajVisual_.reset(new PlanningVisualization(nh));
    //柱子可视化
    trajVisual_->displayPillars(vector<Eigen::Vector3d>{pillar1, pillar2});

    odom_sub = nh.subscribe("odom", 100, &TRAJECTORY_GENERATOR::OdomCallback, this, ros::TransportHints().tcpNoDelay());

    trajPath_pub    = nh.advertise<nav_msgs::Path>("trajectory", 10);
    controlCmd_pub  = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

    
    ControlCmdCallback_Timer = nh.createTimer(ros::Duration(0.01), &TRAJECTORY_GENERATOR::ControlCmdCallback, this);
    Transformer_Timer        = nh.createTimer(ros::Duration(0.01), &TRAJECTORY_GENERATOR::TransformerCallback, this);
    GetWaypoints();
}


void TRAJECTORY_GENERATOR::TransformerCallback(const ros::TimerEvent &e){
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "world";
    transform.child_frame_id = "camera_odom_frame";
    transform.transform.translation.x = 0.0;  // 单位平移x坐标
    transform.transform.translation.y = 0.0;  // 单位平移y坐标
    transform.transform.translation.z = 0.0;  // 单位平移z坐标
    transform.transform.rotation.x = 0.0;  // 单位四元数x分量
    transform.transform.rotation.y = 0.0;  // 单位四元数y分量
    transform.transform.rotation.z = 0.0;  // 单位四元数z分量
    transform.transform.rotation.w = 1.0;  // 单位四元数w分量
    broadcaster.sendTransform(transform);    
}

void TRAJECTORY_GENERATOR::OdomCallback(const nav_msgs::OdometryConstPtr msg){
    odom_pos(0) = msg->pose.pose.position.x;
    odom_pos(1) = msg->pose.pose.position.y;
    odom_pos(2) = msg->pose.pose.position.z;
    has_odom = true;
}


/*
八字路标生成：
            5
        *        *
      *            *
    6        ×       4
      *            *  
        *        *
            3/7
         *       * 
      *             *
    2         ×       8
     *               *
       *           * 
          *   1  *
*/
void TRAJECTORY_GENERATOR::GetWaypoints(){
    nav_msgs::Path waypoints;
    geometry_msgs::PoseStamped pt;
    pt.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    pt.pose.position.x = pillar1(0) - 1.5;
    pt.pose.position.y = pillar1(1);
    pt.pose.position.z = 1;
    waypoints.poses.push_back(pt);

    pt.pose.position.x = pillar1(0);
    pt.pose.position.y = pillar1(1) + 1;
    pt.pose.position.z = 1;
    waypoints.poses.push_back(pt);

    pt.pose.position.x = (pillar1(0) + pillar2(0)) / 2;
    pt.pose.position.y = pillar1(1);
    pt.pose.position.z = 1;
    waypoints.poses.push_back(pt);

    // pt.pose.position.x = pillar2(0);
    // pt.pose.position.y = pillar2(1) - 1.5;
    // pt.pose.position.z = 1;
    // waypoints.poses.push_back(pt);

    // pt.pose.position.x = pillar2(0) + 1.5;
    // pt.pose.position.y = pillar2(1);
    // pt.pose.position.z = 1;
    // waypoints.poses.push_back(pt);

    // pt.pose.position.x = pillar2(0);
    // pt.pose.position.y = pillar2(1) + 1.5;
    // pt.pose.position.z = 1;
    // waypoints.poses.push_back(pt);

    // pt.pose.position.x =(pillar1(0) + pillar2(0)) / 2;
    // pt.pose.position.y = pillar1(1);
    // pt.pose.position.z = 1;
    // waypoints.poses.push_back(pt);

    // pt.pose.position.x = pillar1(0);
    // pt.pose.position.y = pillar1(1) - 1.5;
    // pt.pose.position.z = 1;
    // waypoints.poses.push_back(pt); 

    waypointsCallback(waypoints);  

}



void TRAJECTORY_GENERATOR::waypointsCallback(const nav_msgs::Path &msg){
    
    vector<Eigen::Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int)msg.poses.size(); k++)
    {
        Vector3d pt( msg.poses[k].pose.position.x, msg.poses[k].pose.position.y, msg.poses[k].pose.position.z);
        wp_list.push_back(pt);
        ROS_INFO("waypoint%d: (%f, %f, %f)", k+1, pt(0), pt(1), pt(2));
    }

    //add the original point
    MatrixXd waypoints(wp_list.size() + 1, 3);  
    waypoints.row(0) = odom_pos;

    ros::Rate loop_rate(100);
    for(int k = 1; k <= (int)wp_list.size(); k++){
        waypoints.row(k) = wp_list[k - 1];
        waypoints_visual.push_back(wp_list[k - 1]);
        loop_rate.sleep();
    }
    //路标可视化
    trajVisual_->displayWaypoints(waypoints_visual);
    trajGeneration(waypoints);
}

void TRAJECTORY_GENERATOR::trajGeneration(Eigen::MatrixXd& path){
    
    TRAJECTORY_GENERATOR_WAYPOINT trajectoryGeneratorWaypoint;
    MatrixXd vel  = MatrixXd::Zero(2, 3); 
    MatrixXd acc  = MatrixXd::Zero(2, 3);
    vel.row(0)  = _startVel;
    vel.row(1)  = _endVel;
    acc.row(0)  = _startAcc;
    acc.row(1)  = _endAcc;

    // use "trapezoidal velocity" time allocation
    ros::Time time_start = ros::Time::now();
    _polyTime  = timeAllocation(path);
    // generate a minimum-jerk/snap piecewise monomial polynomial-based trajectory
    _polyCoeff = trajectoryGeneratorWaypoint.PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);
    Traj = MiniSnapTraj(time_start, _polyTime, _polyCoeff);
    ros::Time time_end = ros::Time::now();
    ROS_WARN("Time consumed in trajectory generation is %f ms", (time_end - time_start).toSec() * 1000.0); 
    ROS_WARN("The duration of trajectory is %f s", Traj.time_duration); 

    is_complete_Traj = true;
    //轨迹可视化
    getVisual(_polyCoeff, _polyTime);
    //发布轨迹到rviz进行可视化
    trajVisual_->displayTraj(traj);

}


void TRAJECTORY_GENERATOR::getVisual(Eigen::MatrixXd& polyCoeff, Eigen::VectorXd& time){
    double traj_len = 0.0;
    int count = 1;
    Eigen::Vector3d cur, pre, vel;
    cur.setZero();
    pre.setZero();
    geometry_msgs::PoseStamped poses;
    trajPath.header.frame_id = poses.header.frame_id = "world";
    trajPath.header.stamp    = poses.header.stamp    = ros::Time::now();
    poses.pose.orientation     = tf::createQuaternionMsgFromYaw(0.0);

    ros::Rate loop_rate(1000);
    for(double t = 0.0; t < Traj.time_duration; t += 0.01, count++)   // go through each segment
    {   
        auto info = getTrajInfo(Traj, t);
        cur = info.first;
        vel = info.second;
        poses.pose.position.x = cur[0];
        poses.pose.position.y = cur[1];
        poses.pose.position.z = cur[2];

        trajPath.poses.push_back(poses);
        traj.push_back(cur);
        trajPath_pub.publish(trajPath);
        if (count % 1000 == 0) traj_len += (pre - cur).norm();
        pre = cur;
        loop_rate.sleep();
    }
    
    ROS_WARN("Trajectory length is %f m", traj_len);
}

Eigen::VectorXd TRAJECTORY_GENERATOR::timeAllocation(Eigen::MatrixXd& Path){
    VectorXd time(Path.rows() - 1);
    // The time allocation is many relative timelines but not one common timeline
    for(int i = 0; i < time.rows(); i++)
    {
        double distance = (Path.row(i+1) - Path.row(i)).norm();    // or .lpNorm<2>()
        double x1 = _Vel * _Vel / (2 * _Acc); 
        double x2 = distance - 2 * x1;
        double t1 = _Vel / _Acc;
        double t2 = x2 / _Vel;
        time(i) = 2 * t1 + t2;
    }
    return time;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d>  TRAJECTORY_GENERATOR::getTrajInfo(const MiniSnapTraj& traj, double time){
    int     traj_id;
    double  t;
    //判断当前时间处于哪段轨迹当中
    for (int i = 0; i < traj.traj_nums; i++){
        if (time <= traj.poly_time(i)){
            traj_id = i;
            t = (i == 0 ? time : time - traj.poly_time(i - 1));
            break;
        } 
    }
    Vector3d position, velocity;
    for (int dim = 0; dim < 3; dim++){
        VectorXd coeff      = (traj.poly_coeff.row(traj_id)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd posi_time  = VectorXd::Zero( _poly_num1D );
        VectorXd vel_time   = VectorXd::Zero( _poly_num1D );

        posi_time(0) = 1.0;
        vel_time(0)  = std::numeric_limits<double>::min();
        
        for (int j = 1; j < _poly_num1D; j++){
            if (j == 1){
                posi_time(j) = pow(t, j);
                vel_time(j)  = 1;
            }else{
                posi_time(j) = pow(t, j);
                vel_time(j)  = j * pow(t, j - 1);                
            }
        }
        position(dim) = coeff.dot(posi_time);
        velocity(dim) = coeff.dot(vel_time);
    }
    return std::pair<Eigen::Vector3d, Eigen::Vector3d>(position, velocity);
}




