#include "minimumsnap/trajectory_generator.hpp"


void TRAJECTORY_GENERATOR::ControlFSM(const ros::TimerEvent &e){

  int current_state = state;
  if ((!is_complete_Traj || !has_odom) && odom_pos(2) < 1) {
    state = WAIT_TAKE_OFF_FINISHED;

  }else if (is_complete_Traj && has_odom && abs(odom_pos(2) - 1) < 0.1 && !is_complete_Start_Traj) {
    Eigen::MatrixXd waypoints(2, 3);
    Eigen::Vector3d statr_pos(odom_pos(0), odom_pos(1), 1);
    waypoints.row(0) = statr_pos;
    waypoints.row(1) = statr_pos + (waypoints_visual[0] - statr_pos) * 0.9;      //起飞轨迹留一点裕量，以适应无人机的惯性
    TRAJECTORY_GENERATOR_WAYPOINT trajectoryGeneratorWaypoint;
    MatrixXd vel  = MatrixXd::Zero(2, 3); 
    MatrixXd acc  = MatrixXd::Zero(2, 3);
    vel.row(0)  = _startVel;
    vel.row(1)  = _endVel;
    acc.row(0)  = _startAcc;
    acc.row(1)  = _endAcc;
    ros::Time time_start = ros::Time::now();
    auto _polyTime  = timeAllocation(waypoints);
    auto _polyCoeff = trajectoryGeneratorWaypoint.PolyQPGeneration(_dev_order, waypoints, vel, acc, _polyTime);
    Traj_start = MiniSnapTraj(time_start, _polyTime, _polyCoeff);
    ros::Time time_end = ros::Time::now();
    ROS_WARN("Time consumed in trajectory generation is %f ms", (time_end - time_start).toSec() * 1000.0); 
    ROS_WARN("The duration of trajectory is %f s", Traj_start.time_duration); 
    getVisual(_polyCoeff, _polyTime, Traj_start);
    is_complete_Start_Traj = true;
    //等待3s
    ros::Rate loop0(100);
    for (int i = 0; i < 300; i++) loop0.sleep();
    state = FLY_TO_STRAT;

  }else if (finish_start_trajectory && is_complete_Traj && has_odom && !eight_follow_finished) {
    //等待3s
    if (state != FOLLOW_TRAJECTORY) {
      ros::Rate loop1(100);
      for (int i = 0; i < 300; i++) loop1.sleep();
    }
    // ros::Rate loop1(100);
    // for (int i = 0; i < 300; i++) loop1.sleep();
    state = FOLLOW_TRAJECTORY;
     
  }else if (eight_follow_finished){
    //等待1s
    ros::Rate loop2(100);
    for (int i = 0; i < 100; i++) loop2.sleep();    
    state = LANDING;
  }

  // 检查状态转换，并使用 `ROS_WARN` 只在状态改变时打印。
  if (current_state != state) {
    switch (state) {
      case WAIT_TAKE_OFF_FINISHED:
        ROS_WARN("STATE: WAIT_TAKE_OFF_FINISHED");
        break;
      case FLY_TO_STRAT:
        ROS_WARN("STATE: FLY_TO_STRAT");
        break;
      case FOLLOW_TRAJECTORY:
        ROS_WARN("STATE: FOLLOW_TRAJECTORY");
        break;
      case LANDING:
        ROS_WARN("STATE: LANDING");
        break;
      default:
        break;
    }
  }


}


void TRAJECTORY_GENERATOR::ControlCmdCallback(const ros::TimerEvent &e)
{
  if (state == STATE::WAIT_TAKE_OFF_FINISHED) {
    return;
  }
  if (state == STATE::FLY_TO_STRAT) {
    static ros::Time fisrt_tra_start_time = ros::Time::now();
    static bool first_init = true;
    ControlParse(Traj_start, fisrt_tra_start_time, first_init, finish_start_trajectory);
    first_init = false;
  }

  if (state == STATE::FOLLOW_TRAJECTORY) {
    static ros::Time second_tra_start_time = ros::Time::now();
    static bool second_init = true;
    //更新下一段八字轨迹的起始时间
    ros::Time t_now = ros::Time::now();
    //如果当前时间已经跟踪完轨迹进行下一个八字的跟踪
    if ((t_now - second_tra_start_time).toSec() >= (Traj.time_duration + 1) && finished_trajectory) {
      //等待1s,给控制器一点缓冲的时间
      ros::Rate loop3(100);
      for (int i = 0; i < 100; i++) loop3.sleep();

      //更新参数与标志位
      second_tra_start_time = ros::Time::now();
      second_init = true;
      finished_trajectory = false;
      tra_id++;

      //如果绕圈次数达到预设值，退出轨迹跟踪模式
      if (tra_id == eight_times) eight_follow_finished = true;   
    }
    ControlParse(Traj, second_tra_start_time, second_init, finished_trajectory);
    second_init = false;
  }

  if (state == STATE::LANDING) {
    if (!is_land){
    quadrotor_msgs::TakeoffLand cmd;
    cmd.takeoff_land_cmd = 2;
    land_pub.publish(cmd);
    is_land = true;
    }
  }
}


void TRAJECTORY_GENERATOR::ControlParse(MiniSnapTraj& trajectory, ros::Time start_time, bool init, bool& finished) {
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time).toSec();

    Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero());
    std::pair<double, double> yaw_yawdot(0, 0); 
    static ros::Time time_last = ros::Time::now();

    if (init) {
      time_last = ros::Time::now();
    } 
    if (t_cur < trajectory.time_duration && t_cur >= 0.0){
      auto info = getTrajInfo(trajectory, t_cur);
      pos = info.first;
      vel = info.second;
      yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last, trajectory);

    }else if (t_cur >= trajectory.time_duration){
      auto info = getTrajInfo(trajectory, trajectory.time_duration);
      pos = info.first;
      vel.setZero();
      yaw_yawdot.first = 3.14 / 4;
      yaw_yawdot.second = 0;
      finished = true;

    }else{
      cout << "[trajectory server]: invalid time." << endl;
    }

    time_last = time_now;

    posiCmd.header.stamp = time_now;
    posiCmd.header.frame_id = "world";
    posiCmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    posiCmd.trajectory_id = 1;

    posiCmd.position.x = pos(0);
    posiCmd.position.y = pos(1);
    posiCmd.position.z = pos(2);

    posiCmd.velocity.x = vel(0);
    posiCmd.velocity.y = vel(1);
    posiCmd.velocity.z = vel(2);

    posiCmd.yaw = yaw_yawdot.first;
    posiCmd.yaw_dot = yaw_yawdot.second;

    last_yaw_ = posiCmd.yaw;
    controlCmd_pub.publish(posiCmd);
}


std::pair<double, double> TRAJECTORY_GENERATOR::calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last, MiniSnapTraj& trajectory){
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= trajectory.time_duration ? getTrajInfo(trajectory, t_cur + time_forward_).first - pos : getTrajInfo(trajectory, trajectory.time_duration).first - pos;

  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }

  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;    
}







