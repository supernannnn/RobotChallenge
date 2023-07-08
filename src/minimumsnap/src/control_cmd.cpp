#include "minimumsnap/trajectory_generator.hpp"


void TRAJECTORY_GENERATOR::ControlCmdCallback(const ros::TimerEvent &e)
{
  if (!is_complete_Traj) return;
  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - Traj.start_time).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
  if (t_cur < Traj.time_duration && t_cur >= 0.0){
    auto info = getTrajInfo(Traj, t_cur);
    pos = info.first;
    vel = info.second;
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);

  }else if (t_cur >= Traj.time_duration){
    auto info = getTrajInfo(Traj, Traj.time_duration);
    pos = info.first;
    vel.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;
  }else{
    cout << "[Traj server]: invalid time." << endl;
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


std::pair<double, double> TRAJECTORY_GENERATOR::calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last){
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0);
  double yaw = 0;
  double yawdot = 0;

  Eigen::Vector3d dir = t_cur + time_forward_ <= Traj.time_duration ? getTrajInfo(Traj, t_cur + time_forward_).first - pos : getTrajInfo(Traj, Traj.time_duration).first - pos;
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







