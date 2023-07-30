#include "controller.h"

using namespace std;


double LinearControl::smoothfilter(queue<double>& filter_win, double data) {
  filter_win.push(data);

  if (filter_win.size() == max_window_len) {
    filter_win.pop();
  }
  auto tmp_queue = filter_win;
  double sum = 0.0;
  while (!tmp_queue.empty()){
    sum += tmp_queue.front();
    tmp_queue.pop();
  } 
  return sum / filter_win.size();
}


double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

LinearControl::LinearControl(Parameter_t &param) : param_(param)
{
  dual_loop_controller = DualLoopPIDController(param);
  resetThrustMapping();
  p_pre = Eigen::Vector3d::Zero();
  max_window_len = 5;
}

/* 
  compute u.thrust and u.q, controller gains and other parameters are in param_ 
*/
quadrotor_msgs::Px4ctrlDebug
LinearControl::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u)
{

  Eigen::Vector3d des_acc(0.0, 0.0, 0.0);

  /****************************双闭环串级PID控制器******************************/
  if (param_.control_method == DUALLOOP) {
    current_time = ros::Time::now();
    double dt = (current_time - prev_time).toSec();
    des_acc = dual_loop_controller.computeControlSignal(des.p, odom.p, odom.v, dt);
    prev_time = current_time;
  }
  /***************************************************************************/


  /*****************数字PID****************/
  if (param_.control_method == PID) {
    Eigen::Vector3d posiERR = des.p - odom.p;
    double q0_x = param_.position_gain.Kpx + param_.position_gain.Kix + param_.position_gain.Kdx;
    double q1_x = -(param_.position_gain.Kpx) - 2 * param_.position_gain.Kdx;
    double q2_x = param_.position_gain.Kdx;

    double q0_y = param_.position_gain.Kpy + param_.position_gain.Kiy + param_.position_gain.Kdy;
    double q1_y = -(param_.position_gain.Kpy) - 2 * param_.position_gain.Kdy;
    double q2_y = param_.position_gain.Kdy;

    double q0_z = param_.position_gain.Kpz + param_.position_gain.Kiz + param_.position_gain.Kdz;
    double q1_z = -(param_.position_gain.Kpz) - 2 * param_.position_gain.Kdz;
    double q2_z = param_.position_gain.Kdz;

    u_k_x = q0_x * posiERR(0) + b_k_1_x;

    b_k_x = u_k_x + q1_x * posiERR(0) + q2_x * err_k_1_x;

    err_k_1_x = posiERR(0);

    b_k_1_x = b_k_x;


    u_k_y = q0_y * posiERR(1) + b_k_1_y;

    b_k_y = u_k_y + q1_y * posiERR(1) + q2_y * err_k_1_y;

    err_k_1_y = posiERR(1);

    b_k_1_y = b_k_y;    


    u_k_z = q0_z * posiERR(2) + b_k_1_z;

    b_k_z = u_k_z + q1_z * posiERR(2) + q2_z * err_k_1_z;

    err_k_1_z = posiERR(2);

    b_k_1_z = b_k_z; 

    Eigen::Vector3d PositionControl(u_k_x, u_k_y, u_k_z);

    des_acc = PositionControl;
  }

  /**************************************************/


  /****************************位置速度P控制器**********************************/
  if (param_.control_method == PV) {
    Eigen::Vector3d Kp,Kv;
    Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
    Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;

    current_time = ros::Time::now();
    double dt = (current_time - prev_time).toSec();

    des_acc = des.a + Kv.asDiagonal() * (des.v - (odom.p - p_pre) / dt) + Kp.asDiagonal() * (des.p - odom.p);
    p_pre = odom.p;
	  prev_time = current_time;  
}
  /***************************************************************************/

  des_acc += Eigen::Vector3d(0,0,param_.gra);

  u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
  double roll,pitch,yaw,yaw_imu;
  double yaw_odom = fromQuaternion2yaw(odom.q);
  double sin = std::sin(yaw_odom);
  double cos = std::cos(yaw_odom);

  /****对z轴的控制量进行异常值滤波****/
  double tmp_acc_z = des_acc(2);
  static double pre_acc = des_acc(2);
  if (abs(pre_acc - 9.81) < 1 &&  abs(tmp_acc_z - pre_acc) > 2.5) {
      tmp_acc_z = pre_acc;
  }else {
    tmp_acc_z = smoothfilter(filter_win, des_acc(2));
    pre_acc = tmp_acc_z;
  }
  des_acc(2) = tmp_acc_z;
  /*******************************/

  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);


  roll = (des_acc(0) * sin - des_acc(1) * cos )/ param_.gra;
  pitch = (des_acc(0) * cos + des_acc(1) * sin )/ param_.gra;
  // yaw = fromQuaternion2yaw(des.q);
  yaw_imu = fromQuaternion2yaw(imu.q);
  // Eigen::Quaterniond q = Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())
  //   * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX())
  //   * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY());
  Eigen::Quaterniond q = Eigen::AngleAxisd(des.yaw,Eigen::Vector3d::UnitZ())
    * Eigen::AngleAxisd(pitch,Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(roll,Eigen::Vector3d::UnitX());
  u.q = imu.q * odom.q.inverse() * q;


  /* WRITE YOUR CODE HERE */

  //used for debug
  // debug_msg_.des_p_x = des.p(0);
  // debug_msg_.des_p_y = des.p(1);
  // debug_msg_.des_p_z = des.p(2);
  
  debug_msg_.des_v_x = roll;
  debug_msg_.des_v_y = pitch;
  debug_msg_.des_v_z = des.v(2);
  

  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
  
  debug_msg_.des_thr = u.thrust;
  
  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}

/*
  compute throttle percentage 
*/
double 
LinearControl::computeDesiredCollectiveThrustSignal(
    const Eigen::Vector3d &des_acc)
{
  double throttle_percentage(0.0);
  
  /* compute throttle, thr2acc has been estimated before */
  throttle_percentage = des_acc(2) / thr2acc_;

  return throttle_percentage;
}

bool 
LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a,
    const Parameter_t &param)
{
  ros::Time t_now = ros::Time::now();
  while (timed_thrust_.size() >= 1)
  {
    // Choose data before 35~45ms ago
    std::pair<ros::Time, double> t_t = timed_thrust_.front();
    double time_passed = (t_now - t_t.first).toSec();
    if (time_passed > 0.045) // 45ms
    {
      // printf("continue, time_passed=%f\n", time_passed);
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < 0.035) // 35ms
    {
      // printf("skip, time_passed=%f\n", time_passed);
      return false;
    }

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/
    double thr = t_t.second;
    timed_thrust_.pop();
    
    /***********************************/
    /* Model: est_a(2) = thr1acc_ * thr */
    /***********************************/
    double gamma = 1 / (rho2_ + thr * P_ * thr);
    double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1 - K * thr) * P_ / rho2_;
    //printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
    //fflush(stdout);

    // debug_msg_.thr2acc = thr2acc_;
    return true;
  }
  return false;
}

void 
LinearControl::resetThrustMapping(void)
{
  thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
  P_ = 1e6;
}







