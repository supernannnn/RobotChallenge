/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <queue>

#include "input.h"
#include <Eigen/Dense>

/*控制器选择*/
enum CONTROL_METHOD{
	PID = 1,
	PV,
    DUALLOOP
};



struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d j;
	Eigen::Quaterniond q;
	double yaw;
	double yaw_rate;

	Desired_State_t(){};

	Desired_State_t(Odom_Data_t &odom)
		: p(odom.p),
		  v(Eigen::Vector3d::Zero()),
		  a(Eigen::Vector3d::Zero()),
		  j(Eigen::Vector3d::Zero()),
		  q(odom.q),
		  yaw(uav_utils::get_yaw_from_quaternion(odom.q)),
		  yaw_rate(0){};
};

struct Controller_Output_t
{

	// Orientation of the body frame with respect to the world frame
	Eigen::Quaterniond q;

	// Body rates in body frame
	Eigen::Vector3d bodyrates; // [rad/s]

	// Collective mass normalized thrust
	double thrust;

	//Eigen::Vector3d des_v_real;
};



class DualLoopPIDController {

private:
    void LimitOutput(Eigen::Vector3d& des_a){
        for (int i = 0; i <= 1; i++){
            if (des_a(i) < -1) des_a(i) = -1;
            if (des_a(i) > -1) des_a(i) = 1;
        }
    }


public:
    DualLoopPIDController(){}
    DualLoopPIDController(Parameter_t & parm_){
        kp_p_ = Eigen::Vector3d(parm_.dual_control.pos_Kpx, parm_.dual_control.pos_Kpy, parm_.dual_control.pos_Kpz);
        ki_p_ = Eigen::Vector3d(parm_.dual_control.pos_Kix, parm_.dual_control.pos_Kiy, parm_.dual_control.pos_Kiz);
        kd_p_ = Eigen::Vector3d(parm_.dual_control.pos_Kdx, parm_.dual_control.pos_Kdy, parm_.dual_control.pos_Kdz);


        kp_v_ = Eigen::Vector3d(parm_.dual_control.vel_Kpx, parm_.dual_control.vel_Kpy, parm_.dual_control.vel_Kpz);
        ki_v_ = Eigen::Vector3d(parm_.dual_control.vel_Kix, parm_.dual_control.vel_Kiy, parm_.dual_control.vel_Kiz);
        kd_v_ = Eigen::Vector3d(parm_.dual_control.vel_Kdx, parm_.dual_control.vel_Kdy, parm_.dual_control.vel_Kdz);


        integral_error_v_ = Eigen::Vector3d::Zero();
        integral_error_p_ = Eigen::Vector3d::Zero();
        last_error_v_ = Eigen::Vector3d::Zero();
        last_error_p_ = Eigen::Vector3d::Zero();

        kv_ = 1;
        ka_ = 1;
    }
    DualLoopPIDController(Eigen::Vector3d kp_v, Eigen::Vector3d ki_v, Eigen::Vector3d kd_v,
                          Eigen::Vector3d kp_p, Eigen::Vector3d ki_p, Eigen::Vector3d kd_p,
                          double kv,  double ka)
        : kp_v_(kp_v), ki_v_(ki_v), kd_v_(kd_v),
          kp_p_(kp_p), ki_p_(ki_p), kd_p_(kd_p),
          kv_(kv), ka_(ka),
          integral_error_v_(Eigen::Vector3d::Zero()),
          integral_error_p_(Eigen::Vector3d::Zero()),
          last_error_v_(Eigen::Vector3d::Zero()),
          last_error_p_(Eigen::Vector3d::Zero())
    {
        // Initialize any necessary variables here
    }

    Eigen::Vector3d computeControlSignal(const Eigen::Vector3d& des_p, const Eigen::Vector3d& odom_p, const Eigen::Vector3d& odom_v, 
                                  double dt) {
        // Position control
        Eigen::Vector3d error_p = des_p - odom_p;
        integral_error_p_ += error_p * dt;
        Eigen::Vector3d derivative_error_p = (error_p - last_error_p_) / dt;

        Eigen::Vector3d u_p = kp_p_.asDiagonal() * error_p + ki_p_.asDiagonal() * integral_error_p_ + kd_p_.asDiagonal() * derivative_error_p;
        last_error_p_ = error_p;

        // Velocity control
        // Eigen::Vector3d des_v = u_p;

        // Eigen::Vector3d error_v = des_v - odom_v;
        // integral_error_v_ += error_v * dt;
        // Eigen::Vector3d derivative_error_v = (error_v - last_error_v_) / dt;

        // Eigen::Vector3d u_v = kp_v_.asDiagonal() * error_v + ki_v_.asDiagonal() * integral_error_v_ + kd_v_.asDiagonal() * derivative_error_v;
        // last_error_v_ = error_v;

        // Combine velocity and position control
        // Eigen::Vector3d des_a = des_v * ka_ + u_v * kv_;

        Eigen::Vector3d des_a = u_p;

        LimitOutput(des_a);

        return des_a;
    }

private:
    Eigen::Vector3d kp_v_; // Velocity control PID gains
    Eigen::Vector3d ki_v_;
    Eigen::Vector3d kd_v_;

    Eigen::Vector3d kp_p_; // Position control PID gains
    Eigen::Vector3d ki_p_;
    Eigen::Vector3d kd_p_;

    double kv_; // Velocity feedforward gain
    double ka_; // Acceleration feedforward gain

    Eigen::Vector3d integral_error_v_; // Integral error for velocity control
    Eigen::Vector3d integral_error_p_; // Integral error for position control

    Eigen::Vector3d last_error_v_; // Previous error for velocity control
    Eigen::Vector3d last_error_p_; // Previous error for position control

};



class LinearControl
{
public:
    DualLoopPIDController dual_loop_controller;
    
	double u_k_x = 0,b_k_x = 0,b_k_1_x = 0,err_k_x = 0,err_k_1_x = 0;
    double u_k_y = 0,b_k_y = 0,b_k_1_y = 0,err_k_y = 0,err_k_1_y = 0;
    double u_k_z = 0,b_k_z = 0,b_k_1_z = 0,err_k_z = 0,err_k_1_z = 0;
	LinearControl(Parameter_t &);
	quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
		const Odom_Data_t &odom,
		const Imu_Data_t &imu, 
		Controller_Output_t &u);
	bool estimateThrustModel(const Eigen::Vector3d &est_v,
		const Parameter_t &param);
	void resetThrustMapping(void);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	ros::Time prev_time;
    ros::Time current_time; 
    Eigen::Vector3d p_pre;
	Parameter_t param_;
	quadrotor_msgs::Px4ctrlDebug debug_msg_;
	std::queue<std::pair<ros::Time, double>> timed_thrust_;
	static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;

	// Thrust-accel mapping params
	const double rho2_ = 0.998; // do not change
	double thr2acc_;
	double P_;

	double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);
	double fromQuaternion2yaw(Eigen::Quaterniond q);
};


#endif
