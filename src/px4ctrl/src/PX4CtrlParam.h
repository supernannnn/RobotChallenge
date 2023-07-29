#ifndef __PX4CTRLPARAM_H
#define __PX4CTRLPARAM_H

#include <ros/ros.h>

class Parameter_t
{
public:
	struct Gain
	{
		double Kp0, Kp1, Kp2;
		double Kv0, Kv1, Kv2;
		double Kvi0, Kvi1, Kvi2;
		double Kvd0, Kvd1, Kvd2;
		double KAngR, KAngP, KAngY;
	};

	struct PositionGain
	{
		double Kpx, Kpy, Kpz;
		double Kix, Kiy, Kiz;
		double Kdx, Kdy, Kdz;
	};


	struct DualControl{
		/*位置环参数*/
		double pos_Kpx, pos_Kpy, pos_Kpz;
		double pos_Kix, pos_Kiy, pos_Kiz;
		double pos_Kdx, pos_Kdy, pos_Kdz;

		/*速度环参数*/
		double vel_Kpx, vel_Kpy, vel_Kpz;
		double vel_Kix, vel_Kiy, vel_Kiz;
		double vel_Kdx, vel_Kdy, vel_Kdz;
	};


	struct RotorDrag
	{
		double x, y, z;
		double k_thrust_horz;
	};

	struct MsgTimeout
	{
		double odom;
		double rc;
		double cmd;
		double imu;
		double bat;
	};

	struct ThrustMapping
	{
		bool print_val;
		double K1;
		double K2;
		double K3;
		bool accurate_thrust_model;
		double hover_percentage;
	};

	struct RCReverse
	{
		bool roll;
		bool pitch;
		bool yaw;
		bool throttle;
	};

	struct AutoTakeoffLand
	{
		bool enable;
		bool enable_auto_arm;
		bool no_RC;
		double height;
		double speed;
	};

	DualControl dual_control;
	Gain gain;
	PositionGain position_gain;
	RotorDrag rt_drag;
	MsgTimeout msg_timeout;
	RCReverse rc_reverse;
	ThrustMapping thr_map;
	AutoTakeoffLand takeoff_land;

	int pose_solver;
	double mass;
	double gra;
	double max_angle;
	double ctrl_freq_max;
	double max_manual_vel;
	double low_voltage;

	bool use_bodyrate_ctrl;
	// bool print_dbg;

	int control_method;

	Parameter_t();
	void config_from_ros_handle(const ros::NodeHandle &nh);
	void config_full_thrust(double hov);

private:
	template <typename TName, typename TVal>
	void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
	{
		if (nh.getParam(name, val))
		{
			// pass
		}
		else
		{
			ROS_ERROR_STREAM("Read param: " << name << " failed.");
			ROS_BREAK();
		}
	};
};

#endif