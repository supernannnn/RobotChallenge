#include "PX4CtrlParam.h"

Parameter_t::Parameter_t()
{
}

void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh)
{
	read_essential_param(nh, "gain/Kp0", gain.Kp0);
	read_essential_param(nh, "gain/Kp1", gain.Kp1);
	read_essential_param(nh, "gain/Kp2", gain.Kp2);
	read_essential_param(nh, "gain/Kv0", gain.Kv0);
	read_essential_param(nh, "gain/Kv1", gain.Kv1);
	read_essential_param(nh, "gain/Kv2", gain.Kv2);
	read_essential_param(nh, "gain/Kvi0", gain.Kvi0);
	read_essential_param(nh, "gain/Kvi1", gain.Kvi1);
	read_essential_param(nh, "gain/Kvi2", gain.Kvi2);
	read_essential_param(nh, "gain/KAngR", gain.KAngR);
	read_essential_param(nh, "gain/KAngP", gain.KAngP);
	read_essential_param(nh, "gain/KAngY", gain.KAngY);


	read_essential_param(nh, "position_gain/Kpx", position_gain.Kpx);
	read_essential_param(nh, "position_gain/Kpy", position_gain.Kpy);
	read_essential_param(nh, "position_gain/Kpz", position_gain.Kpz);

	read_essential_param(nh, "position_gain/Kix", position_gain.Kix);
	read_essential_param(nh, "position_gain/Kiy", position_gain.Kiy);
	read_essential_param(nh, "position_gain/Kiz", position_gain.Kiz);


	/*数字PID控制器参数*/
	read_essential_param(nh, "position_gain/Kdx", position_gain.Kdx);
	read_essential_param(nh, "position_gain/Kdy", position_gain.Kdy);
	read_essential_param(nh, "position_gain/Kdz", position_gain.Kdz);

	read_essential_param(nh, "control_method", control_method);	
	if (control_method == 1){
		ROS_WARN("USE PID control!");
	}else if (control_method == 2){
		ROS_WARN("USE PV control!");
	}else {
		ROS_WARN("USE DualLoopPIDcontrol!");
	}
	/*******************/


	/*位置速度双闭环PID控制器参数*/
									/*位置环*/
	read_essential_param(nh, "dual_control/pos_Kpx", dual_control.pos_Kpx);
	read_essential_param(nh, "dual_control/pos_Kpy", dual_control.pos_Kpy);
	read_essential_param(nh, "dual_control/pos_Kpz", dual_control.pos_Kpz);

	read_essential_param(nh, "dual_control/pos_Kix", dual_control.pos_Kix);
	read_essential_param(nh, "dual_control/pos_Kiy", dual_control.pos_Kiy);
	read_essential_param(nh, "dual_control/pos_Kiz", dual_control.pos_Kiz);

	read_essential_param(nh, "dual_control/pos_Kdx", dual_control.pos_Kdx);
	read_essential_param(nh, "dual_control/pos_Kdy", dual_control.pos_Kdy);
	read_essential_param(nh, "dual_control/pos_Kdz", dual_control.pos_Kdz);

								/*速度环*/
	read_essential_param(nh, "dual_control/vel_Kpx", dual_control.vel_Kpx);
	read_essential_param(nh, "dual_control/vel_Kpy", dual_control.vel_Kpy);
	read_essential_param(nh, "dual_control/vel_Kpz", dual_control.vel_Kpz);

	read_essential_param(nh, "dual_control/vel_Kix", dual_control.vel_Kix);
	read_essential_param(nh, "dual_control/vel_Kiy", dual_control.vel_Kiy);
	read_essential_param(nh, "dual_control/vel_Kiz", dual_control.vel_Kiz);

	read_essential_param(nh, "dual_control/vel_Kdx", dual_control.vel_Kdx);
	read_essential_param(nh, "dual_control/vel_Kdy", dual_control.vel_Kdy);
	read_essential_param(nh, "dual_control/vel_Kdz", dual_control.vel_Kdz);	



	read_essential_param(nh, "rotor_drag/x", rt_drag.x);
	read_essential_param(nh, "rotor_drag/y", rt_drag.y);
	read_essential_param(nh, "rotor_drag/z", rt_drag.z);
	read_essential_param(nh, "rotor_drag/k_thrust_horz", rt_drag.k_thrust_horz);

	read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
	read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
	read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);
	read_essential_param(nh, "msg_timeout/imu", msg_timeout.imu);
	read_essential_param(nh, "msg_timeout/bat", msg_timeout.bat);

	read_essential_param(nh, "pose_solver", pose_solver);
	read_essential_param(nh, "mass", mass);
	read_essential_param(nh, "gra", gra);
	read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max);
	read_essential_param(nh, "use_bodyrate_ctrl", use_bodyrate_ctrl);
	read_essential_param(nh, "max_manual_vel", max_manual_vel);
	read_essential_param(nh, "max_angle", max_angle);
	read_essential_param(nh, "low_voltage", low_voltage);

	read_essential_param(nh, "rc_reverse/roll", rc_reverse.roll);
	read_essential_param(nh, "rc_reverse/pitch", rc_reverse.pitch);
	read_essential_param(nh, "rc_reverse/yaw", rc_reverse.yaw);
	read_essential_param(nh, "rc_reverse/throttle", rc_reverse.throttle);

	read_essential_param(nh, "auto_takeoff_land/enable", takeoff_land.enable);
    read_essential_param(nh, "auto_takeoff_land/enable_auto_arm", takeoff_land.enable_auto_arm);
    read_essential_param(nh, "auto_takeoff_land/no_RC", takeoff_land.no_RC);
	read_essential_param(nh, "auto_takeoff_land/takeoff_height", takeoff_land.height);
	read_essential_param(nh, "auto_takeoff_land/takeoff_land_speed", takeoff_land.speed);

	read_essential_param(nh, "thrust_model/print_value", thr_map.print_val);
	read_essential_param(nh, "thrust_model/K1", thr_map.K1);
	read_essential_param(nh, "thrust_model/K2", thr_map.K2);
	read_essential_param(nh, "thrust_model/K3", thr_map.K3);
	read_essential_param(nh, "thrust_model/accurate_thrust_model", thr_map.accurate_thrust_model);
	read_essential_param(nh, "thrust_model/hover_percentage", thr_map.hover_percentage);
	

	max_angle /= (180.0 / M_PI);

	if ( takeoff_land.enable_auto_arm && !takeoff_land.enable )
	{
		takeoff_land.enable_auto_arm = false;
		ROS_ERROR("\"enable_auto_arm\" is only allowd with \"auto_takeoff_land\" enabled.");
	}
	if ( takeoff_land.no_RC && (!takeoff_land.enable_auto_arm || !takeoff_land.enable) )
	{
		takeoff_land.no_RC = false;
		ROS_ERROR("\"no_RC\" is only allowd with both \"auto_takeoff_land\" and \"enable_auto_arm\" enabled.");
	}

	if ( thr_map.print_val )
	{
		ROS_WARN("You should disable \"print_value\" if you are in regular usage.");
	}
};

// void Parameter_t::config_full_thrust(double hov)
// {
// 	full_thrust = mass * gra / hov;
// };
