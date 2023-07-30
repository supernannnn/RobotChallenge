#ifndef _TRAJECTORY_GENERATOR_HPP_
#define _TRAJECTORY_GENERATOR_HPP_

#include <tf/tf.h>
#include <stdio.h>
#include <string>
#include <thread>
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <limits>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>

#include <sensor_msgs/Joy.h>
#include <algorithm>
#include <Eigen/Eigen>
#include <vector>


#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>

#include <visual_utils/planning_visualization.hpp>

#include <tf/transform_broadcaster.h>

using namespace std;
using namespace Eigen;



/*任务状态*/
enum STATE{
    INIT = 0,                       //初始化
    WAIT_TAKE_OFF_FINISHED ,     //等待起飞完成              
    FLY_TO_STRAT,                   //飞向起点
    FOLLOW_TRAJECTORY,              //执行轨迹跟踪
    LANDING                         //降落
};


struct MiniSnapTraj{
    ros::Time       start_time;         //轨迹起始时间
    int             traj_nums;          //分段轨迹数量
    double          time_duration;      //轨迹总时间
    Eigen::VectorXd poly_time;          //每段轨迹的结束时间点，相对轨迹起始点来说
    Eigen::MatrixXd poly_coeff;         //多项式轨迹系数矩阵
    MiniSnapTraj(){};
    MiniSnapTraj(ros::Time _start_time, Eigen::VectorXd _poly_time, Eigen::MatrixXd _poly_coeff){
        start_time  = _start_time;
        traj_nums   = _poly_time.size();
        poly_time.resize(traj_nums);
        double sum = 0;
        for (int i = 0; i < _poly_time.size(); i++){
            sum += _poly_time(i);
            poly_time[i] = sum;
        }
        poly_coeff  = _poly_coeff;
        for (int i = 0; i < _poly_time.size(); i++)  time_duration += _poly_time(i);
    };
    ~MiniSnapTraj(){};
};


class TRAJECTORY_GENERATOR{
private:

    // Param from launch file
    double _vis_traj_width;
    double _Vel, _Acc;
    int _dev_order, _min_order;

    int _poly_num1D;

    Vector3d _startPos  = Vector3d::Zero();
    Vector3d _startVel  = Vector3d::Zero();
    Vector3d _startAcc  = Vector3d::Zero();
    Vector3d _endVel    = Vector3d::Zero();
    Vector3d _endAcc    = Vector3d::Zero();

    //参考柱子坐标
    Eigen::Vector3d pillar1    = Vector3d::Zero();
    Eigen::Vector3d pillar2    = Vector3d::Zero();

    //八字banjing
    double radius;

    ros::Timer ControlCmdCallback_Timer;
    ros::Timer Transformer_Timer;
    ros::Timer FSM_Timer;
    tf::TransformBroadcaster broadcaster;

    /*状态机当前状态*/
    int state;

    MiniSnapTraj Traj_start;
    MiniSnapTraj Traj;
    vector<Eigen::Vector3d> traj, start_traj;
    vector<Eigen::Vector3d> waypoints_visual;
    PlanningVisualization::Ptr trajVisual_;
    nav_msgs::Path trajPath;
    ros::Publisher trajPath_pub;
    ros::Publisher controlCmd_pub;
    ros::Publisher land_pub;
    ros::Subscriber odom_sub;
    Eigen::Vector3d odom_pos;
    bool has_odom;
    bool finished_trajectory, finish_start_trajectory, eight_follow_finished;
    int tra_id;
    int eight_times;
    quadrotor_msgs::PositionCommand posiCmd;

    bool is_complete_Traj, is_complete_Start_Traj;
    double last_yaw_, last_yaw_dot_;
    double time_forward_;
    
    bool is_land;

    //通过柱子坐标得到轨迹的waypoints
    void GetWaypoints();

    //路标回调函数
    void waypointsCallback(const nav_msgs::Path& msg);

    //轨迹生成函数
    void trajGeneration(Eigen::MatrixXd& path);

    //时间分配,为每一段轨迹合理分配时间
    Eigen::VectorXd timeAllocation(Eigen::MatrixXd& Path);

    //计算多项式轨迹在给定时间参数t处的位置与速度
    std::pair<Eigen::Vector3d, Eigen::Vector3d>  getTrajInfo(const MiniSnapTraj& traj, double t);

    //根据多项式系数和时间分配得到三维坐标点
    void getVisual(Eigen::MatrixXd& polyCoeff, Eigen::VectorXd& time, MiniSnapTraj& trajectory);

    //轨迹三维坐标点得到轨迹每个点的偏航角
    std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last, MiniSnapTraj& trajectory);
   
    void ControlCmdCallback(const ros::TimerEvent &e);

    void OdomCallback(const nav_msgs::OdometryConstPtr msg);

    /*TF树坐标变换关系*/
    void TransformerCallback(const ros::TimerEvent &e);

    /*控制状态机*/
    void ControlFSM(const ros::TimerEvent &e);

    /*根据轨迹解析控制指令*/
    void ControlParse(MiniSnapTraj& trajectory, ros::Time start_time, bool init, bool& finished);

    inline double Distance_of_waypoints(Eigen::Vector3d& Point1, Eigen::Vector3d& Point2);


public:
    TRAJECTORY_GENERATOR(/* argvs */){
        is_complete_Traj = false;
        has_odom = false;
        is_complete_Start_Traj = false;
        state = INIT;
        finished_trajectory = false;
        finish_start_trajectory = false;
        is_land = false;
        eight_follow_finished = false;
        tra_id = 0;
    }
    ~TRAJECTORY_GENERATOR(){

    }
    void init(ros::NodeHandle& nh);
};

inline double TRAJECTORY_GENERATOR::Distance_of_waypoints(Eigen::Vector3d& Point1, Eigen::Vector3d& Point2) {
    double x_diff = Point2.x() - Point1.x();
    double y_diff = Point2.y() - Point1.y();
    double z_diff = Point2.z() - Point1.z();
    return std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
}


class TRAJECTORY_GENERATOR_WAYPOINT  
{
private:
    // double _qp_cost;
    Eigen::MatrixXd _Q;
    Eigen::MatrixXd _M;
    Eigen::MatrixXd _Ct;
    
    Eigen::VectorXd _Px, _Py, _Pz;

    Eigen::MatrixXd getQ(const int p_num1d,
                         const int order, 
                         const Eigen::VectorXd &Time, 
                         const int seg_index);

    Eigen::MatrixXd getM(const int p_num1d,
                         const int order, 
                         const Eigen::VectorXd &Time, 
                         const int seg_index);

    Eigen::MatrixXd getCt(const int seg_num, const int d_order);

    Eigen::VectorXd closedFormCalCoeff1D(const Eigen::MatrixXd &Q,
                                         const Eigen::MatrixXd &M,
                                         const Eigen::MatrixXd &Ct,
                                         const Eigen::VectorXd &WayPoints1D,
                                         const Eigen::VectorXd &StartState1D,
                                         const Eigen::VectorXd &EndState1D,
                                         const int seg_num, 
                                         const int d_order);

public:
    TRAJECTORY_GENERATOR_WAYPOINT();

    ~TRAJECTORY_GENERATOR_WAYPOINT();

    Eigen::MatrixXd PolyQPGeneration(
        const int order,
        const Eigen::MatrixXd &Path,
        const Eigen::MatrixXd &Vel,
        const Eigen::MatrixXd &Acc,
        const Eigen::VectorXd &Time);

    int Factorial(int x);
};



#endif




