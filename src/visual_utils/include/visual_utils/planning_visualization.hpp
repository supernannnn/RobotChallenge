#ifndef _PLANNING_VISUALIZATION_HPP_
#define _PLANNING_VISUALIZATION_HPP_


#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>

using std::vector;

class PlanningVisualization{
private:

    ros::Publisher traj_pub;


public:
    typedef std::shared_ptr<PlanningVisualization> Ptr;
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    //构造函数
    PlanningVisualization(ros::NodeHandle &nh);

    void displayMarkerList( ros::Publisher &pub, 
                            const vector<Eigen::Vector3d> &list, 
                            double scale,
                            Eigen::Vector4d color, int id,  
                            bool show_sphere = true);

    void displayOptimalList(vector<Eigen::Vector3d> list, int id);

};




#endif