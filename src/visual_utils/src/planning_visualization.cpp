#include <visual_utils/planning_visualization.hpp>

using std::cout;
using std::endl;


PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh){
    traj_pub = nh.advertise<visualization_msgs::Marker>("mini_snap_traj", 1);
}

void PlanningVisualization::displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                                            Eigen::Vector4d color, int id, bool show_sphere /* = true */ )
{
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
        pt.x = list[i](0);
        pt.y = list[i](1);
        pt.z = list[i](2);
        //if (show_sphere) sphere.points.push_back(pt);
        line_strip.points.push_back(pt);
    }
    //if (show_sphere) pub.publish(sphere);
    pub.publish(line_strip);
}


void PlanningVisualization::displayOptimalList(vector<Eigen::Vector3d> list, int id){
    // if (traj_pub.getNumSubscribers() == 0){
    //   return;
    // }
    Eigen::Vector4d color(1, 0, 0, 1);
    displayMarkerList(traj_pub, list, 0.15, color, id);
}



