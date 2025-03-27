
#pragma once
#include <algorithm>
#include <angles/angles.h>
#include <math.h>
#include <vector>
#include <stdlib.h>

#include <ros/ros.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <nav_core/base_global_planner.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>

#include <vector>
#include <cmath>

#include "nodes_structure.h"








/**
 * @brief Get points in a bezier sgment 
 * 
 * @param segment is bezier segment
 * @param p the number of points to divide the segment in
 * @return the set of points
 */

std::vector<geometry_msgs::Point> get_bez_points(bezier_seg &segment, int p){
    //std::cout<<"bezier points file running"<<std::endl;
    std::vector<geometry_msgs::Point> bezier_curve_points;
    bezier_curve_points.resize(p+1);
    for (int i = 0; i<=p; ++i){
        //std::cout<<"the value of i in bez_points is: "<<i<<std::endl;


        
        double t = (i*1.0)/p;
        //std::cout<<t<<std::endl;
        bezier_curve_points[i].x = pow((1-t),3)*(segment.p0.point.x) + 3*pow((1-t),2)*t*(segment.p1.point.x)+3*pow((1-t),1)*pow(t,2)*(segment.p2.point.x)+pow(t,3)*(segment.p3.point.x);
        bezier_curve_points[i].y = pow((1-t),3)*(segment.p0.point.y) + 3*pow((1-t),2)*t*(segment.p1.point.y)+3*pow((1-t),1)*pow(t,2)*(segment.p2.point.y)+pow(t,3)*(segment.p3.point.y);
        bezier_curve_points[i].z = 0;
        //std::cout<<"the bezier point for iteration: "<<i<<std::endl;
        //std::cout<<bezier_curve_points[i].x<<","<<bezier_curve_points[i].y<<","<<bezier_curve_points[i].z<<std::endl;
    }
    return bezier_curve_points;
}