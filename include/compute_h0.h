
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


double get_h0(bezier_seg &segment,node &q0, node &q2){
    //std::cout<<"compute h0 file running"<<std::endl;
    //std::cout<<"the value of slope is: "<<std::endl;
    //std::cout<<tan(segment.slope_vals[segment.t_c*segment.parts])<<std::endl;
    double m_0 = tan(segment.slope_vals[segment.t_c*segment.parts]);
    //std::cout<<"the val of h0  is: "<<std::endl;
    double h_0 = abs((((q2.point.y-q0.point.y) - m_0*(q2.point.x-q0.point.x))/(pow((1+pow(m_0,2)),0.5))));

    //std::cout<<"the curvature values in get_h_0 fn is"<<","<<segment.curvature_vals[segment.t_c*segment.parts]<<std::endl;
    if(segment.curvature_vals[segment.t_c*segment.parts] < 0.001){
        h_0 = 0;
    }
    if(h_0 <=0.001){
        h_0 = 0;
    }
    return h_0;

}