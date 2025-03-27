
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
#include "curvature_values.h"
#include "bezier_points.h"


/**
 * @brief Get the curvature at a specific point
 * 
 * @param segment 
 * @param t 
 * @param p 
 * @return ** double 
 */
double  get_curvature(bezier_seg &segment, double t, double p){
    //std::cout<<"curvature at t file running"<<std::endl;
    
    int i = t*p;
    return segment.curvature_vals[i];

}