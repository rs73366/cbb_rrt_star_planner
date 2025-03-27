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
#include "slope_at_t.h"


void compute_slopes(bezier_seg &segment){
    //std::cout<<"compute slopes file running"<<std::endl;
    for(int s = 0; s<segment.parts;++s){
        float t_s = (s*1.0)/segment.parts;
        segment.slope_vals.push_back(get_slope(segment,t_s));
    }
}