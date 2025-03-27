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

double get_slope(bezier_seg &segment, double t){

    //std::cout<<"slope at t file running"<<std::endl;
    int i = t*segment.parts;
    int j = t*segment.parts + 1;
    double dy = segment.bezier_seg_points[j].y - segment.bezier_seg_points[i].y;
    double dx = segment.bezier_seg_points[j].x - segment.bezier_seg_points[i].x;
    // return angle between -pi to +pi in radiens
    return atan2(dy,dx);
}