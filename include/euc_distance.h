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
#include <algorithm>

#include "nodes_structure.h"

#include "curvature_values.h"
#include "joint_node.h"     
#include "compute_q1.h"  
#include "compute_slopes.h"




float euc_dist(node &x1, node &x2){
    return pow((pow((x1.point.x-x2.point.x),2) + pow((x1.point.y-x2.point.y),2)),0.5);
}