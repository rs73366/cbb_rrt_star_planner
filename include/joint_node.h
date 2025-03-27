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
#include "bezier_points.h"


/**
 * @brief get the node where the joint is
 * @param seg is bezier segment input
 * @param t_c is the t value for connection
 * @param p is the number of points a segment is divided into
 */
node get_joint_node(bezier_seg &segment, double t){
    
    node pt; // it belongs to the newer segment though
    pt.point.x = pow((1-t),3)*(segment.p0.point.x) + 3*pow((1-t),2)*t*(segment.p1.point.x)+3*pow((1-t),1)*pow(t,2)*(segment.p2.point.x)+pow(t,3)*(segment.p3.point.x);
    pt.point.y = pow((1-t),3)*(segment.p0.point.y) + 3*pow((1-t),2)*t*(segment.p1.point.y)+3*pow((1-t),1)*pow(t,2)*(segment.p2.point.y)+pow(t,3)*(segment.p3.point.y);
    pt.point.z = 0;
    pt.parent = segment.index; // input segment is taken as the parent of the joint node!
    
    return pt;
}