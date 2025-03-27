

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


#
 struct node
{
    geometry_msgs::Point point; // x,y,z of a node
    geometry_msgs::Pose orientation; // describes the heading of robot !
    // x & y cordinate of node and cost of node from initial point
    //const node* parent = NULL; // pointer to parent node, initialized with NULL 
    int index, parent = 0;
    double cost = 0; // every node is initialized with cost zero
    bool active = true; //status of node! whether it is active or a dead branch, default
};
// bezier segment 
struct bezier_seg{
    node p0,p1,p2,p3;
    
    std::vector<geometry_msgs::Point> bezier_seg_points; // points in the bezier segment
    std::vector<double> curvature_vals; // curvature values till i-2 points
    std::vector<double> slope_vals;
    std::vector<int> children; // indices of children nodes
    double max_curvature; // maximum curvature value of entire segment 
    double t_c = 0.4; // point of connection to children segment which defaults to 0.4
    int index, parent; // every segment will have an index and a parent!
    int parts = 40; // number of parts to divide the segment into. defaults to 20
    bool active = true; // status of segment! whether it is active or a dead branch, defaults to true.

    // change the status of node as active or not
    void change_status(bezier_seg &segment,bool status){
        segment.active = status;
        segment.set_nodes(segment.p0,segment.p1,segment.p2,segment.p3,segment.index,segment.parent,segment.active);
    }
    //change the status of nodes!
    void set_nodes(node &x1, node &x2, node &x3, node &x4, int index, int parent, bool status){
        x1.index = index;
        x1.parent = parent;
        x1.active = status;
        x2.index = index;
        x2.parent = parent;
        x2.active = status;
        x3.index = index;
        x3.parent = parent;
        x3.active = status;
        x4.index = index;
        x4.parent = parent;  
        x4.active = status;   
    }

    

};