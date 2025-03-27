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
#include <euc_distance.h>

/**
 * @brief Get the inscribed nodes within a given radius
 * 
 * @param tree 
 * @param selected_seg 
 * @param optimization_radius 
 * @return a vector of nodes inscribed within a circular radius
 */
std::vector <node> get_inscribed_nodes(std::vector<bezier_seg> &tree, bezier_seg &selected_seg, float optimization_radius){

    std::vector<node> inscribed_nodes;
    inscribed_nodes.resize(0);
    for(int i = 0; i <tree.size();i++){
        //std::cout<<"tree[i].p3.index value is"<<std::endl;
        //std::cout<<tree[i].p3.index<<std::endl;

        if (euc_dist(selected_seg.p3,tree[i].p3) <= optimization_radius){
            //std::cout<<"index of inscribed node"<<std::endl;
            //std::cout<<tree[i].p3.index<<std::endl;
            //std::cout<<tree[i].p3.parent<<std::endl;
            //std::cout<<tree[i].p3.point.x<<","<<tree[i].p3.point.y<<std::endl;
            inscribed_nodes.push_back(tree[i].p3);

            //std::cout<<&inscribed_nodes[i].index<<std::endl;
        }
    }
    return inscribed_nodes;
}



