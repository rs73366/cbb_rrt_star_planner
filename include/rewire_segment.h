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

#include <euc_distance.h>

//assumes that selected segment is optimal as per given optimization radius!
void rewire_seg(bezier_seg &selected_segment, std::vector<node> &incircle_nodes){

    //float min_cost = selected_segment.p3.cost;
    int i = 0;
    std::cout<<"the size of incircle nodes is: "<<", "<<incircle_nodes.size()<<std::endl;
    while (i < incircle_nodes.size()){
        std::cout<<i<<std::endl;
        if (selected_segment.parent == incircle_nodes[i].index){
            std::cout<<"parent cannot be rewired"<<std::endl;
            i = i + 1;
            std::cout<<"hmm1"<<std::endl;    
            continue;        
        }
        std::cout<<"hmm2"<<std::endl;  
        float min_cost = incircle_nodes[i].cost;
        std::cout<<"hmm3"<<std::endl;  
        if (selected_segment.p3.cost + euc_dist(selected_segment.p3,incircle_nodes[i]) < min_cost ){
            incircle_nodes[i].parent = selected_segment.index; 
            std::cout<<"hmm4"<<std::endl;  
        }
        std::cout<<"hmm5"<<std::endl;  
        i = i + 1;
    }
    return;
    
    //return incircle_nodes;
}
