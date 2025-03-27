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

std::vector <int> get_children_index(std::vector <bezier_seg> &tree, bezier_seg &segment){
    std::vector<int> children_nodes;

    int j = 0; // keep track on which children the loop is on!
    int i = segment.children.size();
    while (j < i){
        segment = tree[segment.children[j]];
        children_nodes.insert(children_nodes.end(),get_children_index(tree,segment).begin(),get_children_index(tree,segment).end());
        j = j +1;
    }
    if (j == i){
        children_nodes.push_back(segment.index);
        return children_nodes;
    }
    return children_nodes;
}
