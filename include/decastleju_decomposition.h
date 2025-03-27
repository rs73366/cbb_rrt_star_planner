

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

#include <nodes_structure.h>

// write a recursive decomposition to divide the segment
bezier_seg decompose_bezier_seg(bezier_seg &seg, float t){
    //std::cout<<"t value is: "<<","<<t<<std::endl;
    //std::cout<<"decomposition function step1"<<std::endl;
    bezier_seg new_seg;
    //std::cout<<"decomposition function step2"<<std::endl;
    new_seg.p0 = seg.p0;
    //std::cout<<"decomposition function step3"<<std::endl;
    new_seg.p1.point.x = (1-t)*(seg.p0.point.x) + t*(seg.p1.point.x);
    new_seg.p1.point.y = (1-t)*(seg.p0.point.y) + t*(seg.p1.point.y);
    //std::cout<<"decomposition function step4"<<std::endl;
    new_seg.p2.point.x = pow((1-t),2)*seg.p0.point.x+2*t*(1-t)*seg.p1.point.x+pow(t,2)*seg.p2.point.x;
    new_seg.p2.point.y = pow((1-t),2)*seg.p0.point.y+2*t*(1-t)*seg.p1.point.y+pow(t,2)*seg.p2.point.y;
    //std::cout<<"decomposition function step5"<<std::endl;
    new_seg.p3.point.x = pow((1-t),3)*seg.p0.point.x+3*t*pow((1-t),2)*seg.p1.point.x+3*pow(t,2)*(1-t)*seg.p2.point.x+pow(t,3)*seg.p3.point.x;
    new_seg.p3.point.y = pow((1-t),3)*seg.p0.point.y+3*t*pow((1-t),2)*seg.p1.point.y+3*pow(t,2)*(1-t)*seg.p2.point.y+pow(t,3)*seg.p3.point.y;
    //std::cout<<"decomposition function step6"<<std::endl;
    new_seg.active = seg.active;
    //std::cout<<"decomposition function step7"<<std::endl;
    new_seg.index = seg.index;
    //std::cout<<"decomposition function step8"<<std::endl;
    new_seg.parent = seg.parent;
    //std::cout<<"decomposition function step9"<<std::endl;
    return new_seg;
}
