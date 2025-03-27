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


/**
 * @brief Get the area between tree points
 * 
 * @param point1 
 * @param point2 
 * @param point3 
 * @return ** double 
 */

double get_area(geometry_msgs::Point &point1, geometry_msgs::Point &point2, geometry_msgs::Point &point3){
    //std::cout<< "the area between points is: "<<std::endl;
    //std::cout<<abs(point1.x*(point2.y-point3.y)+point2.x*(point3.y-point1.y)+point3.x*(point1.y-point2.y))<<std::endl;
    //std::cout<<"curvature values file running"<<std::endl;
    return abs(point1.x*(point2.y-point3.y)+point2.x*(point3.y-point1.y)+point3.x*(point1.y-point2.y));
}

double get_distance(geometry_msgs::Point &point1, geometry_msgs::Point &point2){
    //std::cout<<"curvature values file running"<<std::endl;
    return sqrt(pow((point2.y-point1.y),2)+pow((point2.x-point1.x),2));
}


std::vector <double> get_curvature (std::vector<geometry_msgs::Point> &bezier_curve_points){
    //std::cout<<"curvature values file running"<<std::endl;

    std::vector<double> curvature_vector;
    for(int i = 0; i<bezier_curve_points.size()-2;++i){
        geometry_msgs::Point p1,p2,p3;
        p1 = bezier_curve_points[i];
        p2 = bezier_curve_points[i+1];
        p3 = bezier_curve_points[i+2];
        double d1,d2,d3;
        double area = get_area(p1,p2,p3);
        d1 = get_distance(p1,p2);
        d2 = get_distance(p2,p3);
        d3 = get_distance(p3,p1);
        //std::cout<<"THe value of d1,d2,d3 and area is: "<<d1<<","<<d2<<","<<d3<<","<<area<<std::endl;
        double curvature = (4.0*area)/(d1*d2*d3);
        if(curvature < 0.001){
            curvature = 0.0;
            //std::cout<<"if is true"<<std::endl;
        }
        curvature_vector.push_back(curvature);
        //std::cout<<" the curvature value is: "<<double((4.0*area)/(d1*d2*d3))<<std::endl;

    }


    return curvature_vector;
}