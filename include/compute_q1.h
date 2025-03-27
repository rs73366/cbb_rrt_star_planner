
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
#include "compute_h0.h"
#include "curvature_values.h"

node get_q1(bezier_seg &parent_segment, node &q0, node &q2){
    //std::cout<<"compute q1 file running"<<std::endl;

    //std::cout<<"the value of tc is: "<<parent_segment.t_c<<std::endl;
    double k_t = parent_segment.curvature_vals[(parent_segment.t_c*parent_segment.parts)]; // compute the curvature at joint point
    //std::cout<<"curvature k_t at joint is: "<<k_t<<std::endl;
    double angle = get_slope(parent_segment, parent_segment.t_c); // compute the slope at joint point!
    double m_0;
    if(abs(abs(angle) - 1.5707) <=0.07){
        if(angle < 0){
            m_0 = -20.0;
        }
        else {
            m_0 = 20.0;
        }
    }
    else{
        m_0 = tan(angle);
    }
    //std::cout<<"slope m_0 at joint is: "<<m_0<<std::endl;
    double h_0 = get_h0(parent_segment, q0,q2);
    //std::cout<<"h_0 is: "<<h_0<<std::endl;
    node q1; // q1 node
    q1.parent = parent_segment.index;
    q1.index = q0.index;
    
    geometry_msgs::Point p0,p2;
    p0 = q0.point;
    p2 = q2.point;
    if (get_distance(p0,p2)<=0.05){
        //std::cout<<"case_1"<<std::endl;
        q1.point.x = ((p0.x + p2.x)/2.0);
        q1.point.y = ((p0.y + p2.y)/2.0);
    }
    else if(k_t == 0){
        //std::cout<<"case 2"<<std::endl;
        q1.point.x = ((p0.x + p2.x)/2.0);
        q1.point.y = ((p0.y + p2.y)/2.0);
        //q1 = q2;
    }
    else if (cos(angle) >= 0){
            q1.point.x = q0.point.x + pow(((2.0/3.0))*abs((h_0/k_t)*(1/(1+pow(m_0,2)))),0.5);
            q1.point.y = m_0*(q1.point.x - q0.point.x) + q0.point.y;
        }
    else if (cos(angle) <0){
            q1.point.x = q0.point.x - pow(((2.0/3.0))*abs((h_0/k_t)*(1/(1+pow(m_0,2)))),0.5);
            q1.point.y = m_0*(q1.point.x - q0.point.x) + q0.point.y;
        }
    else{
            std::cout<<"this should never occur ideally"<<std::endl;
            q0 = q1;
        }
    return q1;

    /*
    else if(m_0 < 0){
        //std::cout<<"case_3"<<std::endl;
        if(q2.point.y <= q0.point.y && q2.point.x >= q0.point.x){
            q1.point.x = q0.point.x + pow(((2.0/3.0))*abs((h_0/k_t)*(1/(1+pow(m_0,2)))),0.5);
            q1.point.y = m_0*(q1.point.x - q0.point.x) + q0.point.y; // m_0 is already negative so q1y would be lesser!
        }

        else if(q2.point.y >= q0.point.y && q2.point.x <= q0.point.x){
            q1.point.x = q0.point.x - pow(((2.0/3.0))*((h_0/k_t)*(1/(1+pow(m_0,2)))),0.5);
            q1.point.y = m_0*(q1.point.x - q0.point.x) + q0.point.y;
        }
        else if(q2.point.y >= q0.point.y && q2.point.x >= q0.point.x){
            q1.point.x = q0.point.x + pow(((2.0/3.0))*abs((h_0/k_t)*(1/(1+pow(m_0,2)))),0.5);
            q1.point.y = m_0*(q1.point.x - q0.point.x) + q0.point.y;
        }
        else if(q2.point.y <= q0.point.y && q2.point.x <= q0.point.x){
            q1.point.x = q0.point.x - pow(((2.0/3.0))*abs((h_0/k_t)*(1/(1+pow(m_0,2)))),0.5);
            q1.point.y = m_0*(q1.point.x - q0.point.x) + q0.point.y;
        }


    }
    else if(m_0 >= 0){
        //std::cout<<"case_4"<<std::endl;
        if(q2.point.y <= q0.point.y && q2.point.x >= q0.point.x){
            q1.point.x = q0.point.x + pow(((2.0/3.0))*abs((h_0/k_t)*(1/(1+pow(m_0,2)))),0.5);
            q1.point.y = m_0*(q1.point.x - q0.point.x) + q0.point.y;
        }

        else if(q2.point.y >= q0.point.y && q2.point.x <= q0.point.x){
            q1.point.x = q0.point.x - pow(((2.0/3.0))*abs((h_0/k_t)*(1/(1+pow(m_0,2)))),0.5);
            q1.point.y = m_0*(q1.point.x - q0.point.x) + q0.point.y;
        }
        else if(q2.point.y >= q0.point.y && q2.point.x >= q0.point.x){
            q1.point.x = q0.point.x + pow(((2.0/3.0))*abs((h_0/k_t)*(1/(1+pow(m_0,2)))),0.5);
            q1.point.y = m_0*(q1.point.x - q0.point.x) + q0.point.y;
        }
        else if(q2.point.y <= q0.point.y && q2.point.x <= q0.point.x){
            q1.point.x = q0.point.x - pow(((2.0/3.0))*abs((h_0/k_t)*(1/(1+pow(m_0,2)))),0.5);
            q1.point.y = m_0*(q1.point.x - q0.point.x) + q0.point.y;
        }
        else{
            q1 = q0;
            //std::cout<< "failed to compute q1"<<std::endl;
        }


    }
    */
    
   
}
