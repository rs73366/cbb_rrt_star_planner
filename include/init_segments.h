#pragma once

#include <vector>
#include <cmath>
 //#include "all_functions.h"

#define TWO_M_PI 2 * M_PI
#define M_PI_10 M_PI / 10.

#include "nodes_structure.h"


bezier_seg  get_init_segment(node &initial, double yaw_angle, double step_size){
    //std::cout<<"init segments file running"<<std::endl;
    bezier_seg init_segment;
    //if(0<=yaw_angle < (M_PI/2)){
        init_segment.p0 = initial;
        init_segment.p1.point.x = initial.point.x + double((step_size*cos(yaw_angle))/3);
        init_segment.p1.point.y = initial.point.y + double((step_size*sin(yaw_angle))/3);
        init_segment.p2.point.x = initial.point.x + 2*double((step_size*cos(yaw_angle))/3);
        init_segment.p2.point.y = initial.point.y + 2*double((step_size*sin(yaw_angle))/3);
        init_segment.p3.point.x = initial.point.x + step_size*cos(yaw_angle);
        init_segment.p3.point.y = initial.point.y + step_size*sin(yaw_angle);

    /*}
    else if((M_PI/2)<=yaw_angle < M_PI){
        init_segment.p0 = initial;
        init_segment.p1.point.x = initial.point.x - float(1/3)*step_size*cos(yaw_angle);
        init_segment.p1.point.y = initial.point.y + float(1/3)*step_size*sin(yaw_angle);
        init_segment.p2.point.x = initial.point.x - float(2/3)*step_size*cos(yaw_angle);
        init_segment.p2.point.y = initial.point.y + float(2/3)*step_size*sin(yaw_angle);
        init_segment.p3.point.x = initial.point.x - step_size*cos(yaw_angle);
        init_segment.p3.point.y = initial.point.y + step_size*sin(yaw_angle);
    }
    else if((M_PI)<= yaw_angle <((3*M_PI)/2)){
        init_segment.p0 = initial;
        init_segment.p1.point.x = initial.point.x - float(1/3)*step_size*cos(yaw_angle);
        init_segment.p1.point.y = initial.point.y - float(1/3)*step_size*sin(yaw_angle);
        init_segment.p2.point.x = initial.point.x - float(2/3)*step_size*cos(yaw_angle);
        init_segment.p2.point.y = initial.point.y - float(2/3)*step_size*sin(yaw_angle);
        init_segment.p3.point.x = initial.point.x - step_size*cos(yaw_angle);
        init_segment.p3.point.y = initial.point.y - step_size*sin(yaw_angle);

    }
    else if(M_PI <= yaw_angle < 2*M_PI){
        init_segment.p0 = initial;
        init_segment.p1.point.x = initial.point.x + float(1/3)*step_size*cos(yaw_angle);
        init_segment.p1.point.y = initial.point.y - float(1/3)*step_size*sin(yaw_angle);
        init_segment.p2.point.x = initial.point.x + float(2/3)*step_size*cos(yaw_angle);
        init_segment.p2.point.y = initial.point.y - float(2/3)*step_size*sin(yaw_angle);
        init_segment.p3.point.x = initial.point.x + step_size*cos(yaw_angle);
        init_segment.p3.point.y = initial.point.y - step_size*sin(yaw_angle);
        
    }*/
    return init_segment;
}