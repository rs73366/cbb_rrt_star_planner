/** include the libraries you need in your planner here */
 /** for global path planner interface */

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
#include "init_segments.h"
#include "curvature_values.h"
#include "joint_node.h"     
#include "compute_q1.h"  
#include "compute_slopes.h"
#include <decastleju_decomposition.h>
#include <euc_distance.h>
#include <rewire_segment.h>
#include <inscribed_nodes.h>
#include <get_children.h>
 //#include "all_functions.h"

#define TWO_M_PI 2 * M_PI
#define M_PI_10 M_PI / 10.


 using std::string;

 #ifndef cbb_rrt_star_planner_cpp
 #define cbb_rrt_star_planner_cpp

 namespace cbb_rrt_star_global_planner {

 class cbb_rrt_star_planner : public nav_core::BaseGlobalPlanner {
 public:

  cbb_rrt_star_planner();
  cbb_rrt_star_planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /** overridden classes from interface nav_core::BaseGlobalPlanner **/
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);
    void publish_plan(std::vector<geometry_msgs::PoseStamped>& plan);
    ~cbb_rrt_star_planner(){

    }

    private:
    int K; // maximum number of iterations;
    std::string name_;
    costmap_2d::Costmap2D* costmap_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    std::vector<geometry_msgs::Point> footprint;
    double goal_tolerance, step_size, robot_radius, optimization_radius; // standard rrt planner parameters
    bool tree_viz, tree_initalized;
    ros::Publisher plan_pub_, tree_pub_;
    double critical_curvature;
   

  };
 }; // namespace ends here
 #endif



/**
 * @brief rrt data structure
 * 
 */

class cbb_rrt_optimal_tree{
 
 public:
    node x_init;
    bezier_seg initial_segment;
    std::vector <bezier_seg> tree; // a vector of bezier segments actuallaY! 
    std::vector <node> first_node_tree;
    std::vector <node> last_node_tree;
    std::vector <std::pair<node, node>> edges; // edges are a pair of nodes which represent an edge
    costmap_2d::Costmap2DROS* free_space; // free space for tree;
    
    bool success{ 0 };

    cbb_rrt_optimal_tree(node x_initial, costmap_2d::Costmap2DROS* costmap_){

        // initialize the members
        this->x_init = x_initial;
        this->free_space = costmap_;

        // add the starting node to tree
        //add_node(x_initial);
        
    }


    // add a node to tree
    //void add_node(node x){
       //this->tree.push_back(x);
    //}

    void add_segment(bezier_seg seg){
        this->tree.push_back(seg);
    }

    // create an edge from two nodes
    void add_edge(node x1, node x2){
        this->edges.push_back(std::make_pair(x1,x2));
    }

    // edge would be defined in a map fashion for easy insert and delete !
    void delete_edge(node x1, node x2){
        //this->edges.erase(std::make_pair(x1,x2));
    }

    ~cbb_rrt_optimal_tree(){};
};






/**
 * @brief Get the curvature of a bezier segment
 * 
 * @param segment is the bezier segment!
 * @p p is the number of points to divide the segment in
 * @return **  curvature vector for some points 
 */




/**
 * @brief check wheter the node is in free space or not
 * 
 * @param x  node
 * @param robot_radius radiuss of robot
 * @param costmap_ros costmap 
 * @return true  if  node is in free space
 * @return false  if node is not in free space
 */

// check if node lies in free space
bool node_in_free_space(node x, costmap_2d::Costmap2DROS* costmap_ros, double robot_radius){
    //std::cout<<"cbb_rrt_Star_planner file running"<<std::endl;

    //ROS_INFO("Node in free space function initiated");
    bool result{ 1 };
    double theta = 0.0;
    costmap_2d::Costmap2D* costmap_; // access costmap
     // locations on map, not world
    std::vector<costmap_2d::MapLocation> map_polygon, polygon_cells; // vectors to get polgon to work with costmap
    map_polygon.clear();
    polygon_cells.clear();

    costmap_ = costmap_ros->getCostmap();
    double robot_radius_step(0.05);  // Assume 5cm step.
    unsigned int mx,my;
    //std::cout<<x.point.x<<","<<x.point.y<<std::endl;
    //std::cout<<"The robot radius is"<<","<<robot_radius<<std::endl;
    while (theta <= TWO_M_PI){
        costmap_2d::MapLocation map_loc;
        //std::cout<<"theta value is: "<<","<<theta<<std::endl;
        // take all the geometric points and then convert them in cells using world to map function
        if (!costmap_->worldToMap(x.point.x + robot_radius*cos(theta), x.point.y + robot_radius*sin(theta), map_loc.x, map_loc.y )){
            // false if conversion is not successful, i.e. the point does not exist on map or lies outside its bounds
            return false;

        }
        // push these cells, in a vector which essentially represents a polygon
        map_polygon.push_back(map_loc);
        theta += M_PI_10;
        //ROS_INFO("map_polygon SIZE is");
        
        //std::cout<<"the map locs are: "<<map_loc.x<<","<<map_loc.y<<std::endl;
    }
    //std::cout<<"the size of map_polygon is"<<map_polygon.size()<<std::endl;
    // get all the cells within a polygon
    costmap_->convexFillCells(map_polygon, polygon_cells);

    // evaluate all the points in a  polygon for obstacles!
    //int num = costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    //std::cout<<num<<std::endl;
    //std::cout<<costmap_2d::INSCRIBED_INFLATED_OBSTACLE<<std::endl;
    //std::cout<<"the size of polygon cell is: "<<polygon_cells.size()<<std::endl;
    for (unsigned int i = 0; i < polygon_cells.size(); ++i){
        if (costmap_->getCost(polygon_cells[i].x, polygon_cells[i].y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
            result = 0;
            return result;
            
        }
    }
    //ROS_INFO("cost info for polygon is checked!");
    return result;

}


/**
 * @brief Get the euclidean distance between two nodes
 * 
 * @param x1 first node
 * @param x2 second node
 * @return distance between two nodes
 */
double get_euc_dist(node x1, node x2){
    //std::cout<<"cbb_rrt_Star_planner file running"<<std::endl;
    return sqrt(pow((x1.point.x-x2.point.x),2)+pow((x1.point.y-x2.point.y),2));
}

/**
 * @brief check if an edge consisting of two nodes will have no collison inclding robot radius
 * 
 * @param x1 first node
 * @param x2 second node
 * @param map_ map input
 * @param robot_radius robot radius
 * @return true  if edge is collision free
 * @return false if edge has some collision
 */
bool check_edge(node x1, node x2, costmap_2d::Costmap2DROS* map_, double robot_radius){
    //std::cout<<"cbb_rrt_Star_planner file running"<<std::endl;

    double d = get_euc_dist(x1,x2);

    int check_points = ceil(d/(robot_radius*0.2));
    node x_intermediate;
    double ang = atan2(x2.point.y-x1.point.y,x2.point.x-x1.point.x);
    for (int i = 1; i<check_points;++i){
        x_intermediate.point.x = x1.point.x + robot_radius*cos(ang);
        x_intermediate.point.y = x1.point.y + robot_radius*sin(ang);
        x_intermediate.point.z = 0;
        if (!node_in_free_space(x_intermediate, map_,robot_radius)){
            return false;
        }
    }
    return true; // return true if no collision



}

// get safe robot radius



double get_robot_radius(std::vector<geometry_msgs::Point> footprint)
{
    //std::cout<<"cbb_rrt_Star_planner file running"<<std::endl;
    //ROS_INFO("getRobotRadius function running");
  double max_dist{ 0. }, dist{};
  geometry_msgs::Point origin{};
  node org;
  org.point.x = 0.0;
  org.point.y = 0.0;
  org.point.z = 0.0;

  for (int i = 0; i < footprint.size(); ++i)
  {
    //std::cout<<"auto_pt is: "<<pt.x<<pt.y<<std::endl;
    auto pt = footprint[i];
    //std::cout<<"auto_pt is: "<<pt.x<<pt.y<<std::endl;
    node pt_; 
    pt_.point.x = pt.x;
    pt_.point.y = pt.y;
    pt_.point.z = 0;
    dist = get_euc_dist(org, pt_);
    if (dist > max_dist)
    {
      max_dist = dist;
      //std::cout<<"Max dist is: "<<","<<max_dist<<std::endl;
    }
  }
  return max_dist;
}

// generate random points
double randomDouble(double fMin, double fMax)
{
    //std::cout<<fMin<<","<<fMax<<std::endl;
  //std::cout<<"cbb_rrt_Star_planner file running"<<std::endl;
  fMin = -50;
  fMax = 50;
  double f = (double)rand() / RAND_MAX;
  //std::cout<<f<<","<<fMin + f * (fMax - fMin)<<std::endl;
  return fMin + f * (fMax - fMin);
}

/**
 * @brief generates a random node for algorithm
 * 
 * @param costmap_ros   map to get some limits
 * @param robot_radius  to check if space around the selected point is free or not
 * @return ** node  returns a feasable free node
 */
node gen_random_node(costmap_2d::Costmap2DROS* costmap_ros, double robot_radius){
    //std::cout<<"cbb_rrt_Star_planner file running"<<std::endl;
    //ROS_INFO("generate random node function initiated");
    node x_rand;
    costmap_2d::Costmap2D* costmap_;
    costmap_ = costmap_ros->getCostmap();
    bool node_free{0};
    //std::cout<<costmap_->getSizeInMetersX()<<","<<costmap_->getSizeInMetersY()<<std::endl;
    
    while (!node_free){ // run the loop till a free node is found!
        x_rand.point.x = randomDouble(costmap_->getOriginX(), costmap_->getOriginX() + costmap_->getSizeInMetersX());
        x_rand.point.y = randomDouble(costmap_->getOriginY(), costmap_->getOriginY() + costmap_->getSizeInMetersY());
        x_rand.point.z = 0;
        //ROS_INFO("dum dum: ");
        //std::cout<<x_rand.point.x<<","<<x_rand.point.y<<std::endl;
        node_free = node_in_free_space(x_rand, costmap_ros, robot_radius);
        //std::cout<<"node free"<<node_free<<std::endl;
        //std::cout<<node_in_free_space(x_rand, costmap_ros, robot_radius)<<std::endl;
       
        //if(!node_in_free_space(x_rand, costmap_ros, robot_radius)){ // if true exit the function
           // ROS_INFO("The node lies in free region");
            //return x_rand;
        //}
    }
    return x_rand; // this is not required though
}

// extend the rrt tree to get new node
node extend_tree(const node &x_near, node &x_rand, double step_size){
    //std::cout<<"cbb_rrt_Star_planner file running"<<std::endl;
    //std::cout<<"Extend Tree Running!"<<std::endl;
    double d = get_euc_dist(x_near, x_rand);
    //std::cout<<"The distance betwween random and near node is!"<<std::endl;
    //std::cout<<d<<std::endl;
    double ang = atan2((x_rand.point.y-x_near.point.y),(x_rand.point.x-x_near.point.x));
    node x_new;
    if (d >= step_size){
        x_new.point.x = x_near.point.x + step_size*(cos(ang));
        x_new.point.y = x_near.point.y + step_size*(sin(ang));
        x_new.point.z = 0;
    }
    else{
        x_new.point.x = x_near.point.x + d*(cos(ang));
        x_new.point.y = x_near.point.y + d*(sin(ang));
        x_new.point.z = 0;
    }
    return x_new;
}

/**
 * @brief Get the pointer to nearest node object
 * 
 * @param random_tree of which the nearest node is a part 
 * @param x_rand which is the random node
 * @return ** node* 
 */

node get_nearest_node(const cbb_rrt_optimal_tree &tree, node &x_rand){
    //std::cout<<"cbb_rrt_Star_planner file running"<<std::endl;
    int index = 0;
    // initialize min value by a huge value
    double min_dist = 5000.00;
    for (int i = 0; i<tree.last_node_tree.size();++i){
        //ROS_INFO("THE distance calculated is: ");
        double d = get_euc_dist(x_rand, tree.last_node_tree[i]);
        //std::cout<<"THe distance calculated is: "<<std::endl;
        //std::cout<<d<<std::endl;
        
        if (d < min_dist){
            index = i;
            min_dist = d;
        }
    }
    //ROS_INFO("THE INDEX OF THE CLOSEST NODE IS: ");
    //std::cout<<index<<std::endl;
    //ROS_INFO("The minimum distance is: ");
    //std::cout<<min_dist<<std::endl;
    return tree.last_node_tree[index];
}

/**
 * @brief check if there is any obstacle in the edge formed 
 * 
 * @param x1  the first node, probably belonging to tree
 * @param x2  a new node
 * @return true  if there is no collision
 * @return false if there is a collision
 */



std::vector <node> inscribed_nodes(const cbb_rrt_optimal_tree &tree, node &selected, double circle_radius){

    std::vector <node> returned_nodes;
    double d;

    for (int i = 0; i<tree.last_node_tree.size(); ++i){
        if(tree.last_node_tree[i].active){
            d = get_euc_dist(selected, tree.last_node_tree[i]);
            if(tree.last_node_tree[i].point.x == selected.point.x && tree.last_node_tree[i].point.y == selected.point.y){
            continue;
            }
            else if (d <= circle_radius){
            returned_nodes.push_back(tree.last_node_tree[i]);
            }
        }
        else{
            continue;
        }
    }
    return returned_nodes;
}


/**
 * @brief rewires a node to a lower cost parent!
 * 
 * @param tree  the tree data structure
 * @param selected node for rewiring 
 * @param circle_radius optimization radius
 * @param map_ costmap
 * @param robot_radius radius of robot!
 * @return ** node which is rewired with updated parents
 */

bezier_seg rewired_seg(cbb_rrt_optimal_tree &tree, node &selected, double circle_radius, double critical_curvature, costmap_2d::Costmap2DROS* map_, double robot_radius){
    bezier_seg rewired_seg; // a newly rewired segment!
    rewired_seg = tree.tree[selected.index];
    std::vector <node> inn_nodes = inscribed_nodes(tree, selected,circle_radius); // all inn nodes are active!
    double new_cost;
    // get cost of selected from 
    double selec_node_cost = selected.cost;
    selec_node_cost = tree.first_node_tree[selected.parent].cost + get_euc_dist(tree.first_node_tree[selected.parent],selected); 
    for(int i = 0; i<inn_nodes.size(); ++i){
        double d = get_euc_dist(selected,tree.first_node_tree[inn_nodes[i].index]);
        new_cost = tree.first_node_tree[inn_nodes[i].index].cost + d;
        if(new_cost < selec_node_cost){
            //if(check_edge(selected, inn_nodes[i], map_, robot_radius)){ // if the edge is not in collision then only change the parent!
                //selected.parent = inn_nodes[i].index;
                //selected.cost = new_cost;
            //}
            
            rewired_seg.p0 = get_joint_node(tree.tree[inn_nodes[i].index],tree.tree[inn_nodes[i].index].t_c);
            rewired_seg.p2 = tree.last_node_tree[inn_nodes[i].index];
            rewired_seg.p3 = selected;
            rewired_seg.p1 = get_q1(tree.tree[inn_nodes[i].index],rewired_seg.p0,rewired_seg.p2);
            rewired_seg.bezier_seg_points = get_bez_points(rewired_seg,rewired_seg.parts);
            rewired_seg.curvature_vals = get_curvature(rewired_seg.bezier_seg_points);
            rewired_seg.max_curvature = *max_element(std::begin(rewired_seg.curvature_vals),std::end(rewired_seg.curvature_vals));
            if(rewired_seg.max_curvature > critical_curvature || check_edge(rewired_seg.p0, rewired_seg.p3, map_, robot_radius) ){
                //std::cout<<"the rewired segment attempet invalidated/max curvature exceeding critical curvature"<<std::endl;
                continue;
            }
            else{
                selec_node_cost = new_cost;
                rewired_seg.index = selected.index;
                rewired_seg.parent = inn_nodes[i].index;
                rewired_seg.p0.cost = tree.first_node_tree[inn_nodes[i].index].cost + get_euc_dist(tree.first_node_tree[inn_nodes[i].index],rewired_seg.p0);
                rewired_seg.p3.cost = rewired_seg.p0.cost + get_euc_dist(rewired_seg.p0,rewired_seg.p3);
                rewired_seg.set_nodes(rewired_seg.p0,rewired_seg.p1,rewired_seg.p2,rewired_seg.p3,rewired_seg.index,rewired_seg.parent,true);
            }
        }
    }
    return rewired_seg; 
}


node near_node(std::vector<node> &lost_nodes,node x1){
    node closest_node;
    int index = -1;
    float min_dist = 5000;
    for (int i = 0;i <=lost_nodes.size()-1;){
        if (get_euc_dist(x1,lost_nodes[i]) < min_dist)
        {
            index = i;
            closest_node = lost_nodes[i];
        }
    }
    return closest_node;
}

/*
std::vector<node> greed_recovery(std::vector<node> &lost_nodes, node &goal_node){
    std::vector<node> greedy_nodes;
    node close_node = near_node(lost_nodes,goal_node);
    auto index = find(lost_nodes.begin(),lost_nodes.end(),close_node);
    while (index != lost_nodes.end()){
        greedy_nodes.push_back(close_node);
        index
        close_node = lost_nodes[]
    }


}

*/


//void reconnect_node(std::vector<node> &inside_nodes, )

/**
 * @brief Get the random tree object
 * 
 * @param start 
 * @param goal 
 * @param K 
 * @param optimization_radius 
 * @param step_size 
 * @param robot_radius 
 * @param goal_tolerance 
 * @param costmap_ros 
 * @return ** rrt_optimal_tree 
 */

cbb_rrt_optimal_tree get_random_tree(node start, node goal, int K, double optimization_radius, double step_size, double robot_radius, double critical_curvature, double goal_tolerance, costmap_2d::Costmap2DROS* costmap_ros){
    
    
    std::cout<<start.point.x<<","<<start.point.y<<std::endl;
    start.parent = -1; // start node does not have any parent
    start.index = 0; // index of the start node!
    start.cost = 0;
    double init_dist = step_size/(double)2.0; // some distance for initialization
    tf::Quaternion q(start.orientation.orientation.x, start.orientation.orientation.y, start.orientation.orientation.z, start.orientation.orientation.w);
    tf::Matrix3x3 heading(q);

    double roll,pitch,yaw;
    heading.getRPY(roll, pitch, yaw);
    cbb_rrt_optimal_tree random_tree(start, costmap_ros);
    
    bezier_seg init_Seg;
    init_Seg.index = 0;
    init_Seg.parent = -1;
    init_Seg.set_nodes(init_Seg.p0,init_Seg.p1,init_Seg.p2,init_Seg.p3,init_Seg.index,init_Seg.parent,true);
    init_Seg = get_init_segment(start, yaw, init_dist);
    init_Seg.index = 0;
    init_Seg.parent = -1;
    init_Seg.set_nodes(init_Seg.p0,init_Seg.p1,init_Seg.p2,init_Seg.p3,init_Seg.index,init_Seg.parent,true);
    
    init_Seg.bezier_seg_points = get_bez_points(init_Seg,init_Seg.parts);
    init_Seg.curvature_vals = get_curvature(init_Seg.bezier_seg_points);
    init_Seg.max_curvature = 0;

    std::cout<<"compute the slopes of init segment"<<std::endl;
    compute_slopes(init_Seg);

    random_tree.tree.push_back(init_Seg); // push the initial segment in tree
    random_tree.first_node_tree.push_back(init_Seg.p0); // push the initial node in  the tree!
    random_tree.last_node_tree.push_back(init_Seg.p3); // push the final node in tree!

    //std::cout<<"check_point_11"<<std::endl;
    node x_rand, x_near, x_new;

    int i = 1;
    int swc = 1;  // whether the goal should be sampled or not, defaults to yes;
    // run for K number of operations
    std::cout<<"CBB_rrt_Star_planner"<<std::endl;
    while (i < K){
        
        bezier_seg new_seg;
        //std::cout<<"The value of i and K is: "<<std::endl;
        //std::cout<<i<<","<< K <<std::endl;

        if(i%10 == 0 && swc == 1){ // give a 10 percent goal biasing
            //std::cout<<"The value of i and K is: "<<std::endl;
            //std::cout<<i<<","<< K <<std::endl;
            //ROS_INFO("GOAL SAMPLED");
            x_rand = goal;
            swc = 0;
            
        }
        else{   
            x_rand = gen_random_node(costmap_ros, robot_radius); // generate a random node in free space
            swc = 1; // reset the switch to shift to goal next time
        }


        //std::cout<<"check_point_13"<<std::endl;
        x_near = get_nearest_node(random_tree, x_rand); // get the nearest node in tree along with its index!
        if(!x_near.active){
            if(i%20 == 0 && swc == 1){
                swc = 0;
            }
            //std::cout<<"the near node was not active!"<<std::endl;
            continue;
        }

        new_seg.index = i; //assing number to nerwly sampled node
        new_seg.parent = x_near.index;
        x_new = extend_tree(x_near, x_rand, step_size); // extend from nearest node to obtain a new node
        if(x_new.point.x == x_near.point.x && x_new.point.y == x_near.point.y){
            std::cout<<"Same point is reproduced!"<<std::endl;
            //i = i + 1;
            if(i%20 == 0 && swc == 1){
                swc = 0;
            }
            continue;
        }
        
        new_seg.p0 = get_joint_node(random_tree.tree[x_near.index],random_tree.tree[x_near.index].t_c);
        new_seg.p0.cost = random_tree.tree[new_seg.parent].p0.cost + get_euc_dist(random_tree.tree[new_seg.parent].p0,new_seg.p0);
        new_seg.p2 = x_near;
        x_new.cost = random_tree.tree[new_seg.parent].p3.cost+ get_euc_dist(new_seg.p3, x_new);
        new_seg.p3 = x_new;
        new_seg.p1 = get_q1(random_tree.tree[x_near.index],new_seg.p0,new_seg.p2);
        if (!check_edge(new_seg.p0, new_seg.p3, costmap_ros, robot_radius)){ // write down a better collision function
            
            //std::cout<<"EDGE IS NOT GOOD"<<std::endl;
            if(i%20 == 0 && swc == 1){
                swc = 0;
            }
            continue;
        }
        new_seg.bezier_seg_points = get_bez_points(new_seg,new_seg.parts);
        new_seg.curvature_vals = get_curvature(new_seg.bezier_seg_points);
        int k_index = new_seg.curvature_vals.size();
        float t_cut = 1.0;
        for (int e = 0;e<k_index;++e){
            //std::cout <<"the curvature values are"<<","<<new_seg.curvature_vals[e]<<std::endl;
            if (new_seg.curvature_vals[e] > critical_curvature){
          //      std::cout<<"maxium curvature limit is exceeded!"<<std::endl;
            //    std::cout<<e<<std::endl;
              //  std::cout<<new_seg.curvature_vals[e]<<std::endl;
                t_cut = (float)e/(float)(new_seg.bezier_seg_points.size()-1);
                break;
            }
            
        }
        if (t_cut <0.5){
            //std::cout<<" segment discarded due to maxium curvature more than kc!"<<std::endl;
            if(i%20 == 0 && swc == 1){
                swc = 0;
            }
            continue;

        }
        //std::cout<<"running decomposition function"<<std::endl;
        new_seg = decompose_bezier_seg(new_seg, t_cut);
        
        new_seg.p0.cost = random_tree.tree[new_seg.parent].p0.cost + get_euc_dist(random_tree.tree[new_seg.parent].p0,new_seg.p0);
        new_seg.p3.cost = random_tree.tree[new_seg.parent].p3.cost + get_euc_dist(random_tree.tree[new_seg.parent].p3, new_seg.p3);
        /*
        new_seg.max_curvature = *max_element(std::begin(new_seg.curvature_vals),std::end(new_seg.curvature_vals));
        if(new_seg.max_curvature > critical_curvature){
            if(i%20 == 0 && swc == 1){
                swc = 0;
            }
            //std::cout<<"the value of i is: "<<i<<std::endl;
            std::cout<<"maxium curvature limit is exceeded!"<<std::endl;
            continue;
        }
        */
        new_seg.bezier_seg_points = get_bez_points(new_seg,new_seg.parts);
        new_seg.curvature_vals = get_curvature(new_seg.bezier_seg_points);
        compute_slopes(new_seg);
        new_seg.set_nodes(new_seg.p0,new_seg.p1,new_seg.p2,new_seg.p3, new_seg.index,new_seg.parent,true);
        
        // get the inscribed nodes
        std::vector<node> incircle_nodes = get_inscribed_nodes(random_tree.tree,new_seg,optimization_radius);
       
        // lets optimize the inscribed node
        float min_cost = new_seg.p3.cost; // assuming new_Seg is on minimum cost
        int original_parent = new_seg.parent; // store the original parent


        for (int w = 0; w < incircle_nodes.size();w++){

            if(incircle_nodes[w].index = new_seg.parent){// if parent is somehow selected, avoid it
                new_seg.parent = incircle_nodes[w].index;
                //w = w + 1;
                
            }

            if ((incircle_nodes[w].cost + euc_dist(new_seg.p3,incircle_nodes[w])) < min_cost) {

                new_seg.parent = incircle_nodes[w].index;
            }

        }
        new_seg = new_seg;
        new_seg.p0 = get_joint_node(random_tree.tree[new_seg.parent],random_tree.tree[new_seg.parent].t_c); 
        if (!check_edge(new_seg.p0, new_seg.p3, costmap_ros, robot_radius)){ // write down a better collision function
            
            //std::cout<<"EDGE IS NOT GOOD"<<std::endl;
            if(i%20 == 0 && swc == 1){
                swc = 0;
            }
            continue;
        }
        new_seg.p2 = random_tree.tree[new_seg.parent].p3;
        new_seg.p1 = get_q1(random_tree.tree[new_seg.parent],new_seg.p0,new_seg.p2);
        // p3 is already there
        new_seg.bezier_seg_points = get_bez_points(new_seg,new_seg.parts);
        new_seg.curvature_vals = get_curvature(new_seg.bezier_seg_points);
        


        k_index = new_seg.curvature_vals.size();
        t_cut = 1.0;
        for (int e = 0;e<k_index;++e){
            //std::cout <<"the curvature values are"<<","<<new_seg.curvature_vals[e]<<std::endl;
            if (new_seg.curvature_vals[e] > critical_curvature){
                //std::cout<<"maxium curvature limit is exceeded!"<<std::endl;
                std::cout<<e<<std::endl;
                t_cut = (float)e/(float)(new_seg.bezier_seg_points.size()-1);
                break;
            }
            
        }
        if (t_cut <0.5){
            //std::cout<<" segment discarded due to maxium curvature more than kc!"<<std::endl;
            if(i%20 == 0 && swc == 1){
                swc = 0;
            }
            continue;

        }
        //std::cout<<"running decomposition function"<<std::endl;
        
        //new_seg = decompose_bezier_seg(new_seg,t_cut);
        random_tree.add_segment(new_seg);
        random_tree.first_node_tree.push_back(new_seg.p0);
        random_tree.last_node_tree.push_back(new_seg.p3);
        random_tree.tree[new_seg.parent].children.push_back(new_seg.index);// add the new segment as children to its parent segment
        // finally newly added bezier segment is done. now we can rewire other existing nodes within circle
        /*
        //incircle_nodes = rewire_seg(new_seg,incircle_nodes); // get the updated parent of incirle nodes
        // update the parent of incirle nodes as per the optimised newly added segment! this change is not extending directly to tree!
        rewire_seg(new_seg,incircle_nodes);
        std::cout<<"incircle nodes rewired"<<std::endl;
        int j = 0;
        while (j < incircle_nodes.size()){
            if (incircle_nodes[j].index == new_seg.parent){
                j = j + 1;
                continue;
            }
            bezier_seg rewired_seg;
            rewired_seg.p0 = get_joint_node(random_tree.tree[incircle_nodes[j].parent],random_tree.tree[incircle_nodes[j].parent].t_c);
            rewired_seg.p2 = random_tree.tree[incircle_nodes[j].parent].p3;
            rewired_seg.p1 = get_q1(random_tree.tree[incircle_nodes[j].parent],rewired_seg.p0,rewired_seg.p2);
            rewired_seg.p3 = incircle_nodes[j];

            if (!check_edge(rewired_seg.p0, rewired_seg.p3, costmap_ros, robot_radius)){ // write down a better collision function
            
            //std::cout<<"EDGE IS NOT GOOD"<<std::endl;
                if(i%20 == 0 && swc == 1){
                    swc = 0;
                }
                j = j + 1;
                continue;

            
            }

            rewired_seg.bezier_seg_points = get_bez_points(new_seg,new_seg.parts);
            rewired_seg.curvature_vals = get_curvature(new_seg.bezier_seg_points);
            k_index = rewired_seg.curvature_vals.size();
            t_cut = 1.0;
            for (int e = 0;e<k_index;++e){
                //std::cout <<"the curvature values are"<<","<<new_seg.curvature_vals[e]<<std::endl;
                if (rewired_seg.curvature_vals[e] > critical_curvature){
                    //std::cout<<"maxium curvature limit is exceeded!"<<std::endl;
                    std::cout<<e<<std::endl;
                    t_cut = (float)e/(float)(rewired_seg.bezier_seg_points.size()-1);
                    break;
                }
                
            }
            if (t_cut <0.9){
                //std::cout<<" segment discarded due to maxium curvature more than kc!"<<std::endl;
                if(i%20 == 0 && swc == 1){
                    swc = 0;
                }
                // since there is an alternative available but rewiring cannot be done due to curvature violations.
                //rewired_seg.active = false;
                //random_tree.tree[rewired_seg.index].active = false; // turn the node off and turn all of its children off
                std::vector <int> children_nodes = get_children_index(random_tree.tree,random_tree.tree[incircle_nodes[j].index]);
                for (int c = 0;c < children_nodes.size();++c){
                    random_tree.tree[children_nodes[c]].active = false;
                    random_tree.first_node_tree[children_nodes[c]].active = false;
                    random_tree.last_node_tree[children_nodes[c]].active = false;
                    random_tree.tree[children_nodes[c]].set_nodes(random_tree.tree[children_nodes[c]].p0,random_tree.tree[children_nodes[c]].p1,random_tree.tree[children_nodes[c]].p2,random_tree.tree[children_nodes[c]].p3,-2,-3,false);
                }
                j + 1;
                continue;

            }
            std::vector <int> children_nodes = get_children_index(random_tree.tree,random_tree.tree[incircle_nodes[j].index]);
            children_nodes.pop_back(); // pop back the top one!
            for (int c = 0;c < children_nodes.size();++c){
                random_tree.tree[children_nodes[c]].active = false;
                random_tree.first_node_tree[children_nodes[c]].active = false;
                random_tree.last_node_tree[children_nodes[c]].active = false;
                random_tree.tree[children_nodes[c]].set_nodes(random_tree.tree[children_nodes[c]].p0,random_tree.tree[children_nodes[c]].p1,random_tree.tree[children_nodes[c]].p2,random_tree.tree[children_nodes[c]].p3,-2,-3,false);
            }
            random_tree.tree[incircle_nodes[j].index].parent = incircle_nodes[j].parent;
            random_tree.tree[incircle_nodes[j].index].set_nodes(random_tree.tree[incircle_nodes[j].index].p0,
            random_tree.tree[incircle_nodes[j].index].p1,random_tree.tree[incircle_nodes[j].index].p2,
            random_tree.tree[incircle_nodes[j].index].p3,incircle_nodes[j].index,incircle_nodes[j].parent,true);
            j = j + 1;


        }
*/
        /*
        // lets rewire this segment now!

        bezier_seg rewir_Seg = rewired_seg(random_tree,new_seg.p3,optimization_radius,critical_curvature,costmap_ros,robot_radius);


        random_tree.tree[new_seg.index] = rewir_Seg;
        random_tree.first_node_tree[new_seg.index] = rewir_Seg.p0;
        random_tree.last_node_tree[new_seg.index] = rewir_Seg.p3;
s
        std::vector<node> in_nodes = inscribed_nodes(random_tree,rewir_Seg.p3,optimization_radius); // inscribed nodes wont have the selected node!
        
        for(int l = 0; l <in_nodes.size();++l){
            double reconnect_cost;
            if(in_nodes[l].index == rewir_Seg.parent){
                continue;
            }
            double check_cost = random_tree.tree[in_nodes[l].parent].p0.cost + get_euc_dist(random_tree.tree[in_nodes[l].parent].p0,in_nodes[l]);
            reconnect_cost = random_tree.tree[rewir_Seg.index].p0.cost + get_euc_dist(rewir_Seg.p0,in_nodes[l]);
            if(reconnect_cost < check_cost){
                bezier_seg reconnected_seg;
                reconnected_seg.p0 = get_joint_node(rewir_Seg,rewir_Seg.t_c);
                reconnected_seg.p2 = rewir_Seg.p3;
                reconnected_seg.p3 = random_tree.last_node_tree[in_nodes[l].index];
                reconnected_seg.p2 = get_q1(rewir_Seg,reconnected_seg.p0,reconnected_seg.p2);
                reconnected_seg.bezier_seg_points = get_bez_points(reconnected_seg, reconnected_seg.t_c);
                reconnected_seg.curvature_vals = get_curvature(reconnected_seg.bezier_seg_points);
                reconnected_seg.max_curvature = *max_element(std::begin(reconnected_seg.curvature_vals),std::end(reconnected_seg.curvature_vals ));
                
                if(reconnected_seg.max_curvature > critical_curvature || !check_edge(reconnected_seg.p0, reconnected_seg.p3, costmap_ros, robot_radius) ){
                    if(i%20 == 0 && swc == 1){
                    swc = 0;
                    }
                    std::cout<<"reconnected segment failed"<<std::Endl;
                    continue;
                }
                
                

            }

        }
        */

        // update the properties of newly added node
        // select the node for rewiring
        /*
        node selected = random_tree.tree[i];
        random_tree.tree[i] = rewired_node(random_tree, selected, optimization_radius, costmap_ros,robot_radius);
        // once rewired the edge can be added to 

        
        random_tree.add_edge(random_tree.tree[random_tree.tree[i].parent], random_tree.tree[i]); // add the rewired edge to tree!
        std::vector<node> in_nodes = inscribed_nodes(random_tree,selected,optimization_radius); // inscribed nodes wont have the selected node!
        for(int l = 0; l<in_nodes.size(); ++l){
            //node rewired;
            double new_cost = 0;
            
            double c1 = get_euc_dist(in_nodes[l],selected); // cost for each inside node!
            new_cost = c1 + selected.cost;
            if(in_nodes[l].index == selected.parent){ // selected node is already rewired! so its parent is optimal!
                continue;
            }
            // replicate the rewired node
            //rewired = rewired_node(random_tree, in_nodes[l], optimization_radius, costmap_ros,robot_radius);
            // update the rewired node within the tree
            if(new_cost < in_nodes[l].cost){
                
                if(check_edge(selected, in_nodes[l], costmap_ros, robot_radius)){ // if the edge is not in collision then only change the parent!

                    //in_nodes[l].parent = selected.index;
                    random_tree.tree[in_nodes[l].index].parent = selected.index;
                }
                ROS_INFO("THE RECONNECTED NODE IS IN COLLISION");
                
            }
            
            //random_tree.tree[rewired.index] = rewired;
        }
        */
        //std::cout<<x_n<<","<<random_tree.tree[i].parent<<std::endl;
        
        if(get_euc_dist(new_seg.p3, goal) < goal_tolerance){
            ROS_INFO("Solution found!");
            random_tree.success = 1;
            return random_tree;
        }
        
        //ROS_INFO("Value of i is: ");
        //std::cout<<"value of i is: "<<std::endl;
        //std::cout<<i<<std::endl;
        i = i + 1;
    }

    
    random_tree.success = 1;
    return random_tree;
    
}



/**
 * @brief Get the path to goal
 * 
 * @param tree  rrt tree constructed
 * @param goal  
 * @return ** std::vector <node> 
 */

bool get_path(const cbb_rrt_optimal_tree &tree, std::vector<geometry_msgs::PoseStamped>* plan,
                   const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal){


    ROS_INFO("tree size is");
    std::cout<<tree.tree.size()<<std::endl;
    ROS_INFO("get path function initated");

    std::vector<bezier_seg> path_segments;

    geometry_msgs::PoseStamped pose_stamped;

    pose_stamped.header.frame_id = "map";
    // i guess specifiying orientation in plan is very important.
    tf2::Quaternion quat_tf;
    geometry_msgs::Quaternion quat_msg;
    plan->clear();
    node xi, xg;
    xg.point.x = goal.pose.position.x;
    xg.point.y = goal.pose.position.y;
    xg.point.z = 0;
    xi.point.x = start.pose.position.x;
    xi.point.y = start.pose.position.y;
    //ROS_INFO("goal location is: ");
    //std::cout<<xg.point.x<<","<<xg.point.y<<std::endl;
    //ROS_INFO("start location is: ");
    //std::cout<<xi.point.x<<xi.point.y<<std::endl;
    //ROS_INFO("find the closest point to goal node");
    node closest_to_goal = get_nearest_node(tree,xg);
    bezier_seg connectng_segment = tree.tree[closest_to_goal.index];

    path_segments.push_back(connectng_segment);
    //std::cout<<closest_to_goal.point.x<<","<<closest_to_goal.point.y<<std::endl;
  

    while(connectng_segment.parent != -1){
        connectng_segment = tree.tree[connectng_segment.parent];
        path_segments.push_back(connectng_segment);
    }
    // push the initial segment  into tree!
    bezier_seg init_segment = tree.tree[0];
    path_segments.push_back(init_segment);

    // reverse the order!
    std::reverse(path_segments.begin(),path_segments.end());
    // now push the points in bezier curve to plan!
    
    pose_stamped.pose.orientation = goal.pose.orientation;

    

    // push pose and orientation to plan
    for(int c = 0; c<path_segments.size()-1;++c){
        int t = path_segments[c].t_c*path_segments[c].parts -1; // take one point before the connecting joint to avoid repitition!
        //int t = 0.95*path_segments[c].parts;
        for(int g = 0 ; g<=t; ++g){
            pose_stamped.pose.position.x = path_segments[c].bezier_seg_points[g].x;
            pose_stamped.pose.position.y = path_segments[c].bezier_seg_points[g].y;
            pose_stamped.pose.position.z = 0;
            
            
            quat_tf.setRPY(0, 0, path_segments[c].slope_vals[g]);
            quat_msg = tf2::toMsg(quat_tf);
            pose_stamped.pose.orientation = quat_msg;
            plan->push_back(pose_stamped);
            
        }
    }

    bezier_seg last_seg = path_segments.back();
    // push the final segment!
    for(int g = 0; g < last_seg.bezier_seg_points.size()-1; ++g){

        pose_stamped.pose.position.x = last_seg.bezier_seg_points[g].x;
        pose_stamped.pose.position.y = last_seg.bezier_seg_points[g].y;
        pose_stamped.pose.position.z = 0;

        
        
        quat_tf.setRPY(0, 0, last_seg.slope_vals[g]);
        quat_msg = tf2::toMsg(quat_tf);
        pose_stamped.pose.orientation = quat_msg;
        plan->push_back(pose_stamped);

    }
    // push the last point!
    pose_stamped.pose.position.x = last_seg.bezier_seg_points.back().x;
    pose_stamped.pose.position.y = last_seg.bezier_seg_points.back().y;
    pose_stamped.pose.position.z = 0;
    pose_stamped.pose.orientation = goal.pose.orientation;
    plan->push_back(pose_stamped);

    /*
    while (closest_to_goal.parent != -1){
        //std::cout<<&closest_to_goal->point.x<<&closest_to_goal->point.y<<std::endl;
        ROS_INFO("INDEX OF NODE");
        std::cout<<closest_to_goal.index<<std::endl;
        ROS_INFO("PARENT OF CLOSEST");
        std::cout<<closest_to_goal.parent<<std::endl;

        pose_stamped.pose.position.x = closest_to_goal.point.x;
        //ROS_INFO("the next x cordinate!");
        //std::cout<<closest_to_goal.point.x<<std::endl;
        pose_stamped.pose.position.y = closest_to_goal.point.y;
        pose_stamped.pose.position.z = 0;
        //std::cout<<closest_to_goal->point.x<<","<<closest_to_goal->point.y<<std::endl;
        plan->push_back(pose_stamped);
        std::cout<<closest_to_goal.point.x<<","<<closest_to_goal.point.y<<std::endl;
        node prev = closest_to_goal;
        closest_to_goal = tree.tree[closest_to_goal.parent];
        // oritentation regarding each step
        double dx, dy, yaw;
        dx = closest_to_goal.point.x - prev.point.x;
        dy = closest_to_goal.point.y - prev.point.x;
        yaw = atan2(dy,dx);

        quat_tf.setRPY(0, 0, yaw);
        // Convert Quat TF to msg
        quat_msg = tf2::toMsg(quat_tf);
        pose_stamped.pose.orientation = quat_msg;
        //std::cout<<closest_to_goal<<std::endl;
        //std::cout<<&closest_to_goal->point
    }
    // add the initial position
    pose_stamped.pose.position.x = closest_to_goal.point.x;
    //ROS_INFO("the next x cordinate!");
    //std::cout<<closest_to_goal.point.x<<std::endl;
    pose_stamped.pose.position.y = closest_to_goal.point.y;
    pose_stamped.pose.position.z = 0;
    pose_stamped.pose.orientation = start.pose.orientation;
    plan->push_back(pose_stamped);
    std::reverse(plan->begin(),plan->end());
    */    
    

    return true;
}