
#include <pluginlib/class_list_macros.h>
#include "cbb_rrt_star_planner.h"

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(cbb_rrt_star_global_planner::cbb_rrt_star_planner, nav_core::BaseGlobalPlanner)


using namespace std;

// Default Constructor
namespace cbb_rrt_star_global_planner
{
cbb_rrt_star_planner::cbb_rrt_star_planner()
{
}

cbb_rrt_star_planner::cbb_rrt_star_planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}
void cbb_rrt_star_planner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!tree_initalized)
  {
    ROS_INFO("Initializing rrt_planner.");
    name_ = name; 
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    std::cout<<costmap_->getOriginX()+costmap_->getSizeInMetersX()<<","<<costmap_->getOriginX()<<std::endl;
    std::cout<<costmap_->getSizeInCellsX()<<std::endl;
    std::cout<<costmap_->getResolution()<<std::endl;

    footprint = costmap_ros_->getRobotFootprint();  // footprint is padded by footprint_padding rosparam
    robot_radius = get_robot_radius(footprint);
    
    ros::NodeHandle nh("~/" + name);
    // Retrieve RRT parameters (or set to default values)
    nh.param("goal_tolerance", goal_tolerance, 0.1);
    nh.param("K", K, 1000);
    nh.param("step_size", step_size, 2.0);
    nh.param("optimization_radius", optimization_radius, 0.5);
    nh.param("critical_curvature",critical_curvature, 1.0/robot_radius);
    //nh.param("viz_tree", viz_tree, false);

    plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
    /*
    if (viz_tree)
    {
      tree_pub_ = nh.advertise<visualization_msgs::Marker>("tree", 1);
    }
    */
   
    tree_initalized = true;
  }
}

bool cbb_rrt_star_planner::makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan){
       
                ros::Time s1 = ros::Time::now(); 
                if (!tree_initalized){
                  ROS_INFO("RRT star planner was not initialized, doing it now!");
                  initialize(name_,costmap_ros_);
                  tree_initalized = true;
                }
                node start_, goal_;
                start_.point.x = start.pose.position.x;
                start_.point.y = start.pose.position.y;
                start_.orientation.orientation = start.pose.orientation;
                goal_.point.x = goal.pose.position.x;
                goal_.point.y = goal.pose.position.y;
                
                
                cbb_rrt_optimal_tree rrt_Tree = get_random_tree(start_,goal_,K,optimization_radius,step_size,robot_radius,critical_curvature,goal_tolerance,costmap_ros_);
                std::cout<<"rrt planner running"<<std::endl;
                std::cout<<rrt_Tree.success<<std::endl;
                if(rrt_Tree.success){
                  ROS_INFO("Found path!");
                  //ROS_INFO("I have the power!!");
                  ros::Duration s2 = ros::Time::now() - s1;
                  std::cout<< s2<<std::endl;
                  get_path(rrt_Tree, &plan, start, goal);
                  publish_plan(plan);
                  return true;
                }
                else{
                  ROS_INFO("no path found using cubic bezier rrt planner!");
                  return false;
                }

             
                }
void cbb_rrt_star_planner::publish_plan( std::vector<geometry_msgs::PoseStamped>& plan){
   // create a message for the plan
  //ROS_INFO("PATH Recieved");
  //for(int i = 0; i<plan.size(); ++i){
    //std::cout<<plan[i].pose.position.x<<plan[i].pose.position.y<<std::endl;
  //}
  nav_msgs::Path rviz_path;
  rviz_path.poses.resize(plan.size());

  if (plan.empty())
  {
    // still set a valid frame so visualization won't hit transform issues
    rviz_path.header.frame_id = "map";
    rviz_path.header.stamp = ros::Time::now();
  }
  else
  {
    rviz_path.header.frame_id = plan[0].header.frame_id;
    rviz_path.header.stamp = plan[0].header.stamp;
  }

  // Extract the plan in world co-ordinates, we assume the plan is all in the same frame
  for (unsigned int i = 0; i < plan.size(); i++)
  {
    rviz_path.poses[i] = plan[i];
  }

  plan_pub_.publish(rviz_path);


}

};  // namespace global_planner
