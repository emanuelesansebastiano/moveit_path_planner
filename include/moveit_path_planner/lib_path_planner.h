/* Author: Emanuele Sansebastiano
   Desc: Library to encapsulate some functions useful to accomplish the "path Planning" final project based on Baxter.
         Amazon Robotics Challenge - Universidad Jaume I (Spain) Competitor
*/

#ifndef MOVEIT_PATH_PLANNER_H
#define MOVEIT_PATH_PLANNER_H

// ROS
#include <ros/ros.h>
#include <visualization_msgs/InteractiveMarkerInit.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>

// Official Moveit! libraries
#include <moveit_msgs/PlanningScene.h>

// Unofficial Moveit! libraries
#include <moveit_side_pkg/side_functions.h>

// C++
#include <geometry_msgs/PoseStamped.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <stdio.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>


//////////////////////////////////////////////////////////////////////////////////////
// VALUES MODIFIABLE BY THE USER \\

#define exit_time					5.5
#define std_sleep_time				0.02

//////////////////////////////////////////////////////////////////////////////////////


namespace planner_functions
{
  //brief: Function to generate the map divided in cells so that X-axis is the front axis, Y-axis is the left_right axis, Z-axis is the vertical axis
  //       Having a map with odd values for every dimension is preferred due to further calculations, so this map has odd sizes
  std::vector< std::vector< std::vector< double > > > mapCell_generator(double depth_x, double width_y, double height_z, double cell_size);

  //brief: Function to saturate the values of a mapCell matrix
  bool mapCell_saturator(std::vector< std::vector< std::vector< double > > > &map, double saturation_value = 1.0);

  //brief: Function to insert a vector of objet into the path planner scene reducing it in cells having values
  bool mapCell_objsInsertion(std::vector< std::vector< std::vector< double > > > &map, double cell_size, std::vector<moveit_msgs::CollisionObject> coll_objs);

  //brief: Function to extract the real world coordinates of the cells having the value bigger than the 'saturation_value'
  //       The world frame is assumed to be in the center of this map and that this map has just odd dimensions
  std::vector<geometry_msgs::Vector3> mapCell_extractor(std::vector< std::vector< std::vector< double > > > &map, double cell_size, double saturation_value = 1.0);



  //brief: Function to get every marker (balls to move the end_point) listed in the rviz scene (InteractiveMarkerInit msg)
  visualization_msgs::InteractiveMarkerInit getRvizInterMarkInit(std::string topic_str, ros::NodeHandle &nh);

  //brief: Function to get every marker (balls to move the end_point) listed in the rviz scene (InteractiveMarkerUpdate msg)
  visualization_msgs::InteractiveMarkerUpdate getRvizInterMarkUpdate(std::string topic_str, ros::NodeHandle &nh);

  //brief: Function to get the planning scene (PlanningScene)
  moveit_msgs::PlanningScene getMoveitPlanningScene(std::string topic_str, ros::NodeHandle &nh);

  //brief: Function to get the planning scene (PlanningSceneWorld)
  moveit_msgs::PlanningSceneWorld getMoveitPlanningSceneWorld(std::string topic_str, ros::NodeHandle &nh);


// End namespace "planner_functions"
}


#endif /* MOVEIT_PATH_PLANNER_H */


