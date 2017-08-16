 /*Author: Emanuele Sansebastiano */

// my libs
#include <moveit_path_planner/lib_path_planner.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "planner_test");
	ros::NodeHandle node_handle("~");

	ros::AsyncSpinner spinner(1);

	spinner.start();

	namespace msf = moveit_side_functions;
	namespace mbf = moveit_basics_functions;
	namespace of = obj_functions;

	namespace plf = planner_functions;

	//moveit initialization
	static const std::string PLANNING_GROUP = "both_arms";
	moveit::planning_interface::MoveGroup move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	ros::Duration(2.0).sleep();

	//real program init

	int c_val = 2;
	for(int i = 0; i < c_val; i++)
	{
		std::cout << "countdown: " << c_val-1-i << std::endl;
		ros::Duration(1.0).sleep();
	}


	std::vector<moveit_msgs::CollisionObject> coll_objs;
	std::vector<geometry_msgs::PoseStamped> arm_pose_s;

	for(int j = 0; j < 2; j++)
	{
		if (j == 0)
			arm_pose_s = mbf::TF_arm_point_pose("left");
		else
			arm_pose_s = mbf::TF_arm_point_pose("right");
		std::vector<geometry_msgs::Pose> arm_pose = msf::PoseStamped2Pose(arm_pose_s);

		for(int i = 0; i < arm_pose.size(); i++)
		{
			std::string temp_name = arm_pose_s[i].header.frame_id;
			geometry_msgs::Vector3 temp_vec3;
			temp_vec3.x = arm_pose[i].position.x;
			temp_vec3.y = arm_pose[i].position.y;
			temp_vec3.z = arm_pose[i].position.z;
			moveit_msgs::CollisionObject temp_obj = mbf::collision_obj_generator(temp_name, temp_vec3, arm_pose[i].orientation, 0.1, 0.02);
			coll_objs.push_back(temp_obj);
		}

		of::addObject(planning_scene_interface, coll_objs);
	}

	for(double i = 0.0; i < 360.1; i +=1.0)
	{
		geometry_msgs::Vector3  vect = msf::makeVector3(0.0, 180.0, i);
		geometry_msgs::Quaternion quat_t = msf::RPY2Quat(vect, false, false);
		//std::cout << vect.x << " | " << vect.y << " | " << vect.z << std::endl;

		//std::cout << quat_t.x << " | " << quat_t.y << " | " << quat_t.z << " | " << quat_t.w  << std::endl;

		vect = msf::Quat2RPY(quat_t,false);

		std::cout << vect.x << " | " << vect.y << " | " << vect.z << std::endl;
	}

	ros::shutdown();
	return 0;
}
