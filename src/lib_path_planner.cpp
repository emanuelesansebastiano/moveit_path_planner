/* Author: Emanuele Sansebastiano
   Desc: Library to encapsulate some functions useful to accomplish the "path Planning" final project based on Baxter.
         Amazon Robotics Challenge - Universidad Jaume I (Spain) [Competitor]
*/

// this pkg
#include <moveit_path_planner/lib_path_planner.h>

namespace msf = moveit_side_functions;
namespace mbf = moveit_basics_functions;
namespace of = obj_functions;

//////////////////////////////////////////////////////////////////////////////////////
// VALUES NOT MODIFIABLE BY THE USER \\

const std::string error_string = "Warning: An invalid input has been inserted!";
//useful char codes '%' is used for the comments inside the files
//DO NOT CHANGE THIS SYMBOLS! Do it just if you are sure you fully understood this library
const char comment_symbol = '%';

//////////////////////////////////////////////////////////////////////////////////////

namespace planner_functions
{
  //global variables
  visualization_msgs::InteractiveMarkerInit gl_rviz_marker_init;
  visualization_msgs::InteractiveMarkerUpdate gl_rviz_marker_up;
  moveit_msgs::PlanningScene gl_moveit_pl_scene;
  moveit_msgs::PlanningSceneWorld gl_moveit_pl_scene_w;

  std::vector< std::vector< std::vector< double > > > mapCell_generator(double depth_x, double width_y, double height_z, double cell_size)
  {
	  namespace msf = moveit_side_functions;

	  int temp_dim[3];
	  double temp_double[3];
	  std::vector< double > temp_vector;
	  std::vector< std::vector< std::vector< double > > > map;

	  if(cell_size <= 0.0)
	  {
		  std::cout << "Error: the cell size must be strictly positive!" << std::endl;
		  cell_size = 0.05;
		  std::cout << "its value has been modified to " << cell_size << "(default value) to make the program work" << std::endl;
	  }

	  temp_double[0] = depth_x/cell_size;
	  temp_double[1] = width_y/cell_size;
	  temp_double[2] = height_z/cell_size;

	  for(int i = 0; i < 3; i++)
	  {
		  if((int)temp_double[i] % 2 != 0)
			  temp_dim[i] = (int)temp_double[i];
		  else
			  temp_dim[i] = (int)temp_double[i] +1;
	  }

	  map.resize(temp_dim[0]);
	  for(int i = 0; i < temp_dim[0]; i++)
	  {
		  map[i].resize(temp_dim[1]);
		  for(int j = 0; j < temp_dim[1]; j++)
		  {
			  map[i][j].resize(temp_dim[2]);
		  }
	  }

	  return map;
  }

  std::vector<int> XYZtoIJK(std::vector< std::vector< std::vector< double > > > &map, double cell_size, geometry_msgs::Vector3 XYZ_location)
  {
	  std::vector<int> IJK2return; IJK2return.resize(3);
	  namespace msf = moveit_side_functions;


	  //main program
	  XYZ_location.x /= cell_size;
	  XYZ_location.y /= cell_size;
	  XYZ_location.z /= cell_size;

	  XYZ_location.x += map.size()/2;
	  XYZ_location.y += map[0].size()/2;
	  XYZ_location.z += map[0][0].size()/2;

	  IJK2return[0] = msf::round_f(XYZ_location.x);
	  IJK2return[1] = msf::round_f(XYZ_location.y);
	  IJK2return[2] = msf::round_f(XYZ_location.z);

	  return IJK2return;
  }

  geometry_msgs::Vector3 IJKtoXYZ(std::vector< std::vector< std::vector< double > > > &map, double cell_size, std::vector<int> IJK_location)
  {
	  geometry_msgs::Vector3 XYZ2return;
	  double temp_dim[3];


	  //main program
	  temp_dim[0] = map.size()/2;
	  temp_dim[1] = map[0].size()/2;
	  temp_dim[2] = map[0][0].size()/2;

	  XYZ2return.x = IJK_location[0] - temp_dim[0];
	  XYZ2return.y = IJK_location[1] - temp_dim[1];
	  XYZ2return.z = IJK_location[2] - temp_dim[2];

	  XYZ2return.x *= cell_size;
	  XYZ2return.y *= cell_size;
	  XYZ2return.z *= cell_size;

	  return XYZ2return;
  }

  bool mapCell_saturator(std::vector< std::vector< std::vector< double > > > &map, double saturation_value)
  {
	  int temp_int;

	  //function check
	  //matrix size check
	  temp_int = map[0].size();
	  for(int i = 0; i < map.size(); i++)
	  {
		  if(map[i].size() != temp_int){
			  std::cout << "Error: the raws of the first matrix have NOT constant size!" << std::endl;
			  goto return_0;
		  }
	  }
	  temp_int = map[0][0].size();
	  for(int i = 0; i < map.size(); i++)
	  {
		  for(int j = 0; j < map[0].size(); j++)
		  {
			  if(map[i][j].size() != temp_int){
				  std::cout << "Error: the depth lines of the first matrix have NOT constant size!" << std::endl;
				  goto return_0;
			  }
		  }
	  }

	  //main program
	  for(int i = 0; i < map.size(); i++)
	  {
		  for(int j = 0; j < map[0].size(); j++)
		  {
			  for(int z = 0; z < map[0][0].size(); z++)
			  {
				  if(map[i][j][z] > saturation_value)
					  map[i][j][z] = saturation_value;
			  }
		  }
	  }

	  return true;

	  return_0:
	  return false;
  }

  bool mapCell_normalization(std::vector< std::vector< std::vector< double > > > &map, double normal_value)
   {
 	  int temp_int;
 	  double temp_double;

 	  //function check
 	  //matrix size check
 	  temp_int = map[0].size();
 	  for(int i = 0; i < map.size(); i++)
 	  {
 		  if(map[i].size() != temp_int){
 			  std::cout << "Error: the raws of the first matrix have NOT constant size!" << std::endl;
 			  goto return_0;
 		  }
 	  }
 	  temp_int = map[0][0].size();
 	  for(int i = 0; i < map.size(); i++)
 	  {
 		  for(int j = 0; j < map[0].size(); j++)
 		  {
 			  if(map[i][j].size() != temp_int){
 				  std::cout << "Error: the depth lines of the first matrix have NOT constant size!" << std::endl;
 				  goto return_0;
 			  }
 		  }
 	  }

 	  //main program

 	  //research of the higher value in the map
 	  temp_double = 0.0;
 	  for(int i = 0; i < map.size(); i++)
 	  {
 		  for(int j = 0; j < map[0].size(); j++)
 		  {
 			  for(int z = 0; z < map[0][0].size(); z++)
 			  {
 				  if(map[i][j][z] > temp_double)
 				  {
 					  temp_double = map[i][j][z];
 				  }
 			  }
 		  }
 	  }

 	  //normalizator
 	  temp_double = normal_value/temp_double;

	  //normalization
 	  for(int i = 0; i < map.size(); i++)
 	  {
 		  for(int j = 0; j < map[0].size(); j++)
 		  {
 			  for(int z = 0; z < map[0][0].size(); z++)
 			  {
 				  map[i][j][z] *= temp_double;
 			  }
 		  }
 	  }


 	  return true;

 	  return_0:
 	  return false;
   }


  bool mapCell_objsInsertion(std::vector< std::vector< std::vector< double > > > &map, double cell_size, std::vector<moveit_msgs::CollisionObject> coll_objs)
  {
	  namespace msf = moveit_side_functions;
	  namespace gsf = geometry_side_functions;

	  int temp_int;
	  geometry_msgs::Pose temp_pose;
	  geometry_msgs::Vector3 temp_vector;
	  std::vector<geometry_msgs::Vector3> set_point_coord;

	  std::vector <std::vector <double> > temp_matrix;
	  std::vector <std::vector <double> > rot_matrix;
	  std::vector <std::vector <double> > trans_matrix;
	  std::vector <std::vector <double> > point_matrix;

	  //function check
	  //matrix size check
	  temp_int = map[0].size();
	  for(int i = 0; i < map.size(); i++)
	  {
		  if(map[i].size() != temp_int){
			  std::cout << "Error: the raws of the first matrix have NOT constant size!" << std::endl;
			  goto return_0;
		  }
	  }
	  temp_int = map[0][0].size();
	  for(int i = 0; i < map.size(); i++)
	  {
		  for(int j = 0; j < map[0].size(); j++)
		  {
			  if(map[i][j].size() != temp_int){
				  std::cout << "Error: the depth lines of the first matrix have NOT constant size!" << std::endl;
				  goto return_0;
			  }
		  }
	  }

	  //cell size check
	  if(cell_size <= 0.0)
	  {
		  std::cout << "Error: the cell size must be strictly positive!" << std::endl;
		  cell_size = 0.05;
		  std::cout << "its value has been modified to " << cell_size << "(default value) to make the program work" << std::endl;
	  }


	  //main program
	  //for every collision object passed in the list
	  for(int i = 0; i < coll_objs.size(); i++)
	  {
		  //object cells generation
		  temp_pose = coll_objs[i].primitive_poses[0];

		  //object point list generation (the object is oriented as the world frame)
		  set_point_coord.clear(); set_point_coord.resize(0);

		  //sphere
		  if(coll_objs[i].primitives[0].type == shape_msgs::SolidPrimitive::SPHERE)
		  {
			  //std::cout << "sphere" << std::endl;

			  double radius = coll_objs[i].primitives[0].dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS];

			  double curr_z = 0.0;
			  while(curr_z*curr_z <= radius*radius)
			  {
				  double curr_y = 0.0;
				  while(curr_y*curr_y + curr_z*curr_z <= radius*radius)
				  {
					  double curr_x = 0.0;
					  while(curr_x*curr_x + curr_y*curr_y + curr_z*curr_z <= radius*radius)
					  {
						  set_point_coord.push_back(msf::makeVector3(-curr_x,-curr_y,-curr_z));
						  set_point_coord.push_back(msf::makeVector3(-curr_x,-curr_y,curr_z));
						  set_point_coord.push_back(msf::makeVector3(-curr_x,curr_y,-curr_z));
						  set_point_coord.push_back(msf::makeVector3(-curr_x,curr_y,curr_z));
						  set_point_coord.push_back(msf::makeVector3(curr_x,-curr_y,-curr_z));
						  set_point_coord.push_back(msf::makeVector3(curr_x,-curr_y,curr_z));
						  set_point_coord.push_back(msf::makeVector3(curr_x,curr_y,-curr_z));
						  set_point_coord.push_back(msf::makeVector3(curr_x,curr_y,curr_z));

						  curr_x += cell_size;
					  }
					  curr_y += cell_size;
				  }
				  curr_z += cell_size;
			  }

		  }//box
		  else if(coll_objs[i].primitives[0].type == shape_msgs::SolidPrimitive::BOX)
		  {
			  //std::cout << "box" << std::endl;

			  geometry_msgs::Vector3 box_size;
			  box_size.x = coll_objs[i].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X];
			  box_size.y = coll_objs[i].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
			  box_size.z = coll_objs[i].primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z];

			  double curr_z = 0.0;
			  while(curr_z <= box_size.z/2)
			  {
				  double curr_y = 0.0;
				  while(curr_y <= box_size.y/2)
				  {
					  double curr_x = 0.0;
					  while(curr_x <= box_size.x/2)
					  {
						  set_point_coord.push_back(msf::makeVector3(-curr_x,-curr_y,-curr_z));
						  set_point_coord.push_back(msf::makeVector3(-curr_x,-curr_y,curr_z));
						  set_point_coord.push_back(msf::makeVector3(-curr_x,curr_y,-curr_z));
						  set_point_coord.push_back(msf::makeVector3(-curr_x,curr_y,curr_z));
						  set_point_coord.push_back(msf::makeVector3(curr_x,-curr_y,-curr_z));
						  set_point_coord.push_back(msf::makeVector3(curr_x,-curr_y,curr_z));
						  set_point_coord.push_back(msf::makeVector3(curr_x,curr_y,-curr_z));
						  set_point_coord.push_back(msf::makeVector3(curr_x,curr_y,curr_z));

						  curr_x += cell_size;
					  }
					  curr_y += cell_size;
				  }
				  curr_z += cell_size;
			  }

		  }//cylinder
		  else if(coll_objs[i].primitives[0].type == shape_msgs::SolidPrimitive::CYLINDER)
		  {
			  //std::cout << "cylinder" << std::endl;

			  double height = coll_objs[i].primitives[0].dimensions[0];
			  double radius = coll_objs[i].primitives[0].dimensions[1];

			  double curr_z = 0.0;
			  while(curr_z <= height/2)
			  {
				  double curr_y = 0.0;
				  while(curr_y*curr_y <= radius*radius)
				  {
					  double curr_x = 0.0;
					  while(curr_x*curr_x + curr_y*curr_y <= radius*radius)
					  {
						  set_point_coord.push_back(msf::makeVector3(-curr_x,-curr_y,-curr_z));
						  set_point_coord.push_back(msf::makeVector3(-curr_x,-curr_y,curr_z));
						  set_point_coord.push_back(msf::makeVector3(-curr_x,curr_y,-curr_z));
						  set_point_coord.push_back(msf::makeVector3(-curr_x,curr_y,curr_z));
						  set_point_coord.push_back(msf::makeVector3(curr_x,-curr_y,-curr_z));
						  set_point_coord.push_back(msf::makeVector3(curr_x,-curr_y,curr_z));
						  set_point_coord.push_back(msf::makeVector3(curr_x,curr_y,-curr_z));
						  set_point_coord.push_back(msf::makeVector3(curr_x,curr_y,curr_z));

						  curr_x += cell_size;
					  }
					  curr_y += cell_size;
				  }
				  curr_z += cell_size;
			  }

		  }//cone
		  else if(coll_objs[i].primitives[0].type == shape_msgs::SolidPrimitive::CONE)
		  {
			  //std::cout << "cone" << std::endl;

			  double height = coll_objs[i].primitives[0].dimensions[0];
			  double radius_base = coll_objs[i].primitives[0].dimensions[1];
			  double radius;

			  double curr_z = 0.0;
			  while(curr_z <= height)
			  {
				  radius = radius_base*(height-curr_z)/height;
				  double curr_y = 0.0;
				  while(curr_y*curr_y <= radius*radius)
				  {
					  double curr_x = 0.0;
					  while(curr_x*curr_x + curr_y*curr_y <= radius*radius)
					  {
						  set_point_coord.push_back(msf::makeVector3(-curr_x,-curr_y,curr_z-height/2));
						  set_point_coord.push_back(msf::makeVector3(-curr_x,curr_y,curr_z-height/2));
						  set_point_coord.push_back(msf::makeVector3(curr_x,-curr_y,curr_z-height/2));
						  set_point_coord.push_back(msf::makeVector3(curr_x,curr_y,curr_z-height/2));

						  curr_x += cell_size;
					  }
					  curr_y += cell_size;
				  }
				  curr_z += cell_size;
			  }
		  }

		  //orientation change world_frame to object one
		  msf::posePosition2vector3(temp_pose, temp_vector);
		  rot_matrix = gsf::Quaternion2rotMatrix(temp_pose.orientation);
		  trans_matrix = gsf::translation_matrix(temp_vector);

		  for(int p = 0; p < set_point_coord.size(); p++)
		  {
			  point_matrix = gsf::point2matrix_gen(set_point_coord[p]);

			  msf::matrix2Dprod(trans_matrix, rot_matrix, temp_matrix);
			  msf::matrix2Dprod(temp_matrix, point_matrix, temp_matrix);

			  set_point_coord[p] = gsf::matrix2point_gen(temp_matrix);

			  //map insertion
			  //Assuming the center on the world is located in the center of the map
			  //we must calculate that the dimension from 0 to half of the full size is negative.
			  temp_vector.x = set_point_coord[p].x/cell_size;
			  temp_vector.y = set_point_coord[p].y/cell_size;
			  temp_vector.z = set_point_coord[p].z/cell_size;

			  //find the 8 points around this one
			  //if the point coordinate exceed the map size
			  if((int)msf::abs_f(temp_vector.x) +1 > (map.size()/2)-1
					  || (int)msf::abs_f(temp_vector.y) +1 > (map[0].size()/2)-1
					  || (int)msf::abs_f(temp_vector.z) +1 > (map[0][0].size()/2)-1)
			  {
				  std::cout << "Warning: the point having coordinates (" << set_point_coord[p].x << ", " << set_point_coord[p].y << ", " << set_point_coord[p].z << ") is outside of the map workspace" << std::endl;
				  std::cout << "The system is not including it in the path planning analysis" << std::endl;
			  }

			  map[(int)(1 +temp_vector.x +map.size()/2)][(int)(1 +temp_vector.y +map[0].size()/2)][(int)(1 +temp_vector.z +map[0][0].size()/2)] += 1.0;
			  map[(int)(1 +temp_vector.x +map.size()/2)][(int)(1 +temp_vector.y +map[0].size()/2)][(int)(temp_vector.z +map[0][0].size()/2)] += 1.0;
			  map[(int)(1 +temp_vector.x +map.size()/2)][(int)(temp_vector.y +map[0].size()/2)][(int)(1 +temp_vector.z +map[0][0].size()/2)] += 1.0;
			  map[(int)(1 +temp_vector.x +map.size()/2)][(int)(temp_vector.y +map[0].size()/2)][(int)(temp_vector.z +map[0][0].size()/2)] += 1.0;
			  map[(int)(temp_vector.x +map.size()/2)][(int)(1 +temp_vector.y +map[0].size()/2)][(int)(1 +temp_vector.z +map[0][0].size()/2)] += 1.0;
			  map[(int)(temp_vector.x +map.size()/2)][(int)(1 +temp_vector.y +map[0].size()/2)][(int)(temp_vector.z +map[0][0].size()/2)] += 1.0;
			  map[(int)(temp_vector.x +map.size()/2)][(int)(temp_vector.y +map[0].size()/2)][(int)(1 +temp_vector.z +map[0][0].size()/2)] += 1.0;
			  map[(int)(temp_vector.x +map.size()/2)][(int)(temp_vector.y +map[0].size()/2)][(int)(temp_vector.z +map[0][0].size()/2)] += 1.0;

			  map[(int)(1 +temp_vector.x +map.size()/2)][(int)(1 +temp_vector.y +map[0].size()/2)][(int)(-1 +temp_vector.z +map[0][0].size()/2)] += 1.0;
			  map[(int)(1 +temp_vector.x +map.size()/2)][(int)(temp_vector.y +map[0].size()/2)][(int)(-1 +temp_vector.z +map[0][0].size()/2)] += 1.0;
			  map[(int)(temp_vector.x +map.size()/2)][(int)(1 +temp_vector.y +map[0].size()/2)][(int)(-1 +temp_vector.z +map[0][0].size()/2)] += 1.0;
			  map[(int)(temp_vector.x +map.size()/2)][(int)(temp_vector.y +map[0].size()/2)][(int)(-1 +temp_vector.z +map[0][0].size()/2)] += 1.0;

		  }
	  }

	  return true;

	  return_0:
	  return false;
  }

  std::vector<geometry_msgs::Vector3> mapCell_extractor(std::vector< std::vector< std::vector< double > > > &map, double cell_size, double saturation_value)
  {
	  std::vector<geometry_msgs::Vector3> coord2return;
	  int temp_int;

	  //function check
	  //cell_size positive
	  if(cell_size <= 0.0)
	  {
		  std::cout << "Error: the cell size must be strictly positive!" << std::endl;
		  std::cout << "A null vector has been returned" << std::endl;
		  return coord2return;
	  }
	  //matrix size check
	  temp_int = map[0].size();
	  for(int i = 0; i < map.size(); i++)
	  {
		  if(map[i].size() != temp_int){
			  std::cout << "Error: the raws of the first matrix have NOT constant size!\nNull vector has been returned..." << std::endl;
			  return coord2return;
		  }
	  }
	  temp_int = map[0][0].size();
	  for(int i = 0; i < map.size(); i++)
	  {
		  for(int j = 0; j < map[0].size(); j++)
		  {
			  if(map[i][j].size() != temp_int){
				  std::cout << "Error: the depth lines of the first matrix have NOT constant size!\nNull vector has been returned..." << std::endl;
				  return coord2return;
			  }
		  }
	  }

	  double temp_size[3];
	  geometry_msgs::Vector3 temp_vector;

	  //main program
	  for(int i = 0; i < map.size(); i++)
	  {
		  for(int j = 0; j < map[0].size(); j++)
		  {
			  for(int z = 0; z < map[0][0].size(); z++)
			  {
				  if(map[i][j][z] >= saturation_value)
				  {
					  temp_size[0] = map.size()/2; temp_size[1] = map[0].size()/2; temp_size[2] = map[0][0].size()/2;
					  temp_vector = msf::makeVector3((i-temp_size[0])*cell_size,(j-temp_size[1])*cell_size,(z-temp_size[2])*cell_size);
					  coord2return.push_back(temp_vector);
				  }
			  }
		  }
	  }

	  return coord2return;
  }

  std::vector<std::vector<int>> mapCell_extractor_IJK(std::vector< std::vector< std::vector< double > > > &map, double saturation_value)
  {
	  std::vector<std::vector<int>> coord2return;
	  std::vector<int> temp_coord; temp_coord.resize(3);
	  int temp_int;

	  //function check
	  //matrix size check
	  temp_int = map[0].size();
	  for(int i = 0; i < map.size(); i++)
	  {
		  if(map[i].size() != temp_int){
			  std::cout << "Error: the raws of the first matrix have NOT constant size!\nNull vector has been returned..." << std::endl;
			  return coord2return;
		  }
	  }
	  temp_int = map[0][0].size();
	  for(int i = 0; i < map.size(); i++)
	  {
		  for(int j = 0; j < map[0].size(); j++)
		  {
			  if(map[i][j].size() != temp_int){
				  std::cout << "Error: the depth lines of the first matrix have NOT constant size!\nNull vector has been returned..." << std::endl;
				  return coord2return;
			  }
		  }
	  }

	  double temp_size[3];

	  //main program
	  for(int i = 0; i < map.size(); i++)
	  {
		  for(int j = 0; j < map[0].size(); j++)
		  {
			  for(int z = 0; z < map[0][0].size(); z++)
			  {
				  if(map[i][j][z] >= saturation_value)
				  {
					  temp_coord[0] = i; temp_coord[1] = j; temp_coord[2] = z;
					  coord2return.push_back(temp_coord);
				  }
			  }
		  }
	  }

	  return coord2return;
  }

  std::vector< std::vector< std::vector< double > > > mapCell_potential_field_attraction(std::vector< std::vector< std::vector< double > > > &map, double cell_size, double wave_step, geometry_msgs::Vector3 goal_location)
  {
	  namespace msf = moveit_side_functions;

	  std::vector< std::vector< std::vector< double > > > map2return;
	  int temp_int;
	  //initial check

	  if(wave_step <= 0.0)
	  {
		  std::cout << "Error: the wave step must be strictly positive!" << std::endl;
		  std::cout << "A default empty map has been returned" << std::endl;
		  return map2return;
	  }

	  //cell_size positive
	  if(cell_size <= 0.0)
	  {
		  std::cout << "Error: the cell size must be strictly positive!" << std::endl;
		  std::cout << "A default empty map has been returned" << std::endl;
		  return map2return;
	  }

	  //function check
	  //matrix size check
	  temp_int = map[0].size();
	  for(int i = 0; i < map.size(); i++)
	  {
		  if(map[i].size() != temp_int){
			  std::cout << "Error: the raws of the first matrix have NOT constant size!\nA default empty map has been returned..." << std::endl;
			  return map2return;
		  }
	  }
	  temp_int = map[0][0].size();
	  for(int i = 0; i < map.size(); i++)
	  {
		  for(int j = 0; j < map[0].size(); j++)
		  {
			  if(map[i][j].size() != temp_int){
				  std::cout << "Error: the depth lines of the first matrix have NOT constant size!\nA default empty map has been returned..." << std::endl;
				  return map2return;
			  }
		  }
	  }

	  //main program start
	  std::vector<int> temp_matrix_pos; temp_matrix_pos.resize(3);
	  std::vector<std::vector<int>> matrix_coor_vector;
	  std::vector<std::vector<int>> temp_matrix_coor_vector;
	  std::vector<std::vector<int>> temp_matrix_coor_vector2;
	  std::vector<std::vector<int>> temp_27_matrix_coor_vector;
	  temp_27_matrix_coor_vector.resize(27);
	  for(int i = 0; i < temp_27_matrix_coor_vector.size(); i++)
		  temp_27_matrix_coor_vector[i].resize(3);
	  int vec[3];
	  int progression;
	  bool done, present;

	  // put the goal position into the matrix definition
	  temp_matrix_pos = planner_functions::XYZtoIJK(map, cell_size, goal_location);

	  //wave algorithm
	  map2return = map;
	  planner_functions::mapCell_saturator(map2return, wave_step);

	  matrix_coor_vector.push_back(temp_matrix_pos);

	  //initial check on goal position
	  if(map2return[matrix_coor_vector[0][0]][matrix_coor_vector[0][1]][matrix_coor_vector[0][2]] != 0.0)
	  {
		  std::cout << "Error:The goal position is located inside of an obstacle." << std::endl;
		  std::cout << "The value contained in the cell (" << matrix_coor_vector[0][0] << ", " << matrix_coor_vector[0][1] << ", " << matrix_coor_vector[0][2] << ") is: " << map2return[matrix_coor_vector[0][0]][matrix_coor_vector[0][1]][matrix_coor_vector[0][2]] << std::endl;
	  }


	  done = false;
	  progression = 0;
	  while(!done)
	  {
		  //potential value increasing
		  progression++;

		  //potential value update
		  for(int i = 0; i < matrix_coor_vector.size(); i++)
		  {
			  map2return[matrix_coor_vector[i][0]][matrix_coor_vector[i][1]][matrix_coor_vector[i][2]] = wave_step*progression;
		  }

		  //reset of the layer
		  temp_matrix_coor_vector.clear();

		  temp_matrix_coor_vector = matrix_coor_vector;
		  temp_matrix_coor_vector2.clear();
		  matrix_coor_vector.clear();

		  for(int i = 0; i < temp_matrix_coor_vector.size(); i++)
		  {

			  //value insertion (generation of the cube including the considered point)
			  int c = 0;
			  for(int p1 = 0; p1 < 3; p1++)
			  {
				  for(int p2 = 0; p2 < 3; p2++)
				  {
					  for(int p3 = 0; p3 < 3; p3++)
					  {
						  if(p1 == 2)
							  vec[0] = -1;
						  else
							  vec[0] = p1;
						  if(p2 == 2)
							  vec[1] = -1;
						  else
							  vec[1] = p2;
						  if(p3 == 2)
							  vec[2] = -1;
						  else
							  vec[2] = p3;

						  temp_27_matrix_coor_vector[c][0] = temp_matrix_coor_vector[i][0] + vec[0];
						  temp_27_matrix_coor_vector[c][1] = temp_matrix_coor_vector[i][1] + vec[1];
						  temp_27_matrix_coor_vector[c][2] = temp_matrix_coor_vector[i][2] + vec[2];

						  c++;
					  }
				  }
			  }

			  for(int z = 0; z < temp_27_matrix_coor_vector.size(); z++)
			  {
				  //new layer addiction
				  //point not outside of the matrix
				  if(temp_27_matrix_coor_vector[z][0] >= 0 &&
						  temp_27_matrix_coor_vector[z][1] >= 0 &&
						  temp_27_matrix_coor_vector[z][2] >= 0 &&
						  temp_27_matrix_coor_vector[z][0] < map2return.size() &&
						  temp_27_matrix_coor_vector[z][1] < map2return[0].size() &&
						  temp_27_matrix_coor_vector[z][2] < map2return[0][0].size())
				  {
					  //if the point has not been already touched by the algorithm map or it is not a wall
					  if(map2return[temp_27_matrix_coor_vector[z][0]][temp_27_matrix_coor_vector[z][1]][temp_27_matrix_coor_vector[z][2]] < wave_step)
					  {
						  //insertion in the new layer
						  temp_matrix_coor_vector2.push_back(temp_27_matrix_coor_vector[z]);
					  }
				  }
			  }
		  }

		  //delete all the duplicated points
		  for(int i = 0; i < temp_matrix_coor_vector2.size(); i++)
		  {
			  present = false;
			  for(int k = 0; k < i; k++)
			  {
				  if(temp_matrix_coor_vector2[i][0] == temp_matrix_coor_vector2[k][0] && temp_matrix_coor_vector2[i][1] == temp_matrix_coor_vector2[k][1] && temp_matrix_coor_vector2[i][2] == temp_matrix_coor_vector2[k][2])
				  {
					  present = true;
					  break;
				  }
			  }
			  if(!present)
				  matrix_coor_vector.push_back(temp_matrix_coor_vector2[i]);
		  }

		  //exit condition: there are no cells to cover yet
		  if(matrix_coor_vector.size() == 0)
			  done = true;

		  //feedback for the user
		  if(progression % 10 == 0)
			  std::cout << "potential field expansion number: " << progression << std::endl;
	  }

	  return map2return;
  }

  std::vector< std::vector< std::vector< double > > > mapCell_potential_field_repulsion(std::vector< std::vector< std::vector< double > > > &map, double cell_size, double radius_repulsive_field)
  {
	  namespace msf = moveit_side_functions;

	  std::vector< std::vector< std::vector< double > > > map2return;
	  int temp_int;
	  //initial check

	  if(radius_repulsive_field <= 0.0)
	  {
		  std::cout << "Error: the radius of the repulsive field must be strictly positive!" << std::endl;
		  std::cout << "A default empty map has been returned" << std::endl;
		  return map2return;
	  }

	  //cell_size positive
	  if(cell_size <= 0.0)
	  {
		  std::cout << "Error: the cell size must be strictly positive!" << std::endl;
		  std::cout << "A default empty map has been returned" << std::endl;
		  return map2return;
	  }

	  if(radius_repulsive_field < cell_size)
	  {
		  std::cout << "Warning: the radius of the repulsive field is smaller than the cell size. It means there is not repulsive field..." << std::endl;
	  }

	  //function check
	  //matrix size check
	  temp_int = map[0].size();
	  for(int i = 0; i < map.size(); i++)
	  {
		  if(map[i].size() != temp_int){
			  std::cout << "Error: the raws of the first matrix have NOT constant size!\nA default empty map has been returned..." << std::endl;
			  return map2return;
		  }
	  }
	  temp_int = map[0][0].size();
	  for(int i = 0; i < map.size(); i++)
	  {
		  for(int j = 0; j < map[0].size(); j++)
		  {
			  if(map[i][j].size() != temp_int){
				  std::cout << "Error: the depth lines of the first matrix have NOT constant size!\nA default empty map has been returned..." << std::endl;
				  return map2return;
			  }
		  }
	  }


	  //main program start
	  int cell_involved = 1 + radius_repulsive_field/cell_size;

	  std::vector<std::vector<int>> matrix_coor_vector;
	  std::vector<std::vector<int>> temp_matrix_coor_vector;
	  std::vector<std::vector<int>> temp_matrix_coor_vector2;
	  std::vector<std::vector<int>> temp_27_matrix_coor_vector;
	  temp_27_matrix_coor_vector.resize(27);
	  for(int i = 0; i < temp_27_matrix_coor_vector.size(); i++)
		  temp_27_matrix_coor_vector[i].resize(3);
	  double sat_val;
	  int vec[3];
	  int progression;
	  bool done, present;

	  //repulsive algorithm
	  map2return = map;
	  progression = 0;
	  sat_val = 1.0*(cell_involved-progression)/cell_involved;

	  planner_functions::mapCell_saturator(map2return,sat_val);
	  matrix_coor_vector = planner_functions::mapCell_extractor_IJK(map2return, sat_val);

	  //check obstacle existence
	  if(matrix_coor_vector.size() == 0)
		  std::cout << "Warning: apparently there are not obstacle in the map..." << std::endl;

	  done = false;
	  while(!done)
	  {

		  temp_matrix_coor_vector.clear();

		  temp_matrix_coor_vector = matrix_coor_vector;
		  temp_matrix_coor_vector2.clear();
		  matrix_coor_vector.clear();

		  for(int i = 0; i < temp_matrix_coor_vector.size(); i++)
		  {
			  //value insertion (generation of the cube including the considered point)
			  int c = 0;
			  for(int p1 = 0; p1 < 3; p1++)
			  {
				  for(int p2 = 0; p2 < 3; p2++)
				  {
					  for(int p3 = 0; p3 < 3; p3++)
					  {

						  if(p1 == 2)
							  vec[0] = -1;
						  else
							  vec[0] = p1;
						  if(p2 == 2)
							  vec[1] = -1;
						  else
							  vec[1] = p2;
						  if(p3 == 2)
							  vec[2] = -1;
						  else
							  vec[2] = p3;

						  temp_27_matrix_coor_vector[c][0] = temp_matrix_coor_vector[i][0] + vec[0];
						  temp_27_matrix_coor_vector[c][1] = temp_matrix_coor_vector[i][1] + vec[1];
						  temp_27_matrix_coor_vector[c][2] = temp_matrix_coor_vector[i][2] + vec[2];

						  c++;
					  }
				  }
			  }

			  for(int z = 0; z < temp_27_matrix_coor_vector.size(); z++)
			  {
				  //new layer addiction
				  //point not outside of the matrix
				  if(temp_27_matrix_coor_vector[z][0] >= 0 &&
						  temp_27_matrix_coor_vector[z][1] >= 0 &&
						  temp_27_matrix_coor_vector[z][2] >= 0 &&
						  temp_27_matrix_coor_vector[z][0] < map2return.size() &&
						  temp_27_matrix_coor_vector[z][1] < map2return[0].size() &&
						  temp_27_matrix_coor_vector[z][2] < map2return[0][0].size())
				  {
					  //if the point has not been already touched by the algorithm map or it is not a wall
					  if(map2return[temp_27_matrix_coor_vector[z][0]][temp_27_matrix_coor_vector[z][1]][temp_27_matrix_coor_vector[z][2]] == 0.0)
					  {
						  //insertion in the new layer
						  temp_matrix_coor_vector2.push_back(temp_27_matrix_coor_vector[z]);
					  }
				  }
			  }
		  }


		  //delete all the duplicated points
		  for(int i = 0; i < temp_matrix_coor_vector2.size(); i++)
		  {
			  present = false;
			  for(int k = 0; k < i; k++)
			  {
				  if(temp_matrix_coor_vector2[i][0] == temp_matrix_coor_vector2[k][0] && temp_matrix_coor_vector2[i][1] == temp_matrix_coor_vector2[k][1] && temp_matrix_coor_vector2[i][2] == temp_matrix_coor_vector2[k][2])
				  {
					  present = true;
					  break;
				  }
			  }
			  if(!present)
			  {
				  matrix_coor_vector.push_back(temp_matrix_coor_vector2[i]);
			  }
		  }

		  progression++;
		  for(int i = 0; i < matrix_coor_vector.size(); i++)
		  {
			  sat_val = 1.0*(cell_involved-progression)/cell_involved;
			  //values around objects
			  map2return[matrix_coor_vector[i][0]][matrix_coor_vector[i][1]][matrix_coor_vector[i][2]] = sat_val;
			  //values on the edge of the map could be introduced, but it does not improve the computation

		  }

		  //exit condition: there are no cells to cover yet or the system reached the edge of the potential field
		  if(matrix_coor_vector.size() == 0 || cell_involved-progression <= 1)
			  done = true;

		  //feedback for the user
		  if(progression % 10 == 0)
			  std::cout << "potential field expansion number: " << progression << std::endl;
	  }

  return map2return;
  }

  std::vector< std::vector< std::vector< double > > > mapCell_potential_field_full(std::vector< std::vector< std::vector< double > > > &map, double cell_size, double attractive_wave_step, double radius_repulsive_field, geometry_msgs::Vector3 goal_location, double norm_val)
  {
	  namespace msf = moveit_side_functions;
	  std::vector< std::vector< std::vector< double > > > map_1 = mapCell_potential_field_attraction(map, cell_size, attractive_wave_step, goal_location);
	  std::vector< std::vector< std::vector< double > > > map_2 = mapCell_potential_field_repulsion(map, cell_size, radius_repulsive_field);

	  mapCell_normalization(map_1, norm_val);
	  mapCell_normalization(map_2, norm_val);

	  msf::matrix3DSUM(map_1,map_2,map_1);

	  mapCell_normalization(map_1, norm_val);

	  return map_1;
  }

  std::vector<std::vector<int>> mapcell_potential_field_path_extractor_IJK(std::vector< std::vector< std::vector< double > > > &map, double cell_size, geometry_msgs::Vector3 starting_point_location)
  {
	  int temp_int;
	  std::vector<std::vector<int>> path_points;
	  std::vector<int> temp_vec;

	  //cell_size positive
	  if(cell_size <= 0.0)
	  {
		  std::cout << "Error: the cell size must be strictly positive!" << std::endl;
		  std::cout << "A default empty map has been returned" << std::endl;
		  return path_points;
	  }

	  //function check
	  //matrix size check
	  temp_int = map[0].size();
	  for(int i = 0; i < map.size(); i++)
	  {
		  if(map[i].size() != temp_int){
			  std::cout << "Error: the raws of the first matrix have NOT constant size!\nA default empty map has been returned..." << std::endl;
			  return path_points;
		  }
	  }
	  temp_int = map[0][0].size();
	  for(int i = 0; i < map.size(); i++)
	  {
		  for(int j = 0; j < map[0].size(); j++)
		  {
			  if(map[i][j].size() != temp_int){
				  std::cout << "Error: the depth lines of the first matrix have NOT constant size!\nA default empty map has been returned..." << std::endl;
				  return path_points;
			  }
		  }
	  }

	  //main variables
	  int c;
	  int vec[3];
	  bool new_found;
	  double temp_double1, temp_double2;
	  std::vector<std::vector<int>> temp_27_matrix_coor_vector; temp_27_matrix_coor_vector.resize(27);
	  for(int i = 0; i < 27; i++)
		  temp_27_matrix_coor_vector[i].resize(3);

	  //main program
	  temp_vec = XYZtoIJK(map, cell_size, starting_point_location);

	  //research of the minimum value in the map
	  temp_double2 = map[0][0][0];
	  for(int p1 = 0; p1 < map.size(); p1++)
	  {
		  for(int p2 = 0; p2 < map[0].size(); p2++)
		  {
			  for(int p3 = 0; p3 < map[0][0].size(); p3++)
			  {
				  if(map[p1][p2][p3] < temp_double2)
					  temp_double2 = map[p1][p2][p3];
			  }
		  }
	  }


	  //main loop
	  temp_double1 = map[temp_vec[0]][temp_vec[1]][temp_vec[2]];
	  while(temp_double1 > temp_double2)
	  {
		  //value insertion (generation of the cube including the considered point)
		  int c = 0;
		  for(int p1 = 0; p1 < 3; p1++)
		  {
			  for(int p2 = 0; p2 < 3; p2++)
			  {
				  for(int p3 = 0; p3 < 3; p3++)
				  {

					  if(p1 == 2)
						  vec[0] = -1;
					  else
						  vec[0] = p1;
					  if(p2 == 2)
						  vec[1] = -1;
					  else
						  vec[1] = p2;
					  if(p3 == 2)
						  vec[2] = -1;
					  else
						  vec[2] = p3;

					  temp_27_matrix_coor_vector[c][0] = temp_vec[0] + vec[0];
					  temp_27_matrix_coor_vector[c][1] = temp_vec[1] + vec[1];
					  temp_27_matrix_coor_vector[c][2] = temp_vec[2] + vec[2];

					  c++;
				  }
			  }
		  }

		  //current minimum value of the investigation
		  new_found = false;
		  for(int i = 0; i < temp_27_matrix_coor_vector.size(); i++)
		  {
			  if(map[temp_27_matrix_coor_vector[i][0]][temp_27_matrix_coor_vector[i][1]][temp_27_matrix_coor_vector[i][2]] < temp_double1)
			  {
				  //temp_double1 is used for the while exit condition
				  //temp_vec is going to be re-used in the new loop
				  temp_double1 = map[temp_27_matrix_coor_vector[i][0]][temp_27_matrix_coor_vector[i][1]][temp_27_matrix_coor_vector[i][2]];
				  temp_vec = temp_27_matrix_coor_vector[i];
				  new_found = true;
			  }
		  }

		  if(new_found)
		  {
			  //new path point stored in the list
			  path_points.push_back(temp_vec);
		  }else{
			  std::cout << "A local minimum has been found..." << std::endl;
			  std::cout << "The " << path_points.size() << " points found until now have been returned..." << std::endl;
			  return path_points;
		  }
	  }

	  return path_points;
  }

  std::vector<geometry_msgs::Vector3> mapcell_potential_field_path_extractor_XYZ(std::vector< std::vector< std::vector< double > > > &map, double cell_size, geometry_msgs::Vector3 starting_point_location)
  {
	  //all the checks are performed by 'mapcell_potential_field_path_extractor_IJK' function
	  std::vector<std::vector<int>> vectorIJK = mapcell_potential_field_path_extractor_IJK(map, cell_size, starting_point_location);
	  std::vector<geometry_msgs::Vector3> vectorXYZ;
	  geometry_msgs::Vector3 temp_vec;

	  //conversion XYK --> XYZ
	  for(int i = 0; i < vectorIJK.size(); i++)
	  {
		  temp_vec = IJKtoXYZ(map,cell_size,vectorIJK[i]);
		  vectorXYZ.push_back(temp_vec);
	  }

	  return vectorXYZ;
  }


  //Callback for the function 'getRvizInterMarkInit' | global variable
  void callback_RIMI(const visualization_msgs::InteractiveMarkerInit data){
	  gl_rviz_marker_init = data;
	  //to check if the callback function is running uncomment the following line
	  //std::cout << "callback_RIMI is working" << std::endl;
  }
  visualization_msgs::InteractiveMarkerInit getRvizInterMarkInit(std::string topic_str, ros::NodeHandle &nh)
  {
	visualization_msgs::InteractiveMarkerInit scene_markers_init;
	if(msf::CheckTopicExistence(topic_str)){
		ros::Subscriber sub = nh.subscribe <visualization_msgs::InteractiveMarkerInit>(topic_str, 10, callback_RIMI);
		double curr_time = 0.0;
		while(scene_markers_init.markers.size() == 0 && curr_time < exit_time)
		{
			scene_markers_init = gl_rviz_marker_init;
			curr_time += std_sleep_time;
			if(scene_markers_init.markers.size() == 0){
				curr_time += std_sleep_time;
				ros::Duration(std_sleep_time).sleep();
				ros::spinOnce();
			}
		}

		if(curr_time >= exit_time)
			std::cout << "The exit time has been reached (" << exit_time << "). An empty message has been returned by." << std::endl;

		//re.initialization of global variable
		visualization_msgs::InteractiveMarkerInit default_gl_rviz_marker_init;
		gl_rviz_marker_init = default_gl_rviz_marker_init;

	}else{
		std::cout << "Warning: the topic '" << topic_str << "' does not exist! Probably you are using another topic definition or the robot is still not publishing. ";
		std::cout << "Check how is the topic defined in you robot and change the topic definition in the #define part of the header file." << std::endl;
		std::cout << "This function returned an empty message by default." << std::endl;
	}

	return scene_markers_init;
  }

  //Callback for the function 'getRvizInterMarkUpdate' | global variable
  void callback_RIMU(const visualization_msgs::InteractiveMarkerUpdate data){
	  gl_rviz_marker_up = data;
	  //to check if the callback function is running uncomment the following line
	  //std::cout << "callback_RIMU is working" << std::endl;
  }
  visualization_msgs::InteractiveMarkerUpdate getRvizInterMarkUpdate(std::string topic_str, ros::NodeHandle &nh)
  {
	visualization_msgs::InteractiveMarkerUpdate scene_markers_up;
	if(msf::CheckTopicExistence(topic_str)){
		ros::Subscriber sub = nh.subscribe <visualization_msgs::InteractiveMarkerUpdate>(topic_str, 10, callback_RIMU);
		double curr_time = 0.0;
		while(scene_markers_up.markers.size() == 0 && curr_time < exit_time)
		{
			scene_markers_up = gl_rviz_marker_up;
			curr_time += std_sleep_time;
			if(scene_markers_up.markers.size() == 0){
				curr_time += std_sleep_time;
				ros::Duration(std_sleep_time).sleep();
				ros::spinOnce();
			}
		}

		if(curr_time >= exit_time)
			std::cout << "The exit time has been reached (" << exit_time << "). An empty message has been returned by." << std::endl;

		//re.initialization of global variable
		visualization_msgs::InteractiveMarkerUpdate default_gl_rviz_marker_up;
		gl_rviz_marker_up = default_gl_rviz_marker_up;

	}else{
		std::cout << "Warning: the topic '" << topic_str << "' does not exist! Probably you are using another topic definition or the robot is still not publishing. ";
		std::cout << "Check how is the topic defined in you robot and change the topic definition in the #define part of the header file." << std::endl;
		std::cout << "This function returned an empty message by default." << std::endl;
	}

	return scene_markers_up;
  }

  //Callback for the function 'getMoveitPlanningScene' | global variable
  void callback_PlScene(const moveit_msgs::PlanningScene data){
	  gl_moveit_pl_scene = data;
	  //to check if the callback function is running uncomment the following line
	  //std::cout << "callback_RIMU is working" << std::endl;
  }
  moveit_msgs::PlanningScene getMoveitPlanningScene(std::string topic_str, ros::NodeHandle &nh)
  {
	moveit_msgs::PlanningScene moveit_pl_scene;
	if(msf::CheckTopicExistence(topic_str)){
		ros::Subscriber sub = nh.subscribe <moveit_msgs::PlanningScene>(topic_str, 10, callback_PlScene);
		double curr_time = 0.0;
		while(moveit_pl_scene.robot_model_name.size() == 0 && curr_time < exit_time)
		{
			moveit_pl_scene = gl_moveit_pl_scene;
			curr_time += std_sleep_time;
			if(moveit_pl_scene.robot_model_name.size() == 0){
				curr_time += std_sleep_time;
				ros::Duration(std_sleep_time).sleep();
				ros::spinOnce();
			}
		}

		if(curr_time >= exit_time)
			std::cout << "The exit time has been reached (" << exit_time << "). An empty message has been returned by." << std::endl;

		//re.initialization of global variable
		moveit_msgs::PlanningScene default_gl_moveit_pl_scene;
		gl_moveit_pl_scene = default_gl_moveit_pl_scene;

	}else{
		std::cout << "Warning: the topic '" << topic_str << "' does not exist! Probably you are using another topic definition or the robot is still not publishing. ";
		std::cout << "Check how is the topic defined in you robot and change the topic definition in the #define part of the header file." << std::endl;
		std::cout << "This function returned an empty message by default." << std::endl;
	}

	return moveit_pl_scene;
  }

  //Callback for the function 'getMoveitPlanningSceneWorld' | global variable
  void callback_PlSceneW(const moveit_msgs::PlanningSceneWorld data){
	  gl_moveit_pl_scene_w = data;
	  //to check if the callback function is running uncomment the following line
	  //std::cout << "callback_RIMU is working" << std::endl;
  }
  moveit_msgs::PlanningSceneWorld getMoveitPlanningSceneWorld(std::string topic_str, ros::NodeHandle &nh)
  {
	moveit_msgs::PlanningSceneWorld moveit_pl_scene_w;
	if(msf::CheckTopicExistence(topic_str)){
		ros::Subscriber sub = nh.subscribe <moveit_msgs::PlanningSceneWorld>(topic_str, 10, callback_PlSceneW);
		double curr_time = 0.0;
		while(moveit_pl_scene_w.collision_objects.size() == 0 && curr_time < exit_time)
		{
			moveit_pl_scene_w = gl_moveit_pl_scene_w;
			curr_time += std_sleep_time;
			if(moveit_pl_scene_w.collision_objects.size() == 0){
				curr_time += std_sleep_time;
				ros::Duration(std_sleep_time).sleep();
				ros::spinOnce();
			}
		}

		if(curr_time >= exit_time)
			std::cout << "The exit time has been reached (" << exit_time << "). An empty message has been returned by." << std::endl;

		//re.initialization of global variable
		moveit_msgs::PlanningSceneWorld default_gl_moveit_pl_scene_w;
		gl_moveit_pl_scene_w = default_gl_moveit_pl_scene_w;

	}else{
		std::cout << "Warning: the topic '" << topic_str << "' does not exist! Probably you are using another topic definition or the robot is still not publishing. ";
		std::cout << "Check how is the topic defined in you robot and change the topic definition in the #define part of the header file." << std::endl;
		std::cout << "This function returned an empty message by default." << std::endl;
	}

	return moveit_pl_scene_w;
  }


// End namespace "planner_functions"
}
