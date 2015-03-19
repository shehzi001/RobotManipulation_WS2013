// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 #include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <vector>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>
#include <urdf/model.h>
using namespace std;
using namespace KDL;
typedef std::map< std::string,KDL::TreeElement>::iterator Iter;

 const char* joint_names[] = {
		"base_link"
		"arm_joint_1",
		"arm_joint_2",
		"arm_joint_3",
		"arm_joint_4",
		"arm_joint_5",
		"gripper_finger_joint_l",
		"gripper_finger_joint_r"
};

double positions[5] = 
	{
	 2.9496, 1.13446, -2.54818, 1.78896, 2.93075
	};
	
const double upper_joints_limits[5] = {5.84014,2.61799,-0.015708,3.4292,5.64159};
const double lower_joints_limits[5] = {0.0100692,0.0100692,-5.02655,0.0221239,0.110619};

KDL::Tree tree;
KDL::Chain chain;


void rand_cart_pose(Vector &cart_positions)
{    
	double theta = lower_joints_limits[0] + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_joints_limits[0]-lower_joints_limits[0])));
	double phi =  lower_joints_limits[1] + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_joints_limits[1]-lower_joints_limits[1])));
	double radius_of_workspace = 0.655;
	cart_positions[0] =  radius_of_workspace * abs(cos(theta) * sin (phi));
	cart_positions[1] =  radius_of_workspace * abs(sin(theta) * sin(phi));
	cart_positions[2] =  radius_of_workspace * abs(cos(phi));
	//printf("\n[x,y,z]=[%f,%f,%f]",cart_positions[0],cart_positions[1],cart_positions[2]);
}


bool urdf2tree()
{
	std::string urdf_file = "/home/shehzad/ros_ws/hydro/catkin_ws/src/RobotManipulation_WS2013/kdl_parser_test/src/youbotarm.urdf";
		urdf::Model model;
	if (!model.initFile(urdf_file))
	{
		ROS_ERROR("Failed to parse urdf file");
		return false;
	}
	else
	{
		ROS_INFO("Successfully parsed urdf file");
	}
	
	if (!kdl_parser::treeFromUrdfModel(model, tree))
	{
		ROS_ERROR("Failed to build tree");
		return false;
	}
	return true;
}
bool tree2chain()
{
	return tree.getChain("arm_link_0","arm_link_5",chain);
}


int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "my_parser");
	
	//**************local variable declaration*******************
	unsigned int nj = 0;
	unsigned int js = 0;
	double roll = 0.0 , pitch = 0.0 , yaw = 0.0;
	//******************************************
	
	if(!urdf2tree())
		return 0;
	
	if(!tree2chain())
		return 0;
		
	//**************************************************
		nj = chain.getNrOfJoints();
		js = chain.getNrOfSegments();
		ROS_INFO("[Segments,joints]:[%d,%d]",js,nj);
	//**************************************************
		
		KDL::JntArray q(nj);
		// Assign some values to the joint positions
		printf("\nFKinput:[");
		for(unsigned int i = 0;i < 5;i++)
		{
			q(i)=positions[i];
		}
		printf("]\n");
		//cout << "\nForward kinematics solver....." << endl;
		cout << "\nFK solver result:" << endl;
		// Calculate forward position kinematics
		KDL::ChainFkSolverPos_recursive fksolver(chain);
		KDL::Frame cartpos;
		bool kinematics_status = fksolver.JntToCart(q,cartpos,js);
		
		KDL::Rotation R;
		R = cartpos.M;
		
		R.GetRPY(roll,pitch,yaw);
		
		if(kinematics_status >= 0)
		{
			std::cout << cartpos << std::endl;
			printf("\n[Roll,Pitch,Yaw] = [%f,%f,%f]\n",roll,pitch,yaw);
		}
		else{
			printf("%s \n kinematic status:,%d","Error: could not calculate forward kinematics :(" ,kinematics_status);
		}
		
		
		/*
		Vector p(-0.392042,  0.00244385,   0.0315553);
		Rotation R(0.779067,  0.00623352,    0.626909,
		  -0.00485643,    0.999981, -0.00390793,
			-0.626921,-1.35245e-17,    0.779083);
		cartpos.p = p;
		cartpos.M = R;
		//Vector rand_cart_p(0,0,0);
		//rand_cart_pose(rand_cart_p);
		//Frame rand0_t;
		//rand0_t.p = rand_cart_p;
		Vector tool_tip(0,0,0.105);
		//cartpos.p = cartpos.p + tool_tip ;
		Frame T0_t =  cartpos;
		Frame T5_t(tool_tip);
		Frame Tt_5 = T5_t.Inverse();
		
		cout << "T0_t" << T0_t << endl;
		cout << "Tt_5" << Tt_5 << endl;
		Frame T0_5 =  T0_t *  Tt_5;
		
		cout << "T0_5" << T0_5 << endl;
		*/

  return 0;
}
/*
 * 	//std::map<std::string,TreeElement>::const_iterator root;
	//std::map<std::string,TreeElement> segments;
    //segments = tree.getSegments();
	//root = tree.getRootSegment();
	* 
	* 
		//const KDL::Segment &segment9 = chain.getSegment(3);
		//const std::string &name = segment9.getName();
		//std::cout << segment9.pose(1.0) << std::endl;
		//std::cout << name.length() << std::endl;
		//std::cout << name.c_str() << std::endl;
		
		KDL::JntArray min_angles(5);
		KDL::JntArray max_angles(5);
		double init_positions[5] = 
			{
				2.9496, 1.13446, -2.54818, 1.78896, 2.93075
			};
		for (int i = 0 ; i < 5;i++){
			max_angles(i) = upper_joints_limits[i];
			min_angles(i) = lower_joints_limits[i];
		}
		KDL::ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver
		KDL::ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
		KDL::ChainIkSolverPos_NR_JL iksolver1(chain,min_angles,max_angles,fksolver1,iksolver1v,500,1e-6);
		
		//Creation of jntarrays:
		KDL::JntArray q_init(nj);
		
		for(unsigned int i = 0;i < 5;i++)
		{
			q_init(i)=init_positions[i];
		}
		
		int ret = iksolver1.CartToJnt(q_init,T0_5,q);
		
		if(ret >= 0)
		{
			//std::cout << "Joint positions:\n" << q.data <<std::endl;
			printf("IK:[");
			for(unsigned int i = 0;i < nj;i++)
			{
				printf("%0.7lf,",q(i));
			}
			//printf("%s,\n","Succes Ik!");
			printf("]\n");
			return true;
			
		}
		else
		{
			printf("%s \n","Error: could not calculate Inverse kinematics :(");
			return false;
		}
		
*/
