// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 #include <ros/ros.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

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

#include <iostream>
#include <math.h>
#include <kdl/jacobian.hpp>
#include <boost/scoped_ptr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <Eigen/Geometry>


using namespace Eigen;
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
double pre_grasp[5] = {2.93836, 2.020597, -1.88253, 3.36243, 3.01283};
double candle[5] = {2.9496, 1.13446, -2.54818, 1.78896, 2.93075};



class kinematic_solver
{

public:

	kinematic_solver()
	{
		//if(!urdf2tree())
		//return 0;
	  
		//if(!tree2chain())
		//	return 0;
		urdf2tree();
		tree2chain();
		//**************************************************
			nj = chain.getNrOfJoints();
			js = chain.getNrOfSegments();
			ROS_INFO("[Segments,joints]:[%d,%d]",js,nj);
		//**************************************************
	}
	
	void urdf2tree()
	{
		std::string urdf_file = "/home/shehzad/ros_ws/hydro/catkin_ws/src/RobotManipulation_WS2013/kdl_parser_test/src/youbotarm.urdf";
			urdf::Model model;
		if (!model.initFile(urdf_file))
		{
			ROS_ERROR("Failed to parse urdf file");
			//return false;
		}
		else
		{
			ROS_INFO("Successfully parsed urdf file");
		}
		
		if (!kdl_parser::treeFromUrdfModel(model, tree))
		{
			ROS_ERROR("Failed to build tree");
			//return false;
		}
		//return true;
	}
	
	void tree2chain()
	{
		tree.getChain("arm_link_0","arm_link_5",chain);
	}

	//FK solver from base to tool tip
	bool FK_joint_cart(Eigen::VectorXd joint_positions,Eigen::VectorXd &eefPose)
	{
		KDL::JntArray q(nj);
		KDL::Frame cartpos;
		eefPose.resize(6);
		// Assign some values to the joint positions
		for(unsigned int i = 0;i < nj;i++)
		{
			q(i)=joint_positions[i];
		}
		
		// Calculate forward position kinematics
		KDL::ChainFkSolverPos_recursive fksolver (chain);
		kinematics_status = fksolver.JntToCart(q, cartpos, js);
		
		if(kinematics_status >= 0)
		{
			 cartpos.M.GetRPY(eefPose(3),eefPose(4),eefPose(5)); 
			 eefPose(0) = cartpos.p[0];
			 eefPose(1) = cartpos.p[1];
			 eefPose(2) = cartpos.p[2];
			return true;
		}
		else
		{
			printf("%s \n", "Error: could not calculate forward kinematics :(");
			
			return false;
		}

	}
	
	void KDLJctoEigenJc(KDL::Jacobian J_KDL)
	{
		jacobian = MatrixXd::Zero(6,nj);
		
		for (unsigned int i = 0 ; i < 6 ; i++)
		{
		 for (unsigned int j = 0 ; j < nj ; j++)
		   jacobian(i,j) = J_KDL(i,j);
		}
	}
	
	KDL::JntArray KDLJStoEigenJS(Eigen::VectorXd joints_state)
	{
		
		KDL::JntArray q_;
		
		q_.resize(nj);
		
		for (unsigned int i = 0 ; i < nj ; i++)
		{
		   q_(i) = joints_state(i);
		}
		return q_;
	}
	
	void computerJacobian(KDL::JntArray q_)
	{
		KDL::Jacobian  J_;
		
		J_.resize(chain.getNrOfJoints());

		jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(chain));

		jnt_to_jac_solver_->JntToJac(q_, J_);
		
		//cout << "KDL jacoabian:";
		//for(int i = 0;i < nj;i++)
		//cout << J_.getColumn(i) << endl;
		
		
		KDLJctoEigenJc(J_);
		
	}
	
	Eigen::MatrixXd jacobian_inv(Eigen::VectorXd joints_state)
	{
		computerJacobian(KDLJStoEigenJS(joints_state));
		
		Eigen::MatrixXd matrix_temp = jacobian * jacobian.transpose();
		
		
		if(matrix_temp.determinant() != 0)
		{
			jacPseudoInv = jacobian.transpose() * ((matrix_temp).inverse());
			//std::cout << "\n===============================" << endl;
			//std::cout << "Eigen jacobian:\n " << jacobian << std::endl;
			//std::cout << "jacPseudoInv:\n " << jacPseudoInv << std::endl;
			//std::cout << "\n===============================" << endl;
			//computePivot();
			//computerQR();
		}
		return jacPseudoInv;
	}
   
   
  void IKSolver(void)
	{
		Eigen::VectorXd current_joint_state(5);
		
		Eigen::VectorXd goal_joint_state(5);
		
		std::vector<double> arm_joint_angles(5);
		
		Eigen::VectorXd new_joint_angles(5);
		
		Eigen::VectorXd error(6);
		
		for(int i = 0;i < 5;i++)
		{
		 current_joint_state[i] = pre_grasp[i];
		 goal_joint_state[i] = candle[i];
		 arm_joint_angles[i] = pre_grasp[i];
		}
    
		Eigen::VectorXd initial_pose,goal_pose,temp_pose; 
		
		FK_joint_cart(current_joint_state,initial_pose);   //initial Pose;
		
		FK_joint_cart(goal_joint_state,goal_pose); //final Pose;
		
		//change final pose 
		 //goal_pose[0] = -0.1; 
		//goal_pose[1] = 0.02;
		//goal_pose[2] -= 0.05;
		
		
		//std::cout<<"Generating plan to goal pose:\n" << goal_pose.transpose() << std::endl;

		Eigen::VectorXd deltaX = goal_pose - initial_pose;
		std::cout << "\nerror:" <<  error << std::endl;
		sleep(2);
		unsigned int iter = 0;
		
		while(iter < 500)
		{
			//goal_pose[2] -= 0.01; 

			 new_joint_angles = current_joint_state + jacobian_inv(current_joint_state) * (deltaX);
			 
			//Eigen::VectorXd newPose = moveit_fk(new_joint_angles);
			double error  = ((Eigen::Matrix<double,6,6>::Identity() - jacobian * jacPseudoInv) * (deltaX)).norm();
			
			//std::cout<<"\n\nStart arm configuration:"<< current_joint_state.transpose() << std::endl;
			
			//std::cout<<"New arm configuration:"<< new_joint_angles.transpose()<< std::endl;
			
			cout << "==================" << iter << "=========" << endl;
			cout << "initial_pose :" << initial_pose.transpose() << endl;
		  cout << "goal_pose :" << goal_pose.transpose() << endl;
			std::cout << "\nerror:" <<  error << std::endl;
			cout << "==========================================" << endl;
			
			if(error <= 0.2)
			{
				//std::cout << "\nerror:" <<  error << std::endl;
				
				//moveit_move_arm(arm_joint_angles);
				
				//initial_pose = goal_pose;
				current_joint_state = new_joint_angles;
				
				FK_joint_cart(current_joint_state,temp_pose);
				deltaX = goal_pose - temp_pose;
				//sleep(5);
				iter++;
			}
			else
				deltaX = 0.5 * deltaX;
			
			if(error < 0.001)
			{
				//if(checkJointLimits(new_joint_angles))
				{
					for(int i=0;i < 5 ;i++)
					{
					 arm_joint_angles[i] = new_joint_angles[i]; 
					}
					cout << "Solution found with intermediate Points : " <<  iter << endl;
					cout << "Moving arm to goal Pose::" << new_joint_angles << endl;
					//moveit_move_arm(arm_joint_angles);
				}
				//else
				//	ROS_INFO_STREAM("No solution found Joint Limit Violation.");
				 break;
			 }
			
			if(iter == 99)
				 std::cout << "\nNo solution found with error difference:" <<  error << std::endl;
		 }
	}

public:
	unsigned int nj,js;
	
private:
	KDL::Chain chain;
	KDL::Tree tree;
	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
	KDL::JntArray  q0_;
	// Create the frame that will contain the results
	KDL::Frame cartpos;
	KDL::Twist xdot_;
	bool kinematics_status;
	Eigen::MatrixXd jacobian,jacPseudoInv;
	KDL::Frame T[8];
};


KDL::JntArray doubleArray2JntArray(double joints[])
{
		KDL::JntArray  q0_;
		q0_.resize(5);
  //updating initial guess
	for(int i = 0; i < 5; i++)
	{
	 q0_(i) = joints[i];
	}
	return q0_;
}




int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "my_parser");
	
	//**************local variable declaration*******************
	unsigned int nj = 0;
	unsigned int js = 0;
	double roll = 0.0 , pitch = 0.0 , yaw = 0.0;
	//******************************************
	kinematic_solver ks;
	ks.IKSolver();
		/*
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
   */
  return 0;
}
