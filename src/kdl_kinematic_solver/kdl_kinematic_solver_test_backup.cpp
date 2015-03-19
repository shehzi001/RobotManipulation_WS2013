//============================================================================
// Name        : RobotManipulation.cpp
// Author      : Matthias Fuller and Shehzad Ahmed
// Version     :
// Copyright   : Your copyright notice
//============================================================================

#include <iostream>
using namespace std;

#include "extApi.h"

#include <math.h>
#include "vrep/VRepJointInterface.h"

#include <vector>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/frames.hpp>

#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"
#include "youbot/YouBotGripper.hpp"


using namespace youbot;

const char* connection_ip = "127.0.0.1";
const int connection_port = 19998;
const double upper_joints_limits[5] = {5.84014,2.61799,-0.015708,3.4292,5.64159};
const double lower_joints_limits[5] = {0.0100692,0.0100692,-5.02655,0.0221239,0.110619};

const double upper_cart_limits[3] = {0.2,0.2,0.6};
const double lower_cart_limits[3] = {-0.2,-0.2,0.35};


const char* joint_names[] = {
		"arm_joint_1",
		"arm_joint_2",
		"arm_joint_3",
		"arm_joint_4",
		"arm_joint_5",
		"gripper_finger_joint_l",
		"gripper_finger_joint_l"
};

double d2r(double v) {
	return v / 180 * M_PI;
}

class kinematic_solver{

public:

	kinematic_solver(){
		double offset[5] = {d2r(-169),d2r(-65),d2r(151),d2r(-102.5),d2r(-165)};
		/*
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH(0.0, M_PI, 0.147, 0)));
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.033,  +M_PI_2,  0.000, offset[0] + M_PI)));
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.155,  0,    0.000, offset[1] - M_PI_2)));
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.135,  0,    0.000, offset[2])));
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.081,  0,       0.000, offset[3])));
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.000, -M_PI_2,  0.000, offset[4] + M_PI_2)));
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH(0.00, 0, 0.137, 0)));
		*/
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH(0.0, M_PI, 0.147, 0)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.033,  + M_PI_2,  0.0, offset[0] + M_PI       )));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.155,  0,    0.000,  offset[1] - M_PI_2)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.135,  0,    0.000,  offset[2]    )));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0,  M_PI+-M_PI_2,       0.0,  offset[3] - M_PI_2   )));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0, 0,  0.0,  offset[4])));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH(0.00, 0, -0.218, 0)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH(0.00, M_PI, 0, 0))); 
		nj = chain.getNrOfJoints();
	}
	
	//FK solver from base to tool tip
	bool FK_joint_cart(double joint_positions[],KDL::Frame &cartpos)
	{
		
		KDL::JntArray q(nj);
		// Assign some values to the joint positions
		printf("\nFK_input:[");
		for(unsigned int i = 0;i < 5;i++)
		{
			q(i)=joint_positions[i];
			printf("%0.7lf,",q(i));
		}
		printf("]\n");
		//cout << "\nForward kinematics solver....." << endl;
		cout << "\nFK solver result:" << endl;
		// Calculate forward position kinematics
		KDL::ChainFkSolverPos_recursive fksolver(chain);
		kinematics_status = fksolver.JntToCart(q,cartpos);
		if(kinematics_status >= 0)
		{
			std::cout << cartpos <<std::endl;
			//printf("%s \n","Succes forward kinematics!");
			return true;
		}
		else{
			printf("%s \n kinematic status:,%d","Error: could not calculate forward kinematics :(" ,kinematics_status);
			return false;
		}
		
	}
	//FK solver from base to link
	bool FK_joint_cart(double joint_positions[],KDL::Frame &cartpos,int link)
	{
		cout << "\nForward kinematics solver....." << endl;
		KDL::JntArray q(nj);
		// Assign some values to the joint positions
		for(unsigned int i = 0;i < sizeof(joint_positions);i++)
		{
			q(i)=joint_positions[i];
		}
		cout << "\nFK solver result:" << endl;
		// Calculate forward position kinematics
		KDL::ChainFkSolverPos_recursive fksolver(chain);
		kinematics_status = fksolver.JntToCart(q, cartpos ,link);
		if(kinematics_status >= 0)
		{
			std::cout << cartpos <<std::endl;
			printf("%s \n","Succes Fk!");
			return true;
		}
		else{
			printf("%s \n","Error: could not calculate forward kinematics :(");
			return false;
		}
	}
	
	//IK solver
	bool IK_cart_joint(KDL::Frame cartpos,double init_positions[],double joint_positions[])
	{
		//cout << "\nInverse kinematics solver....." << endl;
		KDL::JntArray min_angles(5);
		KDL::JntArray max_angles(5);
		
		for (int i = 0 ; i < 5;i++){
			max_angles(i) = upper_joints_limits[i];
			min_angles(i) = lower_joints_limits[i];
		}
		KDL::ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver
		KDL::ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
		KDL::ChainIkSolverPos_NR_JL iksolver1(chain,min_angles,max_angles,fksolver1,iksolver1v,100,0.01);
		//KDL::ChainIkSolverPos_NR iksolver1(chain,fksolver1,iksolver1v,100,0.01);
		
		//Creation of jntarrays:
		KDL::JntArray q(nj);
		KDL::JntArray q_init(nj);
		
		for(unsigned int i = 0;i < 5;i++)
		{
			q_init(i)=init_positions[i];
		}
		
		int ret = iksolver1.CartToJnt(q_init,cartpos,q);
		
		if(ret >= 0)
		{
			//std::cout << "Joint positions:\n" << q.data <<std::endl;
			printf("IK:[");
			for(unsigned int i = 0;i < nj;i++)
			{
				joint_positions[i] = q(i);
				printf("%0.7lf,",q(i));
			}
			//printf("%s,\n","Succes Ik!");
			printf("]\n");
			return true;
			
		}
		else
		{
			//printf("%s \n","Error: could not calculate Inverse kinematics :(");
			return false;
		}
	}
	
	
private:
	KDL::Chain chain;
	unsigned int nj;
	// Create the frame that will contain the results
	//KDL::Frame cartpos;  
	bool kinematics_status;
};
//void rand_joint_values(double joint_low_limit,double joint_high_limit)
void rand_joint_values(double joint_positions[])
{
	for(int i=0; i< 5;i++){
	//double v1 = rand() % 6 + 0.0100692; 
	joint_positions[i] = lower_joints_limits[i] + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_joints_limits[i]-lower_joints_limits[i])));
	//printf("rand value:%0.7lf",v1);
	//printf("\n");
	}
	//joint_positions[1] = 1.13446;
	//joint_positions[2] = -2.54818;
}

void rand_cart_pose(double cart_positions[],double cart_orientation[])
{    
	double theta = lower_joints_limits[0] + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_joints_limits[0]-lower_joints_limits[0])));
	double phi =  lower_joints_limits[1] + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(upper_joints_limits[1]-lower_joints_limits[1])));
	double radius_of_workspace = 0.688;
	cart_positions[0] =  radius_of_workspace * abs(cos(theta) * sin (phi));
	cart_positions[1] =  radius_of_workspace * abs(sin(theta) * sin(phi));
	cart_positions[2] =  radius_of_workspace * abs(cos(phi));
	//printf("\n[x,y,z]=[%f,%f,%f]",cart_positions[0],cart_positions[1],cart_positions[2]);
	/*
	for(int i=0; i < 3;i++)
	{
	 cart_orientation[i] = static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(360)));
	}
	*/
}

void compute_rpt(KDL::Vector p)
{    
	float r = sqrt(pow(p(0),2)+pow(p(1),2)+pow(p(2),2));
	float theta = atan2(p(1),p(0));
	float phi  = acos(p(2)/r);
	printf("\nrpt:[r,theta,phi]=[%f,%f,%f]",r,theta,phi);
}

int main() 
{
	//connect with youBot
	/*
	YouBotBase youBotBase("youbot-base",YOUBOT_CONFIGURATIONS_DIR);
	youBotBase.doJointCommutation();
		
	YouBotManipulator youBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);
	
	youBotManipulator.doJointCommutation();
	youBotManipulator.calibrateManipulator();
        youBotManipulator.calibrateGripper();
	
	*/
	// connect with V-rep
	VRepJointInterface* jointInterface = new VRepJointInterface(connection_ip, connection_port, joint_names);
	int index[5] = {0, 1, 2, 3, 4};
	double rand_joint[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
	
	double positions[5] = 
	{
		//2.9496, 1.13446, -2.54818, 1.78896, 2.93075 //candle 2.9496,1.2157,-2.72271,-0.735093,-0.735093
		//2.94958, 0.01564, -2.59489, 2.38586, 2.93068 // out of view
		//3.02221, 2.48996, -1.53309, 1.17502, 2.92980 // pre grasping stangding
		2.93836, 2.020597, -1.88253, 3.36243, 3.01283 // grasp standing 2.93836,2.0206,-1.88253,0.0460373,0.0460373
		//2.5061, 0.0935881, -2.60509, 1.42038, 2.93033 // tower_right 2.5061,-25.0109,66.4492,-19.7996,-19.7996
			
		//2.71339, 0.156002, -3.15581, 1.04624, 3.09898 //platform_right
		//1.5, 0.134162, -2.97261, 0.745996, 2.5
		//2.9496, 1.03446, -2.54818, 1.78896, 2.93075
	};
	
	double final_pos[5] = {2.95584, 1.84354, -1.25588, 2.09657, 2.94754};// 2.96956, 1.84105, -1.30199, 2.1908, 2.88949
	
	double init_positions[5] = 
	{
		//2.9496, 1.13446, -2.54818, 1.78896, 2.93075 //candle 2.9496,1.2157,-2.72271,-0.735093,-0.735093
		//2.94958, 0.01564, -2.59489, 2.38586, 2.93068 // out of view
		//3.02221, 2.48996, -1.53309, 1.17502, 2.92980 // pre grasping stangding
		2.93836, 2.020597, -1.88253, 3.36243, 3.01283 // grasp standing 2.93836,2.0206,-1.88253,0.0460373,0.0460373
		//2.5061, 0.0935881, -2.60509, 1.42038, 2.93033 // tower_right 2.5061,-25.0109,66.4492,-19.7996,-19.7996
		//2.71339, 0.156002, -3.15581, 1.04624, 3.09898 //platform_right
		//1.5, 0.134162, -2.97261, 0.745996, 2.5
		//2.9496, 1.03446, -2.54818, 1.78896, 2.93075
		
		//2.96956, 1.84105, -1.30199, 2.1908, 2.88949
	};
	std::vector<double> min_angles;
	min_angles.resize(5,0);
	std::vector<double> max_angles;
	max_angles.resize(5);

	for (int i = 0 ; i < 5;i++){
		max_angles[i] = upper_joints_limits[i];
		min_angles[i] = lower_joints_limits[i];
	}
	JointAngleSetpoint desiredJointAngle;
	
	//printf("sent position[");
    for (int i = 0; i < 5;i++) 
    {
		//directly control the youBot arm
		//printf("%f,",positions[i]);
		//youBotManipulator.getArmJoint(i+1).setData(positions[i] * radian);

		// and the simulated one
		//jointInterface->setJointPosition(index[i], positions[i]);
    }
	//printf("]");
	// calculate the FK using KDL

	//GripperBarSpacingSetPoint gripperSetPoint;
	//gripperSetPoint.barSpacing = 0.02 * meter;
	//youBotManipulator.getArmGripper().setData(gripperSetPoint);
	kinematic_solver KS;
	KDL::Frame cartpos;
	bool result_ik = false;
	
	for(int i=0; i < 100;i++)
	{
	
	 //rand_joint_values(rand_joint);
	 printf("\n===============================================");
	 printf("\nsending random joint positions to youbot\n");
	 for (int j = 0; j < 5;j++) 
		{
		//directly control the youBot arm
			
			//youBotManipulator.getArmJoint(j+1).setData(rand_joint[j] * radian);

		// and the simulated one
		  jointInterface->setJointPosition(index[j], positions[j]);
		}
	 
	 KS.FK_joint_cart(positions,cartpos);
	 sleep(1);
	//calculate the IK using KDL to verify the FK results
	
	cartpos.p[0] += 0.0;
	cartpos.p[1] += 0.0;
	cartpos.p[2] += 0.00;
	int try_ik = 0;
	//compute_rpt(cartpos.p);
	do
	{
		result_ik = KS.IK_cart_joint(cartpos,init_positions,rand_joint);
		//if(!result_ik)
		//rand_joint_values(init_positions);
		try_ik++;
	}while(try_ik < 1 && !(result_ik));


	if(result_ik)
	  {
		  printf("sending ik result(joint poistions) to youbot");
			for (int j = 0; j < 5;j++) 
				{
				//directly control the youBot arm
				//printf("%f,",positions[i]);
				 //youBotManipulator.getArmJoint(j+1).setData(rand_joint[j] * radian);

				// and the simulated one
					jointInterface->setJointPosition(index[j], rand_joint[j]); 
				}
				
		 sleep(1);
     }
    else
			printf("\n%s \n","Error: could not calculate Inverse kinematics :(");
     //KS.FK_joint_cart(rand_joint,cartpos);
	 sleep(1);
     printf("\n===============================================");
	//printf("rand value:%0.7lf",rand_joint[i]);
	printf("\n");
    } 
    sleep(2);
    return 0;
}

/*
	KDL::Vector p(-0.392,  0.00244385,   0.0);
	KDL::Rotation r(0.779067,  0.00623352,    0.626909,-0.00485643,    0.999981, -0.00390793,-0.626921,-1.35245e-17,0.779083);
	//KDL::Rotation r = KDL::Rotation::Identity();
	KDL::Frame cart_pos;
	cart_pos.p = p;
	cart_pos.M = r;
	//calculate the FK again using KDL to check the IK results
	//KS.FK_joint_cart(positions,cartpos);
	KS.IK_cart_joint(cart_pos,init_positions,positions);
	
	sleep(10);
	printf("position from Ik[");
	for (int i = 0; i < 5;i++) 
	{
		//directly control the youBot arm
		printf("%f,",positions[i]);
		//youBotManipulator.getArmJoint(i+1).setData(positions[i] * radian);
		// and the simulated one
		jointInterface->setJointPosition(index[i], positions[i]);
	}
	printf("]\n");
	sleep(5);
	// open the gripper 2 cm
	//GripperBarSpacingSetPoint gripperSetPoint;
	//gripperSetPoint.barSpacing = 0.02 * meter;
	//youBotManipulator.getArmGripper().setData(gripperSetPoint);
	*/
