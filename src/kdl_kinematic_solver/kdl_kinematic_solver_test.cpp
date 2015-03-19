	//============================================================================
// Name        : RobotManipulation.cpp
// Author      : Matthias Fuller and Shehzad Ahmed
// Version     :
// Copyright   : Your copyright notice
//============================================================================

#include "extApi.h"
#include "vrep/VRepJointInterface.h"

#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotGripper.hpp"
#include "youbot/YouBotManipulator.hpp"

#include <vector>
#include <iostream>

#include <math.h>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jacobian.hpp>
#include <boost/scoped_ptr.hpp>
 #include <kdl/chainjnttojacsolver.hpp>
 #include <Eigen/Geometry>

using namespace std;
using namespace youbot;
using namespace Eigen;

bool enable_real_youbot = false;

const char* connection_ip = "127.0.0.1";
const int connection_port = 19998;
const double upper_joints_limits[5] = { 5.84014, 2.61799, -0.015708, 3.4292, 5.64159 };
const double lower_joints_limits[5] = { 0.0100692, 0.0100692, -5.02655, 0.0221239, 0.110619 };
double pre_grasp[5] = {2.93836, 2.020597, -1.88253, 3.36243, 3.01283};
double candle[5] = {2.9496, 1.13446, -2.54818, 1.78896, 2.93075};

const char* joint_names[] = 
{
		"arm_joint_1",
		"arm_joint_2",
		"arm_joint_3",
		"arm_joint_4",
		"arm_joint_5",
		"gripper_finger_joint_l",
		"gripper_finger_joint_r"
};

double d2r(double v) //degree to radian 
{
	return v / 180 * M_PI;
}

class kinematic_solver
{

public:

	kinematic_solver()
	{
		/* //original chain taken from Shehzad Ahmed mail
		 * double offset[5] = {d2r(-169),d2r(-65),d2r(151),d2r(-102.5),d2r(165)};
		 * chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH(0.0, M_PI, 0.147, 0)));
		 * chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.033,  +M_PI_2,  0.000, offset[0] + M_PI)));
		 * chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.155,  0,    0.000, offset[1] - M_PI_2)));
		 * chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.135,  0,    0.000, offset[2])));
		 * chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.081,  0,       0.000, offset[3])));
		 * chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.000, -M_PI_2,  0.000, offset[4] + M_PI_2)));
		 * chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH(0.00, 0, 0.137, 0)));
		 */
		 
		//Matthias last mail chain
		double offset[5] = {d2r(-169),d2r(-65),d2r(151),d2r(-102.5),d2r(165)};
		T[0] = KDL::Frame::DH(0.0, M_PI, 0.147, 0);
		T[1] = KDL::Frame::DH(0.033,  + M_PI_2,  0.0, offset[0] + M_PI);
		T[2] = KDL::Frame::DH(0.155,  0,    0.000,  offset[1] - M_PI_2);
		T[3] = KDL::Frame::DH(0.135,  0,    0.000,  offset[2]    );
		T[4] = KDL::Frame::DH(0.0,  M_PI+-M_PI_2,       0.0,  offset[3] - M_PI_2   );
		T[5] = KDL::Frame::DH(0.0, 0,  0.0,  offset[4]);
		T[6] = KDL::Frame::DH(0.00, 0, -0.218, 0);
		T[7] = KDL::Frame::DH(0.00, M_PI, 0, 0);
		
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), T[0]));
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), T[1]));
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), T[2]));
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), T[3]));
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), T[4]));
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), T[5]));
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), T[6]));
		chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), T[7]));

		nj = chain.getNrOfJoints();
		js = chain.getNrOfSegments();
		//std::cout << "Joints:" << nj << std::endl;
		//std::cout << "segments:" << js << std::endl;
	}

	//FK solver from base to tool tip
	bool FK_joint_cart (Eigen::VectorXd joint_positions)
	{
		
		KDL::JntArray q(nj);
		
		// Assign some values to the joint positions
		for(unsigned int i = 0;i < nj;i++)
		{
			q(i)=joint_positions[i];
		}
		
		// Calculate forward position kinematics
		KDL::ChainFkSolverPos_recursive fksolver (chain);
		kinematics_status = fksolver.JntToCart(q, cartpos);
		
		if(kinematics_status >= 0)
		{
			//printing pborg -> v
			std::cout << "PBorg : ";
			std::cout << cartpos.p << std::endl;
			
			//printing rotation matrix -> M
			std::cout << "FK:Rotation Matrix : " << std::endl;
			std::cout << cartpos.M << std::endl;
			
			printf("%s \n", "Success Forward kinematics !");
			
			return true;
		}
		else
		{
			printf("%s \n", "Error: could not calculate forward kinematics :(");
			
			return false;
		}

	}
	
	bool FK_joint_cart (double joint_positions[], KDL::Frame &cartpos)
	{
		std::cout << "Forward kinematics solver" << std::endl;
		
		std::cout << "Request received with the following joint value args : ";
		
		for(int i = 0; i < 5; i++)
		{
			std::cout << joint_positions [i] << ", ";
		}
		
		std::cout << std::endl;
		
		KDL::JntArray q(nj);
		
		// Assign some values to the joint positions
		for(unsigned int i = 0;i < nj;i++)
		{
			q(i)=joint_positions[i];
		}
		
		// Calculate forward position kinematics
		KDL::ChainFkSolverPos_recursive fksolver (chain);
		kinematics_status = fksolver.JntToCart(q, cartpos);
		
		if(kinematics_status >= 0)
		{
			//printing pborg -> v
			std::cout << "PBorg : ";
			std::cout << cartpos.p << std::endl;
			
			//printing rotation matrix -> M
			std::cout << "FK:Rotation Matrix : " << std::endl;
			std::cout << cartpos.M << std::endl;
			
			printf("%s \n", "Success Forward kinematics !");
			
			return true;
		}
		else
		{
			printf("%s \n", "Error: could not calculate forward kinematics :(");
			
			return false;
		}

	}
	/*
	//FK solver from base to link
	bool FK_joint_cart (double joint_positions[], KDL::Frame &cartpos, int link)
	{
		cout << "\nForward kinematics solver" << endl;
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
			//std::cout << cartpos <<std::endl;
			
			printf("\n %s \n","Succes Forward kinematics !");
			
			return true;
		}
		else{
			printf("%s \n","Error: could not calculate forward kinematics :(");
			return false;
		}
	}
	*/
	
	//IK solver
	bool IK_cart_joint(KDL::Frame cartpos,double init_positions[], double joint_positions[])
	{
		cout << "\nInverse kinematics solver" << endl;
		KDL::JntArray min_angles(5);
		KDL::JntArray max_angles(5);
		
		for (int i = 0 ; i < 5;i++){
			max_angles(i) = upper_joints_limits[i];
			min_angles(i) = lower_joints_limits[i];
		}
		KDL::ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver
		KDL::ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
		//KDL::ChainIkSolverPos_NR_JL iksolver1(chain,min_angles,max_angles,fksolver1,iksolver1v,500,1e-6); //changed epsilon
		KDL::ChainIkSolverPos_NR_JL iksolver1(chain,min_angles,max_angles,fksolver1,iksolver1v,4000,0.01);
		
		//Creation of jntarrays:
		KDL::JntArray q(nj);
		KDL::JntArray q_init(nj);
		
		for(unsigned int i = 0;i < 5;i++)
		{
			q_init(i)=init_positions[i];
		}
		
		int ret = iksolver1.CartToJnt(q_init,cartpos,q);
		
		std::cout << "Joint values : " << std::endl;
		
		if(ret >= 0)
		{
			//printing ik solution
			std::cout << q.data;
			
			for(unsigned int i = 0;i < nj;i++)
			{
				joint_positions[i] = q(i);
			}
			
			std:: cout << "\nSuccess Inverse kinematics !" << std::endl;

			return true;
			
		}
		else
		{
			printf("%s \n","Error: could not calculate Inverse kinematics :(");
			return false;
		}
	}
	
	void move_arm(double target_joint_angles [5], VRepJointInterface* vRepManipulator, YouBotManipulator* youBotManipulator)
	{

		int index [7] = { 0, 1, 2, 3, 4,5,6};
		
		//printing given target angles
		printf("Moving arm to : [ ");
		
		for (int i = 0; i < 5; i++) 
		{
			if(i != 4)
			{
				printf("%f, ", target_joint_angles [i]);
			}
			else 
			{
				printf("%f ]\n", target_joint_angles [i]);
			}

			//moving real youbot arm
			if(enable_real_youbot == true)
			{
				youBotManipulator->getArmJoint(i+1).setData(target_joint_angles [i] * radian);
			}
		
			//moving simulated vrep youbot arm
			vRepManipulator->setJointPosition(index[i], target_joint_angles [i]);
		}
	}

	void open_gripper(YouBotManipulator* youBotManipulator)
	{
		//open the gripper 2 cm
		GripperBarSpacingSetPoint gripperSetPoint;
		gripperSetPoint.barSpacing = 0.02 * meter;
		youBotManipulator->getArmGripper().setData(gripperSetPoint);
	}

	void close_gripper(YouBotManipulator* youBotManipulator)
	{
		//close the gripper
		GripperBarSpacingSetPoint gripperSetPoint;
		gripperSetPoint.barSpacing = 0 * meter;
		youBotManipulator->getArmGripper().setData(gripperSetPoint);
	}


	void open_gripper_sim(VRepJointInterface* vRepManipulator)
	{
		vRepManipulator->setJointPosition(5, 0.015);
		vRepManipulator->setJointPosition(6, 0.015);
	}

	void close_gripper_sim(VRepJointInterface* vRepManipulator)
	{
		vRepManipulator->setJointPosition(5, 0.0);
		vRepManipulator->setJointPosition(6, 0.0);
	}

	void print_joints(double joint_values [5])
	{
		std::cout << "joint values : ";
		
		for(int i = 0; i < 5; i++)
		{
			std::cout << joint_values [i] << ", ";
		}
		
		std::cout << std::endl;
	}

	void computerJacobian(KDL::JntArray q_)
	{
		KDL::Jacobian  J_;
		
		J_.resize(chain.getNrOfJoints());

		jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(chain));

		jnt_to_jac_solver_->JntToJac(q_, J_);
		
		cout << "Jacobian order at pre_grasp: \n" << J_.rows() << "*" << J_.getColumn(0) << endl; 
	}
	
	KDL::Twist computeTwist(KDL::Jacobian  J_,KDL::JntArrayVel qdot_)
	{
		for (unsigned int i = 0 ; i < 6 ; i++)
		{
		 xdot_(i) = 0;
		 for (unsigned int j = 0 ; j < nj ; j++)
		 xdot_(i) += J_(i,j) * qdot_.qdot(j);
		}
    return xdot_;
	}
	
	Eigen::MatrixXd KDLtoEigen(KDL::Jacobian J_KDL)
	{
		Eigen::MatrixXd J_EIGEN;
		J_EIGEN = MatrixXd::Zero(6,nj);
		
		for (unsigned int i = 0 ; i < 6 ; i++)
		{
		 for (unsigned int j = 0 ; j < nj ; j++)
		   J_EIGEN(i,j) += J_KDL(i,j);
		}
		
		return J_EIGEN;
	}
	
	Eigen::MatrixXd jacobian_inv(Eigen::MatrixXd jacobian)
	{
		Eigen::MatrixXd matrix_temp = jacobian * jacobian.transpose();
		Eigen::MatrixXd jacPseudoInv;
		
		if(matrix_temp.determinant() != 0)
		{
			jacPseudoInv = jacobian.transpose() * ((matrix_temp).inverse());
			//std::cout << "\n===============================" << endl;
			//std::cout << "jacobian:\n " << jacobian << std::endl;
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
    /*
		Eigen::VectorXd initial_pose = moveit_fk(current_joint_state);   //initial Pose;
		
		Eigen::VectorXd goal_pose = moveit_fk(goal_joint_state); //final Pose;
		
		//change final pose 
		 //goal_pose[0] = -0.1; 
		//goal_pose[1] = 0.02;
		goal_pose[2] -= 0.05;
		
		
		std::cout<<"Generating plan to goal pose:\n" << goal_pose.transpose() << std::endl;

		Eigen::VectorXd deltaX = goal_pose - initial_pose;
		
		unsigned int iter = 0;
		
		while(iter < 100)
		{
			//goal_pose[2] -= 0.01; 
			
			 new_joint_angles = current_joint_state + moveit_jacobian_inv(current_joint_state) * (deltaX);
			
			//Eigen::VectorXd newPose = moveit_fk(new_joint_angles);
			double error  = ((Eigen::Matrix<double,6, 6>::Identity() - jacobian * jacPseudoInv) * (deltaX)).norm();
			
			//std::cout<<"\n\nStart arm configuration:"<< current_joint_state.transpose() << std::endl;
			
			//std::cout<<"New arm configuration:"<< new_joint_angles.transpose()<< std::endl;
			
			
			
			if(error <= 0.2)
			{
				//std::cout << "\nerror:" <<  error << std::endl;
				
				//moveit_move_arm(arm_joint_angles);
				
				//initial_pose = goal_pose;
				
				current_joint_state = new_joint_angles;
				
				deltaX = goal_pose - moveit_fk(current_joint_state);
				
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
					ROS_INFO_STREAM("Solution found with intermediate Points : " <<  iter );
					ROS_INFO_STREAM("Moving arm to goal Pose.");
					moveit_move_arm(arm_joint_angles);
				}
				//else
				//	ROS_INFO_STREAM("No solution found Joint Limit Violation.");
				 break;
			}
			
			if(iter == 99)
				 std::cout << "\nNo solution found with error difference:" <<  error << std::endl;
		}
		* */
	}
	KDL::Frame getJointPose(int joint_number)
	{
		return T[joint_number];
	}
  
	KDL::Frame getInverseTransorm(KDL::Frame transform)
	{
		KDL::Frame inverse_transform;
		std::cout << "\nJoint Transformations Inverse:" << endl;
		KDL::Rotation r = transform.M;
		KDL::Vector p = transform.p;
		r.SetInverse();
		inverse_transform.p = -1 * (r * p);
		inverse_transform.M = r;
		std::cout << "Transform:\n" << transform << endl;
		std::cout << "Inverse_transform:\n" << inverse_transform << endl;
		return inverse_transform;
	}
	
	void displayJointTransformations(void)
	{
		KDL::Frame total_transformation;
		std::cout << "\nJoint Transformations:" << endl;
		for(int joint_number = 0; joint_number < 8;joint_number++)
		{
			KDL::Frame frame_pose = getJointPose(joint_number);
			std::cout << frame_pose << endl << endl;
			total_transformation = total_transformation * frame_pose;
		}
		std::cout << total_transformation << endl << endl;
	}
	
	void computeRotationMatrix(KDL::Frame ff)
	{
	//======================================================
	std::cout << "Verifying rotation matrix" << std::endl;
	KDL::Vector pp = ff.p;
	KDL::Rotation r = ff.M;
	//Creating our own cartesian pose to later test on IK
	double cosx = (pp[0])/sqrt((pp[0]*pp[0]) + (pp[1]*pp[1]) + (pp[2]*pp[2]));
	double cosy = (pp[1])/sqrt((pp[0]*pp[0]) + (pp[1]*pp[1]) + (pp[2]*pp[2]));
	double cosz = (pp[1])/sqrt((pp[0]*pp[0]) + (pp[1]*pp[1]) + (pp[2]*pp[2]));
	
	double sinx = sqrt(1 - (cosx*cosx));
	double siny = sqrt(1 - (cosy*cosy));
	double sinz = sqrt(1 - (cosz*cosz));
	
	double alpha = atan2(sinx,cosx);
	double beta = atan2(siny,cosy);
	double gamma = atan2(sinz,cosz);
	
	std::cout << KDL::Rotation::EulerZYX(alpha,beta,gamma) << std::endl;
	r.GetEulerZYX(alpha,beta,gamma);
	std::cout << "alpha,beta,gamma:[" << alpha << "," << beta << "," <<  gamma << "]" << endl;
	/*
	KDL::Frame r_box;
	KDL::Vector pp(-0.0250,0.2750,0.1150);
	std::cout << "Rotation matrix" << std::endl;
	
	r_box.M = KDL::Rotation::EulerZYX(alpha,beta,gamma);
	r_box.p = pp;
	*/
	}
	
public:
	unsigned int nj,js;
	
private:
	KDL::Chain chain;
	 boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
	 KDL::JntArray  q0_;
	// Create the frame that will contain the results
	KDL::Frame cartpos;
	KDL::Twist xdot_;
	bool kinematics_status;
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
int main() 
{
	YouBotManipulator* youBotManipulator;
	VRepJointInterface* vRepManipulator = new VRepJointInterface (connection_ip, connection_port, joint_names);
	kinematic_solver ks;
	
	//preprogramed arm positions
	double candle [5] = { 2.9496, 1.13446, -2.54818, 1.78896, 2.93075 };
	double test [5] = { 2.9496, 1.13446, 2.54818, 0.78896, 0.93075 };
	//double pregrasp [5] = { 4.5, 1.520597, -1.88253, 3.36243, 3.01283 };
	double pregrasp [5] = {4.510000, 1.520597, -1.882530, 3.512430, 3.012830};
	double home_position [5] = {0.0 , 0.0 , 0.0 , 0.0 , 0.0};
	double initial_guess [5];
	double ik_joint_values [5];
	double youbot_current_arm_joint_values [5];

	//connect with real youBot
	if(enable_real_youbot == true)
	{
		youBotManipulator = new YouBotManipulator ("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);

		YouBotBase youBotBase("youbot-base",YOUBOT_CONFIGURATIONS_DIR);
		youBotBase.doJointCommutation();

		youBotManipulator->doJointCommutation();
		youBotManipulator->calibrateManipulator();
		youBotManipulator->calibrateGripper();
	}

	//Displaying joint transformations
	//ks.displayJointTransformations();
	
	//ks.getJointPoseInverse(7);
	
	//testing the arm interface and moving to some positions
	std::cout << "\nTesting arm, sending it to test, candle, (preprogramed positions)...\n" << std::endl;
	ks.open_gripper_sim(vRepManipulator);
	
	ks.move_arm(pregrasp, vRepManipulator, youBotManipulator);
	sleep(3);
	
	//ks.move_arm(candle, vRepManipulator, youBotManipulator);
	//sleep(3);

	
	
	//updating current joint values
	for(int i = 0; i < 5; i++)
	{
		youbot_current_arm_joint_values [i] = pregrasp [i];
	}
	
	//updating initial guess
	for(int i = 0; i < 5; i++)
	{
		initial_guess [i] = youbot_current_arm_joint_values [i];
	}
	
	//kdl part
	KDL::Frame fk_cartpos,inverse_transform;
	
	//ks.computerJacobian(doubleArray2JntArray(initial_guess));
	
	//calling kdl forward kinematics, sending joint values and receiving cartesian pose
	ks.FK_joint_cart(youbot_current_arm_joint_values, fk_cartpos);
	
	//calling kdl inverse kinematics, sending cartesian pose and initial guess, receiving joint values
	//ks.IK_cart_joint(fk_cartpos, initial_guess, ik_joint_values);
	
	
	inverse_transform = fk_cartpos.Inverse();//ks.getInverseTransorm(fk_cartpos);
	
	KDL::Frame Tg_0,Tr_0,Tb_0;
	
	KDL::Vector pcg_org(0.077, 0.2750, 0.1150);
	
	KDL::Vector pcr_org(0.00053, 0.2751, 0.1150);
	
	KDL::Vector pcb_org(-0.0739, 0.2749, 0.1150);
	
	Tr_0.p = pcr_org;
	
	KDL::Frame T6_r = inverse_transform * Tr_0;
	
	std::cout << "Transfrom T6_r:\n" <<  fk_cartpos.M * pcr_org <<  std::endl;
	
	double roll1, pitch1, yaw1,roll2, pitch2, yaw2;
	
	fk_cartpos.M.GetRPY(roll1, pitch1, yaw1);
	
	cout << "roll1:" << roll1 << "  \n";
	cout << "pitch1:" << pitch1 << "  \n";
	cout << "yaw1:" << yaw1 << "  \n";
	
	T6_r.M.GetRPY(roll2, pitch2, yaw2);
	
	cout << "roll2:" << roll2 << "  \n";
	cout << "pitch2:" << pitch2 << "  \n";
	cout << "yaw2:" << yaw2 << "  \n";
	
	fk_cartpos.M = fk_cartpos.M.RPY(roll2, pitch1, yaw1);
	
	
	//======================================================
	//Creating our own cartesian pose to later test on IK
	std::cout << "Creating own cartesian pose" << std::endl;
	
	//1. defining PBorg vector
	//KDL::Vector pb_org(-0.00551938, 0.270483, 0.278866);
	KDL::Vector pb_org;//(0, 0.35, 0.115);-0.0250,+0.2750,+0.1150
	pb_org = fk_cartpos.p;
	pb_org[0] = fk_cartpos.p[0] - T6_r.p[0];
	pb_org[1]= fk_cartpos.p[1] - T6_r.p[1];
	pb_org[2] = fk_cartpos.p[2] - T6_r.p[2];
	//pb_org[1] = pb_org[1] - 0.00; 
	//pb_org[2] = 0.115; 
	
	//pb_org[2] = 0.13;
	//2. defining rotation matrix
	KDL::Rotation rotation_matrix = fk_cartpos.M;
	
	//3. defining kdl frame
	KDL::Frame custom_cartpos;
	
	//4. Assign PBorg vector and rotation matrix to the previously created kdl frame
	custom_cartpos.p = pb_org;
	custom_cartpos.M = rotation_matrix;
	
	std::cout << "Transfrom custom:\n" << custom_cartpos <<  std::endl;
	
	//5. Calling kdl IK with custom frame information, NOTE:the call intself will print joint values
	ks.IK_cart_joint(custom_cartpos, initial_guess, ik_joint_values);
	
	
	//ik_joint_values[0] = ik_joint_values[0] - 0.02;
	
	ks.move_arm (ik_joint_values, vRepManipulator, youBotManipulator);
	sleep(3);
  /*
	ks.move_arm (home_position, vRepManipulator, youBotManipulator);
	sleep(3);
  */
	return 0;
}

/*
 * snippets
 * 
 * identity matrix creation on kdl
 * KDL::Rotation r = KDL::Rotation::Identity();
 * 
 * function to transform from roll, pitch, yaw to rotation matrix form
 * KDL::Rotation R = KDL::Rotation::EulerZYX(0,0,0); 
 * KDL::Rotation r2 = KDL::Rotation::RPY(0,0,0);
 * 
 * 
 * positions
 * .9496, 1.13446, -2.54818, 1.78896, 2.93075 
 * //candle 2.9496,1.2157,-2.72271,-0.735093,-0.735093 
 * //2.94958, 0.01564, -2.59489, 2.38586, 2.93068 
 * // out of view //3.02221, 2.48996, -1.53309, 1.17502, 2.92980 
 * // pre grasping standing //2.93836, 2.020597, -1.88253, 3.36243, 3.01283 
 * // grasp standing 2.93836,2.0206,-1.88253,0.0460373,0.0460373 
 * //2.5061, 0.0935881, -2.60509, 1.42038, 2.93033 
 * // tower_right 2.5061,-25.0109,66.4492,-19.7996,-19.7996 
 * //2.71339, 0.156002, -3.15581, 1.04624, 3.09898 
 * //platform_right
 * 
 * 
 * 	//trying to call ik from roll, pitch, yaw rotation matrix
	//KDL::Rotation rpy_rotation_matrix = KDL::Rotation::RPY(0.5, 0.5, 0.5);
	
	//KDL::Rotation rpy_rotation_matrix;
	
	//rpy_rotation_matrix = rotation_matrix;
	
	double roll, pitch, yaw;
	
	rotation_matrix.GetRPY(roll, pitch, yaw);
	
	cout << "roll" << roll << "  \n";
	cout << "pitch" << pitch << "  \n";
	cout << "yaw" << yaw << "  \n";
	
	std::cout << "--------------------------" << std::endl;
	
	KDL::Rotation rpy_rotation_matrix = KDL::Rotation::RPY(2.3264, -0.744136, 1.00282);
	
	custom_cartpos.M = rpy_rotation_matrix;
	ks.IK_cart_joint(custom_cartpos, initial_guess, ik_joint_values);
	
	
		std::cout << "--------------------------" << std::endl;
	
	for(double j = 0.0 ; j < 3.14 ; j += 0.1)
		for(double i = 0.0 ; i < 3.14 ; i += 0.1)
			for(double k = 0.0 ; k < 3.14 ; k += 0.1)
	{
		rpy_rotation_matrix.RPY(j, i, k);
	
		//calling again ik with roll, pitch yaw modified rotation matrix
		custom_cartpos.M = rpy_rotation_matrix;
		ks.IK_cart_joint(custom_cartpos, initial_guess, ik_joint_values);
	}
 * 
 */
