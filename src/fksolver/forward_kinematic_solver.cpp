//============================================================================
// Name        : forward_kinematic_solver.cpp
// Author      : Shehzad and Iuri
// Version     :
// Copyright   : Your copyright notice
// Description : Forward_kinematic_solver in C++, Ansi-style
//============================================================================

/*=====================Angle ranges for youbou arm joints================================
A1 : +/-169 -> 0-338 -> 0.0-5.8962
A2 : -65 - +90 -> 0-155 -> 0.0-2.7038
A3 : -151 - +146 -> 0-294 -> -5.1286 - 0.0
A4 : +/-102.5 -> 0-205 -> 0.0-3.5761
A5 : +/-167.5 -> 0-335 -> 0.0-5.84
*/


#include <iostream>
#include <unistd.h>
#include <Eigen/Dense>
#include <iomanip>
using namespace std;
using namespace Eigen;

#include "extApi.h"
#include "vrep/VRepJointInterface.h"


#define d0 0.115 	//0.0469	//0.115  //147-0      offset distance from base frame to frame 0 
#define L1 0.033 	//33-0 					distance between joint 1 and joint 2 rotation axis
#define L2 0.155	//0.155	//302-147(0.155)  0.302-0.0469    distance between joint 2 and joint 3 rotation axis
#define L3 0.135	//0.135 	//437-302						distance between joint 3 and joint 4 rotation axis
#define d4 0.095	//0.218	//655-437							offset distance between joint 4 and joint tooltip frame

#define PI 3.141592654
#define alpha0 	0      
#define alpha1 	PI/2		//twist angle between Z1-axis and Z2-axis 	
#define alpha4 -PI/2		//twist angle between Z4-axis  and Z5-axis 	
 
 
void set_DH();
void setBasetoZeroFrame();
void setWristtoToolFrame();
void calculateforwardkinematics(Matrix<float,5,1> jointAngles);
Matrix4f calculateTransformationforjoints(RowVector4f row); 

const char* connection_ip = "127.0.0.1";
const int connection_port = 19998;

MatrixXf DH(5,4);	// Matrix for DH parameters only joints
RowVector4d v1,v2;	// Vectors for DH parameters of base frame and tool tip frame
Matrix4f TB_0,T0_1,T1_2,T2_3,T3_4,T4_5,T5_T,TB_T;   //Matrix to contain transformations from one frame to other

const char* joint_names[] = {
		"arm_joint_1",
		"arm_joint_2",
		"arm_joint_3",
		"arm_joint_4",
		"arm_joint_5",
		"gripper_finger_joint_l",
		"gripper_finger_joint_l"
};



int main(int argc, char* argv[]) {
	
	int index[5] = {0,1,2,3,4};
	double positions[5]; // = {2.9, 1.1, -2.5, 1.7, 1.5};
	
	Matrix<float,5,1> jointAngles;
	VRepJointInterface* jointInterface = new VRepJointInterface(connection_ip, connection_port, joint_names);
	
	if(argc == 6){
		for(int t = 1 ; t < argc ; t++){
			  //cout << atof(argv[t]) << " " ; 
			  jointAngles(t-1) = atof(argv[t]);
			  positions[t-1] = atof(argv[t]);
			}
		cout << endl;	
		}
	else {
		cout << "Exact Number of paramerters(angles) are not provided.Initializing with default parameters"<<endl;
			jointAngles << 0.0, 1.1, -2.5, 1.7, 1.5;	
			//jointAngles << 0.0, 0.0, 0.0, 0.0, 0.0;	
			for(int t = 0 ; t < 5 ; t++){
				positions[t] = jointAngles(t);
			}
			
			cout << jointAngles << endl;
		}
	
	set_DH();
	setBasetoZeroFrame();
	setWristtoToolFrame();
	calculateforwardkinematics(jointAngles);
	
	jointInterface->setJointVelocity(0,0.1);
	sleep(2);
	//setting a group of joints
	jointInterface->setJointPosition(index, positions, 5);
	
	sleep(2);

	return 0;
}

//Calculating forward kinematic transformation

void calculateforwardkinematics(Matrix<float,5,1> jointAngles){
	
	DH.col(3) = jointAngles;
	T0_1 = calculateTransformationforjoints(DH.row(0));
	T1_2 = calculateTransformationforjoints(DH.row(1));
	T2_3 = calculateTransformationforjoints(DH.row(2));
	T3_4 = calculateTransformationforjoints(DH.row(3));
	T4_5 = calculateTransformationforjoints(DH.row(4));
	
	//cout << "T0_1:\n" << T0_1 << endl;
	//cout << "T1_2:\n" << T1_2 << endl;
	//cout << "T2_3:\n" << T2_3 << endl;
	//cout << "T3_4:\n" << T3_4 << endl;
	//cout << "T4_5:\n" << T4_5 << endl;
	
	//Composite transformation from Base frame to tool tip
	TB_T = TB_0 * T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_T;
	
	//Using equations  from Book Introduction to RObotics Mechanics and Control,Ch. Manipulator Kinematics,Ch.2
	float beta = atan2(-TB_T(2,0) , sqrt(pow((TB_T(0,0)) , 2) + pow(TB_T(1,0) , 2))) * 180 / PI;
	float alpha = atan2((TB_T(1,0)/cos(beta)),(TB_T(0,0)/cos(beta)))* 180 / PI;
	float gamma = atan2((TB_T(2,1)/cos(beta)),(TB_T(2,2)/cos(beta)))* 180 / PI;
	
	cout << "Forward kinematic transformation(TB_T) :\n" << TB_T << endl;
	cout << "alpha:" << alpha << endl;
	cout << "beta:" << beta << endl;
	cout << "gamma:" << gamma  << endl;
}

//Setting DH parameters
void set_DH(){
	DH = MatrixXf::Zero(5, 4);
	v1 << 0,0,d0,0;
	v2 << 0,0,d4,0;
	DH.col(0) << 0, L1 , L2 , L3 , 0;
	DH.col(1) << alpha0, alpha1 , 0 , 0 , alpha4;
}

Matrix4f calculateTransformationforjoints(RowVector4f row)
{	// here a = link length, d = link offset , alpha = link twist and  theta is joint angle
	float a = row(0);
	float c_alpha = cos(row(1));	  //1st location corresponds to alpha
	float s_alpha = sin(row(1));
	float d = row(2);
	float c_theta = cos(row(3));   //3rd location corresponds to theta 
	float s_theta = sin(row(3));
	Matrix4f T_temporary;
	//Using link transformation formula from Book Introduction to RObotics Mechanics and Control,Ch. Manipulator Kinematics,Eq.3.6 
	T_temporary.row(0) <<  c_theta , -1 * s_theta , 0.0 , a ;
	T_temporary.row(1) <<  (s_theta * c_alpha) , (c_theta * c_alpha) ,-1*s_alpha , (-1 * s_alpha * d) ;
	T_temporary.row(2) <<  (s_theta * s_alpha) , (c_theta * s_alpha) , c_alpha , (c_alpha * d) ;
	T_temporary.row(3) <<  0.0 , 0.0, 0.0 , 1.0 ;
	
	return T_temporary;

}



//Initializing DH parameters for base
void setBasetoZeroFrame()
{	// here a = link length, d = link offset , alpha = link twist and  theta is joint angle
	float a = v1(0);
	float c_alpha = cos(v1(1));	  //1st location corresponds to alpha
	float s_alpha = sin(v1(1));
	float d = v1(2);
	float c_theta = cos(v1(3));   //3rd location corresponds to theta 
	float s_theta = sin(v1(3));
	
	//Using link transformation formula from Book Introduction to RObotics Mechanics and Control,Ch. Manipulator Kinematics,Eq.3.6 
	TB_0.row(0) <<  c_theta , -s_theta , 0.0 , a ;
	TB_0.row(1) <<  (s_theta * c_alpha) , (c_theta * c_alpha) ,-s_alpha , (-s_alpha * d) ;
	TB_0.row(2) <<  (s_theta * s_alpha) , (c_theta * s_alpha) , c_alpha , (c_alpha * d) ;
	TB_0.row(3) <<  0.0 , 0.0, 0.0 , 1.0 ;
	//cout << "TB_0:\n" << TB_0 << endl;
}

//Initializing DH parameters for last joint to tool tip
void setWristtoToolFrame()
{	
	// here a = link length, d = link offset , alpha = link twist and  theta is joint angle
	float a = v2(0);
	float c_alpha = cos(v2(1));	  //1st location corresponds to alpha
	float s_alpha = sin(v2(1));
	float d = v2(2);
	float c_theta = cos(v2(3));   //3rd location corresponds to theta 
	float s_theta = sin(v2(3));
	
	//Using link transformation formula(Eq.3.6) from Book Introduction to RObotics Mechanics and Control,Ch. Manipulator Kinematics
	T5_T.row(0) <<  c_theta , -1 * s_theta , 0.0 , a ;
	T5_T.row(1) <<  (s_theta * c_alpha) , (c_theta * c_alpha) ,-1*s_alpha , (-1 * s_alpha * d) ;
	T5_T.row(2) <<  (s_theta * s_alpha) , (c_theta * s_alpha) , c_alpha , (c_alpha * d) ;
	T5_T.row(3) <<  0.0 , 0.0, 0.0 , 1.0 ;
	//cout << "T5_T:\n" << T5_T << endl;
 
}




