//============================================================================
// Name        : forward_kinematic_solver.cpp
// Author      : Shehzad Ahmed
// Version     :
// Copyright   : Your copyright notice
// Description : Forward_kinematic_solver in C++, Ansi-style
//============================================================================

#include <iostream>
#include <unistd.h>
#include <Eigen/Dense>
#include <iomanip>
#include <vector>
using namespace std;
using namespace Eigen;

#include "FKSolver.h"
#include "extApi.h"
#include "vrep/VRepJointInterface.h"

const char* connection_ip = "127.0.0.1";
const int connection_port = 19998;
const char* joint_names[] = {
		"arm_joint_1",
		"arm_joint_2",
		"arm_joint_3",
		"arm_joint_4",
		"arm_joint_5",
		//"gripper_finger_joint_l",
		//"gripper_finger_joint_l"
};

const double upper_joints_limits[5] = {169,90,146,102.5,167.5};
const double lower_joints_limits[5] = {-169,-65,-151,-102.5,-167.5};

int main(int argc, char* argv[]) {
		
		
		FKSolver fk(5,upper_joints_limits,lower_joints_limits);
		
		
		
		
	
  return 0;	
}
