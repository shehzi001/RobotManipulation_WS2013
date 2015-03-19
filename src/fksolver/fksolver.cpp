//============================================================================
// Name        : fksolver.cpp
// Author      : Shehzad Ahmeds
// Version     :
// Copyright   : Your copyright notice
// Description : Forward_kinematic_solver for robotic arm in C++, Ansi-style
//	Date: 08.11.2013
//============================================================================
#include <iostream>
#include <unistd.h>
#include <Eigen/Dense>
#include <iomanip>
#include <vector>
#include <iterator>
#include "FKSolver.h"

using namespace std;
using namespace Eigen;



FKSolver::FKSolver(const int joints,const double* upper_joints_limits,const double* lower_joints_limits)
{	
	FKSolver::number_of_joints = joints + FKSolver::base_tooltip_frame_count;
	//FKSolver::min_angle_limit.resize(5);
	//FKSolver::max_angle_limit.resize(5);
	//FKSolver::DH_paramters.resize(4);
	for (int i = 0;i < joints;i++)
		{
			std::vector<double> row;
			row.push_back(0.0+i);
			row.push_back(1.0+i);
			row.push_back(2.0+i);
			row.push_back(3.0+i);
			FKSolver::DH_paramters.push_back(row);
		}
	
	for (int iterator = 0;iterator < joints;iterator++)
	{
		FKSolver::min_angle_limit.push_back(lower_joints_limits[iterator] - lower_joints_limits[iterator]);
		FKSolver::max_angle_limit.push_back((upper_joints_limits[iterator] - lower_joints_limits[iterator]) * 3.14 /180);
	}
	for(int row = 0 ; row < joints ; row++)
		std::cout << min_angle_limit[row]<< " " ;
	cout << endl;
	for(int row = 0 ; row < joints ; row++)
		std::cout << max_angle_limit[row]<< " " ;
		
		cout << endl;
	for(int row = 0 ; row < joints ; row++){
	//for(std::vector<double>::size_type i = 0; i != min_angle_limit.size(); i++) 
		for(int col = 0 ; col < 4 ; col++)
		{
			std::cout << DH_paramters[row][col] << " ";
		}
		cout << endl;
	}
	
	
		
}


void FKSolver::setBasetoZeroFrame(const std::vector<double> &DH_base)
{
}

void FKSolver::setWristtoToolFrame(const std::vector<double> &DH_tooltip)
{
}

void FKSolversetDHParameters(const MatrixXf &DH_parameters_joints)
{
}


