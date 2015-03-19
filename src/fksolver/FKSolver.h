/*
 * fksolver.h
 *
 *  Created on: Nov 08, 2013
 *      Author: Shehzad Ahmed
 */

#ifndef FKSOLVER_H_
#define FKSOLVER_H_

class FKSolver
{
public:
	 FKSolver(int joints,const double* upper_joints_limits,const double* lower_joints_limits);
	
	void setBasetoZeroFrame(const std::vector<double>&);
	void setWristtoToolFrame(const std::vector<double>&);
	void setDHParameters(const Eigen::MatrixXf &DH_parameters_Sstable);
	Eigen::Matrix4f getFKtransformation(const std::vector<double>& desired_joint_angles);

private:
	int number_of_joints;
	const int base_tooltip_frame_count = 2; 
	std::vector<double> min_angle_limit;
	std::vector<double> max_angle_limit;
	std::vector<std::vector<double> > DH_paramters; 
	
};
#endif
