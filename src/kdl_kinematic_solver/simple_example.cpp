// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
 #include <math.h>
 
 
using namespace KDL;
using namespace std;
 double d2r(double v) {
	return v / 180 * M_PI;
}

int main( int argc, char** argv )
{  
    KDL::Chain chain;
    double offset[5] = {d2r(-169),d2r(-65),d2r(151),d2r(-102.5),d2r(-165)};
    /*chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH(0.0, 0,0, 0)));
    chain.addSegment(Segment(Joint(Joint::RotZ),KDL::Frame::DH(0.5,0.0,0.0,M_PI_2)));
    chain.addSegment(Segment(Joint(KDL::Joint::None),KDL::Frame::DH(0.0,M_PI_2,0.4,0)));
    */
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH(0.0, M_PI, 0.147, 0)));

    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.033,  + M_PI_2,  0.0, offset[0] + M_PI       )));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.155,  0,    0.000,  offset[1] - M_PI_2)));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.135,  0,    0.000,  offset[2]    )));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0,  M_PI+-M_PI_2,       0.0,  offset[3] - M_PI_2   )));
    
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame::DH(0.0, 0,  0.0,  offset[4])));
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH(0.00, M_PI, -0.218, 0)));
    
    
    //chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), KDL::Frame::DH(0.00, M_PI, 0, 0))); 
 

		double positions[5] = 
		{
	   3.02221, 2.48996, -1.53309, 1.17502, 2.92980 // 2.9496, 1.13446, -2.54818, 1.78896, 2.93075 //candle 
		};
		
		double final_pos[5] = {2.95584, 1.84354, -1.25588, 2.09657, 2.94754};// 2.96956, 1.84105, -1.30199, 2.1908, 2.88949
		
		double init_positions[5] = 
		{
			2.9496, 1.13446, -2.54818, 1.78896, 2.93075
		};
    // Create solver based on kinematic chain
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
 
    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = JntArray(nj);
 
    // Assign some values to the joint positions
    for(unsigned int i = 0;i < nj;i++)
    {
        //float myinput;
        //printf ("Enter the position of joint %i: ",i);
        //scanf ("%e",&myinput);
        jointpositions(i)=positions[i];//(double)myinput;
    }
 
    // Create the frame that will contain the results
    KDL::Frame cartpos;
 
    // Calculate forward position kinematics
    bool kinematics_status;
    for(int loop=1;loop < 8;loop++){
    kinematics_status = fksolver.JntToCart(jointpositions,cartpos,loop);
    if(kinematics_status>=0){
        std::cout << cartpos <<std::endl;
        //printf("%s \n","Succes, thanks KDL!");
    }else{
        printf("%s \n","Error: could not calculate forward kinematics :(");
    }
	 }
   
}
