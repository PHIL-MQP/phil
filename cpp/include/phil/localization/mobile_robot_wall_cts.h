// Copyright (C) 2006 Klaas Gadeyne <first dot last at gmail dot com>
//                    Tinne De Laet <first dot last at mech dot kuleuven dot be>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//

#ifndef __MOBILE_ROBOT_CTS_
#define __MOBILE_ROBOT_CTS_

#include <cmath>

// To use measurements specify 1
// To do deadreckoning (no measurements) specify 0
#define USE_MEASUREMENTS 0

#define DELTA_T 1	        // Delta t (for discretisation)
#define NUM_TIME_STEPS 101  // Number of steps that  simulation is running

// Coordinates of wall to which distance is measured
#define RICO_WALL 0.0
#define OFFSET_WALL (0)//100

// Sizes
#define STATE_SIZE 3 //state: x,y,theta
#define INPUT_SIZE 2 //input: v*deltat, omega*deltat
#define MEAS_SIZE 1  //USmeasurment: distance_to_wall

//Initial position and orientation of the mobile robot
#define X_0 0
#define Y_0 0
#define THETA_0  M_PI/4

// Velocity send to the robot, in this case constant for the whole simulation
#define LIN_SPEED  0.1  //translational velocity: v
#define ROT_SPEED 0     //rotational velocity: omega

#define NUM_SAMPLES 1000 // Default Number of Samples
#define RESAMPLE_PERIOD 0 // Default Resample Period
#define RESAMPLE_THRESHOLD (NUM_SAMPLES/4.0) // Threshold for Dynamic Resampling

// Prior:
// Initial estimate of position and orientation
#define PRIOR_MU_X -0.1
#define PRIOR_MU_Y 0.1
#define PRIOR_MU_THETA M_PI/4	//M_PI/4
// Initial covariances of position and orientation
#define PRIOR_COV_X pow(0.2,2)
#define PRIOR_COV_Y pow(0.2,2)
#define PRIOR_COV_THETA pow(M_PI/8,2)

// FOR MIXTURE
// Initial estimate of position and orientation
#define PRIOR_MU_X1 0.1
#define PRIOR_MU_Y1 -0.1
#define PRIOR_MU_THETA1 M_PI/4	//M_PI/4
// Initial covariances of position and orientation
#define PRIOR_COV_X1 pow(0.2,2)
#define PRIOR_COV_Y1 pow(0.2,2)
#define PRIOR_COV_THETA1 pow(M_PI/8,2)
// Initial estimate of position and orientation
#define PRIOR_MU_X2 -0.1
#define PRIOR_MU_Y2 0.1
#define PRIOR_MU_THETA2 M_PI/4	//M_PI/4
// Initial covariances of position and orientation
#define PRIOR_COV_X2 pow(0.2,2)
#define PRIOR_COV_Y2 pow(0.2,2)
#define PRIOR_COV_THETA2 pow(M_PI/8,2)
// Initial estimate of position and orientation
#define PRIOR_MU_X3 0.1
#define PRIOR_MU_Y3 0.1
#define PRIOR_MU_THETA3 M_PI/4	//M_PI/4
// Initial covariances of position and orientation
#define PRIOR_COV_X3 pow(0.2,2)
#define PRIOR_COV_Y3 pow(0.2,2)
#define PRIOR_COV_THETA3 pow(M_PI/8,2)
// Initial estimate of position and orientation
#define PRIOR_MU_X4 -0.1
#define PRIOR_MU_Y4 0.1
#define PRIOR_MU_THETA4 M_PI/4	//M_PI/4
// Initial covariances of position and orientation
#define PRIOR_COV_X4 pow(0.2,2)
#define PRIOR_COV_Y4 pow(0.2,2)
#define PRIOR_COV_THETA4 pow(M_PI/8,2)

// System Noise
#define MU_SYSTEM_NOISE_X 0.0 
#define MU_SYSTEM_NOISE_Y 0.0 
#define MU_SYSTEM_NOISE_THETA 0.0
#define SIGMA_SYSTEM_NOISE_X pow(0.01,2)
#define SIGMA_SYSTEM_NOISE_Y pow(0.01,2)
#define SIGMA_SYSTEM_NOISE_THETA pow(2*M_PI/180,2)


// System Noise for mobile robot simulator
#define MU_SYSTEM_NOISE_X_ROB 0.0 
#define MU_SYSTEM_NOISE_Y_ROB 0.0 
#define MU_SYSTEM_NOISE_THETA_ROB 0.0
#define SIGMA_SYSTEM_NOISE_X_ROB pow(0.001,2)
#define SIGMA_SYSTEM_NOISE_Y_ROB pow(0.001,2)
#define SIGMA_SYSTEM_NOISE_THETA_ROB pow(0.1*M_PI/180,2)

// System Noise for simulation
#define SIM_FACTOR 1000 //The system covariance in simulation is SIM_FACTOR
                        //smaller than the system covariance of the systemmodel

// Measurement noise
#define SIGMA_MEAS_NOISE pow(0.05,2)
#define MU_MEAS_NOISE 0.0

// Measurement noise for mobile robot simulator 
#define SIGMA_MEAS_NOISE_ROB pow(0.05,2)
#define MU_MEAS_NOISE_ROB 0.0

#define NUM_ITERATIONS 3 //number of iterations used for iteraded filters


//Write output of mixture filter to file
#define OUTPUT_MIXTURE true

#endif //__MOBILE_ROBOT_CTS
