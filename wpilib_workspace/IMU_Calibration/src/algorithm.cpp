/*
 * algorithm.cpp
 *
 *  Created on: Nov 16, 2017
 *      Author: nicol
 */
#include <unistd.h>
#include <exception>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <math.h>
#include <SPI.h>
#include <HAL/HAL.h>

#include <IMU_Calibration.h>
std::vector<double> a_xt;
std::vector<double> a_yt;
std::vector<double> a_zt;

int k=0;
double b_g = 0;
double w_s = 0;
double w_s_bias = 0;
double m_intf[100][3];
double s_init=0;
double threshold = 0;
double s_intervals = 0;
double residuals = 0;
double params_acc = 0;
void read_data(){
	m_intf[0][0] = 0;
}
double mean(std::vector<double> data){
	double sum = 0.0;
	for(double a=0; a<data.size(); a++){
		sum = sum + a;
	}

	return sum/data.size();
}
double variance(std::vector<double> data){

	double m = mean(data);
	double temp = 0;
	for(double a = 0;a<data.size();a++){
		temp += (a-m)*(a-m);
	}
	return temp/(data.size()-1);
}

double calc_static_detector(){

	return 0;
}
void levenberg_marquerd(){
////update residuals and params_acc
	residuals = 0;

}
void calibrate_accel(){

}
void calibrate_gyro(){

}
void IMU_calibration(){


	w_s_bias = w_s*b_g;
	s_init = sqrt((variance(a_xt)*variance(a_xt))+(variance(a_yt)*variance(a_yt))+(variance(a_zt)*variance(a_zt)));
	int j = 0;
	for(int i = 0; i < k; i++){
		threshold = i * s_init*s_init;
		s_intervals = calc_static_detector();
		levenberg_marquerd();

		m_intf[i][0] = residuals;
		m_intf[i][1] = params_acc;
		m_intf[i][2] = threshold;
		m_intf[i][3] = s_intervals;


	}
	int lowest = 0;
	for(int i = 0; i < k; i++){
		if(m_intf[i][0] < m_intf[i+1][0]){
			lowest = i;
		}
	}
	int index_opt = lowest;//index of minimum residual in m_inf
	residuals = m_intf[index_opt][0];
	params_acc = m_intf[index_opt][1];

	double a_0 = 0; //calibrate a_s using params_acc????/
	w_s = levenberg_marquerd();

}




