/*
 * algorithm.cpp
 *
 *  Created on: Nov 16, 2017
 *      Author: nicol
 */
#include <unistd.h>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <vector>


using namespace std;
using namespace Eigen;

vector<double> a_xt;
vector<double> a_yt;
vector<double> a_zt;

int k=0;


double s_init=0;
double threshold = 0;
double s_intervals = 0;
double residuals = 0;
double params_acc = 0;

//void read_data(){
//    m_inf[0][0] = 0;
//}
//

int main(int argc, char** argv) {
    
    
    return 0;
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

    Vector3d b_g;
    Vector3d w_s;
    Vector3d w_s_biasfree;
    MatrixXd m_inf(3, 100);
    
	w_s_biasfree = w_s - b_g;
	s_init = sqrt((variance(a_xt)*variance(a_xt))+(variance(a_yt)*variance(a_yt))+(variance(a_zt)*variance(a_zt)));
	int j = 0;
	for(int i = 0; i < k; i++){
		threshold = i * s_init*s_init;
		s_intervals = calc_static_detector();
		levenberg_marquerd();

		m_inf(0, i) = residuals;
		m_inf(1, i) = params_acc;
		m_inf(2, i) = threshold;
		m_inf(3, i) = s_intervals;


	}
	int lowest = 0;
	for(int i = 0; i < k; i++){
		if(m_inf(0, i) < m_inf(0, i+1)){
			lowest = i;
		}
	}
	int index_opt = lowest;//index of minimum residual in m_inf
	residuals = m_inf(0, index_opt);
	params_acc = m_inf(1, index_opt);

	double a_0 = 0; //calibrate a_s using params_acc????/
	//w_s = levenberg_marquerd();

}




