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

    Vector3f b_g;
    Vector3f w_s;
    Vector3f w_s_biasfree;
    MatrixXf m_inf(3, 100);
    
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

Matrix4f rk4_method(int t_k, int dt, Matrix4f q_k) {
    Matrix4f k1, k2, k3, k4;
    
    k1 = 0.5 * get_angular_velocity(t_k) * q_k;
    
    
    
    return k1;
}


Matrix4f get_angular_velocity(int t) {
    float w_x, w_y, w_z;
    //need to implement the algorithm to read data from the csv
    
    return angular_velocity_2_quaternion(w_x, w_y, w_z);
}


Matrix4f angular_velocity_2_quaternion(float w_x, float w_y, float w_z) {
    Matrix4fn quaternion;
    //need to implement ewwww
    return quaternion;
}



