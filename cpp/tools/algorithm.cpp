/*
 * algorithm.cpp
 *
 *  Created on: Nov 16, 2017
 *      Author: nicol
 */
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <fstream>
#include <eigen3/unsupported/Eigen/NonLinearOptimization>

#include "csvReader.h"

#define PI 3.14159265

//definations, will go to header file later
Eigen::Matrix4f get_angular_velocity(int t);

Eigen::Matrix4f angular_velocity_2_quaternion(float w_x, float w_y, float w_z);

Eigen::Vector3f integrate_angular_velocity(std::string filename, int start, int end, float dt);

Eigen::Vector3f calculate_expect_acc(std::string filename, int start, int end, float dt, Eigen::Vector3f initial_acc);

Eigen::Vector3f calculate_expect_acc_gradually(std::string filename, int start, int end, float dt, Eigen::Vector3f initial_acc);

Eigen::Vector3f get_acc_eigen_vector(std::vector<float> input);

Eigen::Vector3f get_gyro_eigen_vector(std::vector<float> input);

Eigen::Matrix3f make_rotation_matrix(float theta_x, float theta_y, float theta_z);

float degree_to_radian(float degree);

void log_gyroscope_data(std::string filename);

std::vector<double> a_xt;
std::vector<double> a_yt;
std::vector<double> a_zt;

int k = 0;

double s_init = 0;
double threshold = 0;
double s_intervals = 0;
double residuals = 0;
double params_acc = 0;

int main(int argc, char **argv) {
  // Process the initial Tinit data

  log_gyroscope_data(argv[1]);

  return 0;
}

void log_gyroscope_data(std::string filename) {

  int static_intevals_midpoints[33] = {4727, 5275, 5883, 6521, 7017, 7393, 8626, 9590, 10309, 10924, 11616, 12368, 13240, 13994, 14726, 15442, 16277, 16880, 17551, 18237, 19448, 20487, 21177, 22077, 22631, 23244, 23997, 24569, 25174, 25783, 26394, 26967, 27578};
  
  std::ofstream log;
  log.open("../../recorded_sensor_data/gyroscope_data/gyroscope_data2.csv", std::ofstream::trunc);
  if (!log.good()) {
    std::cout << "not good" << std::endl;
    return;
  }

  log << "initial_acc_x,initial_acc_y,initial_acc_z,expected_acc_x,expected_acc_y, expected_acc_z,"
   << "expected_acc_gradually_x, expected_acc_gradually_y, expected_acc_gradually_z, final_acc_x, final_acc_y, final_acc_z" << std::endl;

  int window_size = 101;

  Eigen::Vector3f initial_acc;
  Eigen::Vector3f expected_acc;
  Eigen::Vector3f real_acc; 
  Eigen::Vector3f expected_acc_gradually;

  for(int i = 1; i < 33; i++) {
    int start = static_intevals_midpoints[i - 1];
    int end = static_intevals_midpoints[i];

    real_acc = get_acc_eigen_vector(getData(filename, end)) / window_size;
    initial_acc = get_acc_eigen_vector(getData(filename, start)) / window_size;
    for (int j = 1; j <= (window_size / 2); j++) {
      real_acc += (get_acc_eigen_vector(getData(filename, end - j)) / window_size);
      real_acc += (get_acc_eigen_vector(getData(filename, end + j)) / window_size);
      initial_acc += (get_acc_eigen_vector(getData(filename, start - j)) / window_size);
      initial_acc += (get_acc_eigen_vector(getData(filename, start + j)) / window_size);
    }
    expected_acc = calculate_expect_acc(filename, start, end, 0.00399, initial_acc);
    expected_acc_gradually = calculate_expect_acc_gradually(filename, start, end, 0.00399, initial_acc);
    

    std::string comma = ",";

    std::cout << start << std::endl;

    log << initial_acc(0) << comma << initial_acc(1) << comma << initial_acc(2) << comma;
    log << expected_acc(0) << comma << expected_acc(1) << comma << expected_acc(2) << comma;
    log << expected_acc_gradually(0) << comma << expected_acc_gradually(1) << comma << expected_acc_gradually(2) << comma;
    log << real_acc(0) << comma << real_acc(1) << comma << real_acc(2) << comma << std::endl;
    
  }

  log.close();

}



double mean(std::vector<double> data) {
  double sum = 0.0;
  for (double a = 0; a < data.size(); a++) {
    sum = sum + a;
  }

  return sum / data.size();
}

double variance(std::vector<double> data) {

  double m = mean(data);
  double temp = 0;
  for (double a = 0; a < data.size(); a++) {
    temp += (a - m) * (a - m);
  }
  return temp / (data.size() - 1);
}

double calc_static_detector() {

  return 0;
}

void levenberg_marquerd() {
////update residuals and params_acc
  residuals = 0;

}

void calibrate_accel() {

}

void calibrate_gyro() {

}

void IMU_calibration() {

  Eigen::Vector3f b_g;
  Eigen::Vector3f w_s;
  Eigen::Vector3f w_s_biasfree;
  Eigen::MatrixXf m_inf(3, 100);

  w_s_biasfree = w_s - b_g;
  s_init =
      sqrt((variance(a_xt) * variance(a_xt)) + (variance(a_yt) * variance(a_yt)) + (variance(a_zt) * variance(a_zt)));
  int j = 0;
  for (int i = 0; i < k; i++) {
    threshold = i * s_init * s_init;
    s_intervals = calc_static_detector();
    levenberg_marquerd();

    m_inf(0, i) = residuals;
    m_inf(1, i) = params_acc;
    m_inf(2, i) = threshold;
    m_inf(3, i) = s_intervals;

  }
  int lowest = 0;
  for (int i = 0; i < k; i++) {
    if (m_inf(0, i) < m_inf(0, i + 1)) {
      lowest = i;
    }
  }
  int index_opt = lowest;//index of minimum residual in m_inf
  residuals = m_inf(0, index_opt);
  params_acc = m_inf(1, index_opt);

  double a_0 = 0; //calibrate a_s using params_acc????/
  //w_s = levenberg_marquerd();
}

Eigen::Matrix4f rk4_method(int t_k, int dt, Eigen::Matrix4f q_k) {
  Eigen::Matrix4f k1, k2, k3, k4;

  k1 = 0.5 * get_angular_velocity(t_k) * q_k;

  return k1;
}

Eigen::Matrix4f get_angular_velocity(int t) {
  float w_x, w_y, w_z;
  //need to implement the algorithm to read data from the csv

  return angular_velocity_2_quaternion(w_x, w_y, w_z);
}

Eigen::Vector3f calculate_expect_acc(std::string filename, int start, int end, float dt, Eigen::Vector3f initial_acc) {

  Eigen::Vector3f angles = integrate_angular_velocity(filename, start, end, dt);


  Eigen::Matrix3f rotation_matrix;
  rotation_matrix = Eigen::AngleAxisf(angles(0), Eigen::Vector3f::UnitX())
      * Eigen::AngleAxisf(angles(1), Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(angles(2), Eigen::Vector3f::UnitZ());

  // std::cout << "compare" << std::endl;
  // std::cout << make_rotation_matrix(angles(0), angles(1), angles(2)) << std::endl;
  // std::cout << rotation_matrix << std:: endl;

  return (rotation_matrix * initial_acc.normalized());
}

Eigen::Vector3f calculate_expect_acc_gradually(std::string filename, int start, int end, float dt, Eigen::Vector3f initial_acc) {
  std::vector<std::vector<float> > data = getData(filename, start, end);


  Eigen::Matrix3f rotation_matrix;

  for (int i = 0; i < data.size(); i++) {
    rotation_matrix = Eigen::AngleAxisf(degree_to_radian(data[i][3] * dt), Eigen::Vector3f::UnitX())
      * Eigen::AngleAxisf(degree_to_radian(data[i][4] * dt), Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(degree_to_radian(data[i][5] * dt), Eigen::Vector3f::UnitZ());

    initial_acc = rotation_matrix * initial_acc;
  }

  return initial_acc;
}



Eigen::Matrix3f make_rotation_matrix(float theta_x, float theta_y, float theta_z) {
	Eigen::Matrix3f rotation_matix_x, rotation_matix_y, rotation_matix_z;
	rotation_matix_x << 1, 0, 0,
											0, cos(theta_x), -1 * sin(theta_x),
											0, sin(theta_x), cos(theta_x);
	rotation_matix_y << cos(theta_y), 0, sin(theta_y),
											0, 1, 0,
											-1 * sin(theta_y), 0, cos(theta_y);
	rotation_matix_z << cos(theta_z), -1 * sin(theta_z), 0,
											sin(theta_z), cos(theta_z), 0,
											0, 0, 1;
	return rotation_matix_x * rotation_matix_y * rotation_matix_z;
}


Eigen::Vector3f get_acc_eigen_vector(std::vector<float> input) {
	return Eigen::Vector3f(input[0], input[1], input[2]);
}

Eigen::Vector3f get_gyro_eigen_vector(std::vector<float> input) {
	return Eigen::Vector3f(input[3], input[4], input[5]);
}




/**
	@int start - the start index of the csv file row number
	@int end - the end index of the
	@int dt - time in unit of second
	@Eigen::Vector3f
**/
Eigen::Vector3f integrate_angular_velocity(std::string filename, int start, int end, float dt) {
  std::vector<std::vector<float> > data = getData(filename, start, end);
  Eigen::Vector3f res(0, 0, 0);

  for (int i = 0; i < data.size(); i++) {
    res[0] += data[i][3] * dt;
    res[1] += data[i][4] * dt;
    res[2] += data[i][5] * dt;
  }

  res[0] = degree_to_radian(res[0]);
  res[1] = degree_to_radian(res[1]);
  res[2] = degree_to_radian(res[2]);

  return res;
}

float degree_to_radian(float degree) {
  return degree / 180 * PI;
}

Eigen::Matrix4f angular_velocity_2_quaternion(float w_x, float w_y, float w_z) {
  Eigen::Matrix4f quaternion;
  //need to implement ewwww
  return quaternion;
}



