/*
 * csvReader.cpp
 *
 *  Created on: Nov 16, 2017
 *      Author: jhu4
 */
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "csvReader.h"



std::vector<float> getData(std::string file_name, int row_number) {
	std::vector<float> res;
	std::ifstream file(file_name);

	std::string acc_x, acc_y, acc_z,gyro_x, gyro_y, gyro_z, time;

	if(!file.is_open()) {
		return res;
	}
	
	int i = 1;
	while(file.good() && i < row_number) {
		getline(file, time);
		i++;
	}

	getline(file, acc_x, ',');
	getline(file, acc_y, ',');
	getline(file, acc_z, ',');
	getline(file, gyro_x, ',');
	getline(file, gyro_y, ',');
	getline(file, gyro_z, ',');

	res.push_back(myAtof(acc_x));
	res.push_back(myAtof(acc_y));
	res.push_back(myAtof(acc_z));
	res.push_back(myAtof(gyro_x));
	res.push_back(myAtof(gyro_y));
	res.push_back(myAtof(gyro_z));

	file.close();

	return res;
}

std::vector< std::vector<float> > getData(std::string file_name, int start_row, int end_row) {

	std::vector< std::vector<float> > res;

	std::ifstream file(file_name);

	std::string acc_x, acc_y, acc_z,gyro_x, gyro_y, gyro_z, time;

	if(!file.is_open()) {
		return res;
	}


	int i = 1;
	while(file.good() && i <= end_row) {
		getline(file, acc_x, ',');
		getline(file, acc_y, ',');
		getline(file, acc_z, ',');
		getline(file, gyro_x, ',');
		getline(file, gyro_y, ',');
		getline(file, gyro_z, ',');
		getline(file, time);

		


		if (i >= start_row) {
			std::vector<float> temp;
			temp.push_back(myAtof(acc_x));
			temp.push_back(myAtof(acc_y));
			temp.push_back(myAtof(acc_z));
			temp.push_back(myAtof(gyro_x));
			temp.push_back(myAtof(gyro_y));
			temp.push_back(myAtof(gyro_z));
			res.push_back(temp);
		}

		i++;
	}
	file.close();

	return res;
}

float myAtof ( std::string &num){
   float tmp = 0;
   sscanf ( num.c_str(), "%f" , &tmp);
   return tmp;
}
