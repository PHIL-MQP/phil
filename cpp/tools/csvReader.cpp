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

using namespace std;

extern string imu_data_filename;

int main(int argc, char **argv) {
  vector< vector<float> > data_array = getData(imu_data_filename, 164, 178);

  for (int i = 0; i < data_array.size(); i++) {
  	for (int j = 0; j < 6; j++) {
  		cout << data_array.at(i).at(j) << '\t';
  	}
  	cout << endl;
  }

  return 0;
}


vector< vector<float> > getData(string fileName, int startRow, int endRow) {

	vector< vector<float> > res;

	ifstream file(fileName);

	string acc_x, acc_y, acc_z,gyro_x, gyro_y, gyro_z, time;

	if(!file.is_open()) {
		return res;
	}


	int i = 0;
	while(file.good() && i <= endRow) {
		getline(file, acc_x, ',');
		getline(file, acc_y, ',');
		getline(file, acc_z, ',');
		getline(file, gyro_x, ',');
		getline(file, gyro_y, ',');
		getline(file, gyro_z, ',');
		getline(file, time);

		


		if (i >= startRow) {
			vector<float> temp;
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

float myAtof ( string &num){
   float tmp;
   sscanf ( num.c_str(), "%f" , &tmp);
   return tmp;
}
