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



int main(int argc, char **argv) {
  vector< vector<float> > data_array = getData("imu_calibration_data_11_14.csv", 1, 3);

  cout << data_array.size() << endl;

  for (int i = 0; i < 3; i ++) {
    for (int j = 0; j < 6; j++) {
      cout << data_array[i][j] << endl;
    }
  }

  return 0;
}


vector< vector<float> > getData(string fileName, int startRow, int endRow) {

	vector< vector<float> > res(endRow - startRow + 1);

	ifstream file(fileName);

	string acc_x, acc_y, acc_z,gyro_x, gyro_y, gyro_z, time;
//	float acc_y;
//	float acc_z;
//	float gyro_x;
//	float gyro_y;
//	float gyro_z;

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
		getline(file, time, '\n');

		i++;


		if (i >= startRow) {
			vector<float> temp(6);
			temp.push_back(myAtof(acc_x));
			temp.push_back(myAtof(acc_y));
			temp.push_back(myAtof(acc_z));
			temp.push_back(myAtof(gyro_x));
			temp.push_back(myAtof(gyro_y));
			temp.push_back(myAtof(gyro_z));
			res.push_back(temp);
		}

		cout << acc_x << " " << gyro_z << " "<< time << endl;

	}
	file.close();

	return res;
}

float myAtof ( string &num){
   float tmp;
   sscanf ( num.c_str(), "%f" , &tmp);
   return tmp;
}
