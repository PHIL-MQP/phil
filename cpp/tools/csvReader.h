/*
 * csvReader.h
 *
 *  Created on: Nov 16, 2017
 *      Author: jhu4
 */

#ifndef SRC_CSVREADER_H_
#define SRC_CSVREADER_H_

using namespace std;

string imu_data_filename = "../../recorded_sensor_data/imu_calibration_11_14_20-00-00/imu_calibration_data_11_14.csv";

vector< vector<float> > getData(string fileName, int startRow, int endRow);

float myAtof ( string &num);

#endif /* SRC_CSVREADER_H_ */
