/*
 * csvReader.h
 *
 *  Created on: Nov 16, 2017
 *      Author: jhu4
 */

#ifndef SRC_CSVREADER_H_
#define SRC_CSVREADER_H_

using namespace std;


vector< vector<float> > getData(string fileName, int startRow, int endRow);

float myAtof ( string &num);

#endif /* SRC_CSVREADER_H_ */
