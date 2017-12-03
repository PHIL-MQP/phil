/*
 * csvReader.h
 *
 *  Created on: Nov 16, 2017
 *      Author: jhu4
 */

#ifndef SRC_CSVREADER_H_
#define SRC_CSVREADER_H_

std::vector<float> getData(std::string file_name, int row_number);

std::vector< std::vector<float> > getData(std::string fileName, int startRow, int endRow);

float myAtof ( std::string &num);

#endif /* SRC_CSVREADER_H_ */
