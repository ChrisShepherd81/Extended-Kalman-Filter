/*
 * EkfFileReader.h
 *
 *  Created on: 04.04.2017
 *      Author: christian@inf-schaefer.de
 */
#ifndef SRC_EKFFILEREADER_H_
#define SRC_EKFFILEREADER_H_
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "Data.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::string;
using std::cerr;
using std::ifstream;
using std::ofstream;
using std::endl;
using std::vector;
using std::istringstream;

class EkfFileHandler
{
  string in_name;
  ifstream in_file;

  string out_name;
  ofstream out_file;
  
 public:
  EkfFileHandler(string in_file_name, string out_file_name);
  ~EkfFileHandler();

  bool check_files();
  
  void read_file(vector<MeasurementPackage> &measurement_pack_list,
                 vector<GroundTruthPackage> &gt_pack_list);

  void write_to_file(const VectorXd& v, size_t valsToWrite=0);
  void write_to_file(double val);
  void write_to_file(string val);
};

#endif /* SRC_EKFFILEREADER_H_ */
