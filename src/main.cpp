#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "Data.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::vector;
using std::string;
using std::cerr;
using std::cout;
using std::ifstream;
using std::ofstream;
using std::endl;
using std::istringstream;

void check_arguments(int argc, char* argv[]);
void check_files(ifstream& in_file, string& in_name, ofstream& out_file, string& out_name);
void read_file(ifstream& in_file, vector<MeasurementPackage> &measurement_pack_list, vector<GroundTruthPackage> &gt_pack_list);

int main(int argc, char* argv[])
{
#if LLDB_MI
  string in_file_name_ = "./data/sample-laser-radar-measurement-data-1.txt";
  string out_file_name_ = "./data/output.txt";
#else
  check_arguments(argc, argv);
  string in_file_name_ = argv[1];
  string out_file_name_ = argv[2];
#endif

  //open file streams
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  //Read in the measurements and ground truth from file
  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;
  read_file(in_file_, measurement_pack_list, gt_pack_list);

  // Create a Fusion EKF instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  //Call the EKF-based fusion
  size_t N = measurement_pack_list.size();
  for (size_t k = 0; k < N; ++k) {
    // start filtering from the second frame (the speed is unknown in the first
    // frame)
    fusionEKF.ProcessMeasurement(measurement_pack_list[k]);

    // output the estimation
    out_file_ << fusionEKF.ekf_.x_(0) << "\t";
    out_file_ << fusionEKF.ekf_.x_(1) << "\t";
    out_file_ << fusionEKF.ekf_.x_(2) << "\t";
    out_file_ << fusionEKF.ekf_.x_(3) << "\t";

    // output the measurements
    if (measurement_pack_list[k].sensor_type == MeasurementPackage::LASER) {
      // output the estimation
      out_file_ << measurement_pack_list[k].values(0) << "\t";
      out_file_ << measurement_pack_list[k].values(1) << "\t";
    } else if (measurement_pack_list[k].sensor_type == MeasurementPackage::RADAR) {
      // output the estimation in the cartesian coordinates
      float ro = measurement_pack_list[k].values(0);
      float phi = measurement_pack_list[k].values(1);
      out_file_ << ro * cos(phi) << "\t"; // p1_meas
      out_file_ << ro * sin(phi) << "\t"; // ps_meas
    }

    // output the ground truth packages
    out_file_ << gt_pack_list[k].values(0) << "\t";
    out_file_ << gt_pack_list[k].values(1) << "\t";
    out_file_ << gt_pack_list[k].values(2) << "\t";
    out_file_ << gt_pack_list[k].values(3) << "\n";

    estimations.push_back(fusionEKF.ekf_.x_);
    ground_truth.push_back(gt_pack_list[k].values);
  }

  // compute the accuracy (RMSE)
  Tools tools;
  cout << "Accuracy - RMSE:" << endl << tools.CalculateRMSE(estimations, ground_truth) << endl;

  // close files
  if (out_file_.is_open()) {
    out_file_.close();
  }

  if (in_file_.is_open()) {
    in_file_.close();
  }

  return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void check_arguments(int argc, char* argv[]) {
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name) {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_file(ifstream& in_file, vector<MeasurementPackage> &measurement_pack_list, vector<GroundTruthPackage> &gt_pack_list)
{
	string line;
	// prep the measurement packages (each line represents a measurement at a
	// timestamp)
	while (getline(in_file, line))
	{
		string sensor_type;
		MeasurementPackage meas_package;
		GroundTruthPackage gt_package;
		istringstream iss(line);
		long long timestamp;

		// reads first element from the current line
		iss >> sensor_type;
		if (sensor_type.compare("L") == 0)
		{
			// LASER MEASUREMENT

			// read measurements at this timestamp
			meas_package.sensor_type = MeasurementPackage::LASER;
			meas_package.values = VectorXd(2);
			float x;
			float y;
			iss >> x;
			iss >> y;
			meas_package.values << x, y;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
			measurement_pack_list.push_back(meas_package);
		}
		else if (sensor_type.compare("R") == 0)
		{
			// RADAR MEASUREMENT

			// read measurements at this timestamp
			meas_package.sensor_type = MeasurementPackage::RADAR;
			meas_package.values = VectorXd(3);
			float ro;
			float phi;
			float ro_dot;
			iss >> ro;
			iss >> phi;
			iss >> ro_dot;
			meas_package.values << ro, phi, ro_dot;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
			measurement_pack_list.push_back(meas_package);
		}

		// read ground truth data to compare later
		float x_gt;
		float y_gt;
		float vx_gt;
		float vy_gt;
		iss >> x_gt;
		iss >> y_gt;
		iss >> vx_gt;
		iss >> vy_gt;
		gt_package.values = VectorXd(4);
		gt_package.values << x_gt, y_gt, vx_gt, vy_gt;
		gt_pack_list.push_back(gt_package);
	}
}
