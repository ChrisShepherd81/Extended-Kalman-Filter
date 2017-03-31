/*
 * Data.hpp
 *
 *  Created on: 31.03.2017
 *      Author: christian
 */

#ifndef SRC_DATA_HPP_
#define SRC_DATA_HPP_

#include "Eigen/Dense"

class Data
{
public:
	enum SensorType {LASER,RADAR } sensor_type_;
	Eigen::VectorXd values;
};

class GroundTruthPackage : public Data
{
public:
  long timestamp_;
};

class MeasurementPackage : public Data
{
public:
  long long timestamp_;
};

#endif /* SRC_DATA_HPP_ */
