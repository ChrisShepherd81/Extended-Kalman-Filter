#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "Data.hpp"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
  FusionEKF();
  virtual ~FusionEKF() {};

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf;

private:
  void InitalizeKalmanFilter(const MeasurementPackage &first_measurement);
  double GetDeltaTime(long timestamp);

  // check whether the tracking toolbox was initiallized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long previous_timestamp_;

  //measurement noise covariance matrix laser
  Eigen::MatrixXd R_laser_;
  //measurement noise covariance matrix radar
  Eigen::MatrixXd R_radar_;
  //measurement model mapping matrix
  Eigen::MatrixXd H_laser_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
};

#endif /* FusionEKF_H_ */
