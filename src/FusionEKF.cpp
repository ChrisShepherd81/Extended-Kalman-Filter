#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

///////////////////////////////////////////////////////////////////////////////////////
FusionEKF::FusionEKF()
{
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
		  	  0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 	0.09, 0, 0,
				0, 0.0009, 0,
				0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1, 0, 0, 0,
		  	  0, 1, 0, 0;

  VectorXd x = VectorXd(4);
  x << 1, 1, 1, 1;
  Hj_ = tools.CalculateJacobian(x);

}
///////////////////////////////////////////////////////////////////////////////////////
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement)
{
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;

    VectorXd x = VectorXd(4);
    x << 1, 1, 1, 1;

    //the initial transition matrix F
    MatrixXd F = MatrixXd(4, 4);
    F <<  1, 0, 1, 0,
		  0, 1, 0, 1,
		  0, 0, 1, 0,
		  0, 0, 0, 1;

    //state covariance matrix P
    MatrixXd P = MatrixXd(4, 4);
    P <<  1, 0, 0, 0,
		  0, 1, 0, 0,
		  0, 0, 1000, 0,
		  0, 0, 0, 1000;

    //initalize time
	this->previous_timestamp_ = measurement.timestamp_;

    if (measurement.sensor_type == MeasurementPackage::RADAR) {
      /**
      TODO Convert radar from polar to cartesian coordinates and initialize state.
      */
    	std::cout << "Initalize radar\n";
    	ekf_.Init(x, P, F, Hj_, R_radar_);
    }
    else if (measurement.sensor_type == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
    	std::cout << "Initalize laser\n";
    	ekf_.Init(x, P, F, H_laser_, R_laser_);
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  //TODO time?
  double dt = (measurement.timestamp_ - this->previous_timestamp_)/100000.0;
  this->previous_timestamp_ = measurement.timestamp_;

  std::cout << "time: " << dt << std::endl;

  //Update the state transition matrix F according to the new elapsed time.
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  //Update the process noise covariance matrix
  ekf_.Q_ = tools.CalculateProcessCovarianceMatrix(dt);

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  //Use the sensor type to perform the update step
  if (measurement.sensor_type == MeasurementPackage::RADAR)
  {
	  // Radar updates
	  ekf_.Update(measurement.values);
  }
  else
  {
	  // Laser updates
	  ekf_.UpdateEKF(measurement.values);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
///////////////////////////////////////////////////////////////////////////////////////
