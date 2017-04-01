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

  //measurement noise matrix - laser
  R_laser_ << 0.0225, 0,
		  	  0,      0.0225;

  //measurement noise matrix - radar
  R_radar_ << 	0.09, 0,      0,
				0,    0.0009, 0,
				0,    0,      0.09;

  //measurement matrix
  H_laser_ << 1, 0, 0, 0,
		  	  0, 1, 0, 0;

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

    //Initialize time
	this->previous_timestamp_ = measurement.timestamp_;

    if (measurement.sensor_type == MeasurementPackage::RADAR)
    {
		//Convert radar from polar to cartesian coordinates and initialize state.
    	VectorXd x = tools.MapRadarPolarToCartesianPosition(measurement.values);
		MatrixXd Hj = tools.CalculateJacobian(x);
    	std::cout << "Initialize radar\n";
    	ekf_.Init(x, P, F, Hj, R_radar_);
    }
    else if (measurement.sensor_type == MeasurementPackage::LASER)
    {
		//Initialize state.
    	VectorXd x = VectorXd(4);
    	x << measurement.values(0), measurement.values(1), 0, 0;
    	std::cout << "Initialize laser\n";
    	ekf_.Init(x, P, F, H_laser_, R_laser_);
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  //calculate time delta and convert from mu secs to secs.
  double dt = (measurement.timestamp_ - this->previous_timestamp_)/1.0e6;
  this->previous_timestamp_ = measurement.timestamp_;

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
	  ekf_.R_ = this->R_radar_;
	  ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
	  ekf_.UpdateEKF(measurement.values);
  }
  else
  {
	  // Laser updates
	  ekf_.R_ = this->R_laser_;
	  ekf_.H_ = this->H_laser_;
	  ekf_.Update(measurement.values);
  }

  // print the output
  cout << "x_ = \n" << ekf_.x_ << endl;
  cout << "P_ = \n" << ekf_.P_ << endl;
}
///////////////////////////////////////////////////////////////////////////////////////
