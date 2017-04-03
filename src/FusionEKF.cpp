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

  //observation model mapping matrix
  H_laser_ << 1, 0, 0, 0,
		  	  0, 1, 0, 0;

}
///////////////////////////////////////////////////////////////////////////////////////
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement)
{
  /*****************************************************************************
  *  Initialization
  ****************************************************************************/
  if (!is_initialized_)
  {
	  this->InitalizeKalmanFilter(measurement);
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  double dt = this->GetDeltaTime(measurement.timestamp_);
  ekf.Q = tools.CalculateProcessCovarianceMatrix(dt);
  ekf.Predict(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  //Use the sensor type to perform the update step
  if (measurement.sensor_type == MeasurementPackage::RADAR)
  {
	  // Radar updates
	  ekf.R = this->R_radar_;
	  ekf.H = tools.CalculateJacobian(ekf.GetX());
	  ekf.UpdateEKF(measurement.values);
  }
  else
  {
	  // Laser updates
	  ekf.R = this->R_laser_;
	  ekf.H = this->H_laser_;
	  ekf.Update(measurement.values);
  }

#if PRINT
  // print the output
  cout << "x_ = \n" << ekf.GetX() << endl;
  cout << "P_ = \n" << ekf.GetP() << endl;
#endif
}
///////////////////////////////////////////////////////////////////////////////////////
void FusionEKF::InitalizeKalmanFilter(const MeasurementPackage &first_measurement)
{
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
	this->previous_timestamp_ = first_measurement.timestamp_;

	if (first_measurement.sensor_type == MeasurementPackage::RADAR)
	{
		//Convert radar from polar to cartesian coordinates and initialize state.
		VectorXd x = tools.MapRadarPolarToCartesianPosition(first_measurement.values);
		MatrixXd Hj = tools.CalculateJacobian(x);
		std::cout << "Initialize radar\n";
		ekf.Init(x, P, F, Hj, R_radar_);
	}
	else if (first_measurement.sensor_type == MeasurementPackage::LASER)
	{
		//Initialize state.
		VectorXd x = VectorXd(4);
		x << first_measurement.values(0), first_measurement.values(1), 0, 0;
		std::cout << "Initialize laser\n";
		ekf.Init(x, P, F, H_laser_, R_laser_);
	}

	this->is_initialized_ = true;
}
///////////////////////////////////////////////////////////////////////////////////////
double FusionEKF::GetDeltaTime(long timestamp)
{
	//calculate time delta and convert from mu secs to secs.
	double dt = (timestamp - this->previous_timestamp_)/1.0e6;
	this->previous_timestamp_ = timestamp;
	return dt;
}
///////////////////////////////////////////////////////////////////////////////////////
