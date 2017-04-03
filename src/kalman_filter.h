#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "tools.h"

#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter
{
	// covariance matrix
	Eigen::MatrixXd P_;

	// state vector
	Eigen::VectorXd x_;

	// state transition model
	Eigen::MatrixXd F_;

	// identity matrix
	Eigen::MatrixXd I_;

	//Tools class with helper functions
	Tools tools;

	//Updates the state transition model with 'deltaTime'
	void updateStateTransitionMatrix(double deltaTime);

public:
	// process noise covariance matrix
	Eigen::MatrixXd Q_;

	// measurement noise covariance matrix
	Eigen::MatrixXd R_;

	// measurement matrix
	Eigen::MatrixXd H_;

	// jacobi matrix
	Eigen::MatrixXd Hj_;

	KalmanFilter() {};
	virtual ~KalmanFilter() {};

	/**
	* Init Initializes Kalman filter
	* @param x_in Initial state
	* @param P_in Initial state covariance
	* @param F_in Transition matrix
	* @param H_in Measurement matrix
	* @param R_in Measurement covariance matrix
	*/
	void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
	  Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in);

	/**
	* Prediction Predicts the state and the state covariance
	* using the process model
	* @param deltaTime Time between k and k+1 in s
	*/
	void Predict(double deltaTime);

	/**
	* Updates the state by using standard Kalman Filter equations
	* @param z The measurement at k+1
	*/
	void Update(const Eigen::VectorXd &z);

	/**
	* Updates the state by using Extended Kalman Filter equations
	* @param z The measurement at k+1
	*/
	void UpdateEKF(const Eigen::VectorXd &z);

	VectorXd GetX() const
	{
		return x_;
	}

	MatrixXd GetP() const
	{
		return P_;
	}
};

#endif /* KALMAN_FILTER_H_ */
