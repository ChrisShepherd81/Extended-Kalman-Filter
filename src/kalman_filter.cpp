#include "kalman_filter.h"
///////////////////////////////////////////////////////////////////////////////////////
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	Q_ = Q_in;
	long x_size = x_.size();
	I_ = MatrixXd::Identity(x_size, x_size);
}
///////////////////////////////////////////////////////////////////////////////////////
void KalmanFilter::Predict()
{
	x_ = F_ * x_;
	MatrixXd F_T = F_.transpose();
	P_ = F_ * P_ * F_T + Q_;
}
///////////////////////////////////////////////////////////////////////////////////////
void KalmanFilter::Update(const VectorXd &z) {
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	P_ = (I_ - K * H_) * P_;
}
///////////////////////////////////////////////////////////////////////////////////////
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
///////////////////////////////////////////////////////////////////////////////////////
