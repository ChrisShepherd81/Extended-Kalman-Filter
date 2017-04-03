#include "kalman_filter.h"
///////////////////////////////////////////////////////////////////////////////////////
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in) {
	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	I_ = MatrixXd::Identity(x_.size(), x_.size());
}
///////////////////////////////////////////////////////////////////////////////////////
void KalmanFilter::Predict()
{
#if PRINT
	std::cout << "KalmanFilter::Predict()\n";
#endif
	x_ = F_ * x_;
	MatrixXd F_T = F_.transpose();
	P_ = F_ * P_ * F_T + Q_;
}
///////////////////////////////////////////////////////////////////////////////////////
void KalmanFilter::Update(const VectorXd &z)
{
#if PRINT
	std::cout << "KalmanFilter::Update()\n";
#endif
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * S.inverse();

	//new estimate
	x_ = x_ + (K * y);
	P_ = (I_ - K * H_) * P_;
}
///////////////////////////////////////////////////////////////////////////////////////
void KalmanFilter::UpdateEKF(const VectorXd &z)
{
#if PRINT
	std::cout << "KalmanFilter::UpdateEKF()\n";
#endif
	VectorXd z_pred = tools.MapXprimeToPolarCoordinates(x_);
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * S.inverse();

	//new estimate
	x_ = x_ + (K * tools.AdjustPhiInVector(y));
	P_ = (I_ - K * H_) * P_;
}
///////////////////////////////////////////////////////////////////////////////////////
