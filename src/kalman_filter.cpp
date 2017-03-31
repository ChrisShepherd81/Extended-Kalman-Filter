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
	std::cout << "KalmanFilter::Predict()\n";
	x_ = F_ * x_;
	MatrixXd F_T = F_.transpose();
	P_ = F_ * P_ * F_T + Q_;
}
///////////////////////////////////////////////////////////////////////////////////////
void KalmanFilter::Update(const VectorXd &z)
{
	std::cout << "KalmanFilter::Update()\n";
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
void KalmanFilter::UpdateEKF(const VectorXd &z)
{
	std::cout << "KalmanFilter::UpdateEKF()\n";
	VectorXd z_pred = MapXprimeToPolarCoordinates(x_);
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
VectorXd KalmanFilter::MapXprimeToPolarCoordinates(const VectorXd& x_prime)
{
	VectorXd hx(3);

	//recover state parameters
	float px = x_prime(0);
	float py = x_prime(1);
	float vx = x_prime(2);
	float vy = x_prime(3);

	double px2py2 = std::pow(px, 2) + std::pow(py, 2);
	if(px2py2 <= 0)
	{
		std::cout << "Error on MapXprimeToPolarCoordinates()" << std::endl;
		return hx;
	}
	double sqrt_px2py2 = std::sqrt(px2py2);

	hx << 	sqrt_px2py2,
			std::atan2(py, px),
			(px*vx+py*vy)/sqrt_px2py2;

	std::cout<< "hx = \n" << hx << std::endl;
	return hx;
}
///////////////////////////////////////////////////////////////////////////////////////
