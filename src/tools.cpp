#include "tools.h"
///////////////////////////////////////////////////////////////////////////////////////
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
	VectorXd rmse = VectorXd::Zero(4);

	if(estimations.size() != ground_truth.size() || estimations.size() <= 0 )
	{
		std::cout << "Error in CalculateRMSE()" << std::endl;
		return rmse;
	}

	//accumulate squared residuals
	for(size_t i=0; i < estimations.size(); ++i)
	{
		VectorXd temp = estimations[i] - ground_truth[i];
		temp = temp.array()*temp.array();
		rmse += temp;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}
///////////////////////////////////////////////////////////////////////////////////////
MatrixXd Tools::CalculateProcessCovarianceMatrix(double dt,double noise_ax, double noise_ay )
{
	MatrixXd Q = MatrixXd::Zero(4,4);
	double dt_2 = std::pow(dt,2);
	double dt_3 = dt_2*dt;
	double dt_4 = dt_3*dt;
	Q	<< (0.25*dt_4*noise_ax), 0, (0.5*dt_3*noise_ax), 0,
			0, (0.25*dt_4*noise_ay), 0, (0.5*dt_3*noise_ay),
			(0.5*dt_3*noise_ax), 0, (dt_2*noise_ax), 0,
			0, (0.5*dt_3*noise_ay), 0, (dt_2*noise_ay);
	return Q;
}
///////////////////////////////////////////////////////////////////////////////////////
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{
	MatrixXd Hj = MatrixXd::Zero(3,4);

	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float px2py2 = std::pow(px, 2) + std::pow(py, 2);

	//check division by zero
	if(std::fabs(px2py2) < _epsilon)
	{
		std::cout << "CalculateJacobian() - Error - Division by Zero" << std::endl;
		return Hj;
	}

	float sqr_px2py2 = std::sqrt(px2py2);
	float sqr_px2py2_3 = std::sqrt(std::pow(px2py2, 3));

	//compute the Jacobian matrix
	Hj(0,0) = px/sqr_px2py2;
	Hj(0,1) = py/sqr_px2py2;

	Hj(1,0) = -py/px2py2;
	Hj(1,1) = px/px2py2;

	Hj(2,0) = (py*(vx*py-vy*px))/sqr_px2py2_3;
	Hj(2,1) = (px*(vy*px-vx*py))/sqr_px2py2_3;
	Hj(2,2) = px/sqr_px2py2;
	Hj(2,3) = py/sqr_px2py2;
#if PRINT
	std::cout<< "Hj = \n" << Hj << std::endl;
#endif
	return Hj;
}
///////////////////////////////////////////////////////////////////////////////////////
VectorXd Tools::MapRadarPolarToCartesianPosition(const VectorXd& x_radar)
{
	VectorXd result = VectorXd::Zero(4);
	double rho = x_radar(0);
	double phi = x_radar(1);

	result(0) = rho*std::cos(phi);
	result(1) = rho*std::sin(phi);

	return result;
}
///////////////////////////////////////////////////////////////////////////////////////
VectorXd Tools::MapXprimeToPolarCoordinates(const VectorXd& x_prime)
{
	VectorXd hx = VectorXd::Zero(3);

	//recover state parameters
	float px = x_prime(0);
	float py = x_prime(1);
	float vx = x_prime(2);
	float vy = x_prime(3);

	double px2py2 = std::pow(px, 2) + std::pow(py, 2);
	if(std::fabs(px2py2) < _epsilon)
	{
		std::cout << "Error on MapXprimeToPolarCoordinates()" << std::endl;
		return hx;
	}
	double sqrt_px2py2 = std::sqrt(px2py2);

	hx << 	sqrt_px2py2,
			std::atan2(py, px),
			(px*vx+py*vy)/sqrt_px2py2;
#if PRINT
	std::cout<< "hx = \n" << hx << std::endl;
#endif
	return hx;
}
///////////////////////////////////////////////////////////////////////////////////////
VectorXd& Tools::AdjustPhiVectorY(VectorXd& y)
{
	double phi = y(1);
	while( std::fabs(phi) > M_PI)
	{
		if(phi < 0)
			phi += 2*M_PI;
		else
			phi -= 2*M_PI;
	}
	y(1) = phi;
	return y;
}
///////////////////////////////////////////////////////////////////////////////////////
