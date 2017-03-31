#include "tools.h"
///////////////////////////////////////////////////////////////////////////////////////
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
	VectorXd rmse(4);
	rmse << 0,0,0,0;

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
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{
	MatrixXd Hj(3,4);

	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float px2py2 = std::pow(px, 2) + std::pow(py, 2);

	//check division by zero
	if(px2py2 == 0)
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

	return Hj;
}
///////////////////////////////////////////////////////////////////////////////////////
