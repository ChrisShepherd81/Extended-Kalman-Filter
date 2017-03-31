#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include <iostream>

#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

class Tools
{
public:
  /**
  * Constructor.
  */
  Tools() {}

  /**
  * Destructor.
  */
  virtual ~Tools() {}

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
		  	  	  	  	  	    const std::vector<Eigen::VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

  /**
  * A helper method to calculate process noise covariance matrix.
  */
  MatrixXd CalculateProcessCovarianceMatrix(double dt,double noise_ax = 9.0,
  											double noise_ay=9.0 );

};

#endif /* TOOLS_H_ */
