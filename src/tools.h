#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include <iostream>
#include <cmath>

#include "Eigen/Dense"

#define PRINT 1

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

class Tools
{
	const double _epsilon = 1e-9;
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
  MatrixXd CalculateProcessCovarianceMatrix(double dt,double noise_ax = 9,
  											double noise_ay=9 );

  /**
  * A helper method to map x_radar into cartesian coordinates
  */
  VectorXd MapRadarPolarToCartesianPosition(const VectorXd& x_radar);

  /**
   * A helper method to map x' into polar coordinates
   */
  VectorXd MapXprimeToPolarCoordinates(const VectorXd& x_prime);

  /**
    * A helper method that adjust phi in vector y to [-pi, pi]
    */
  VectorXd& AdjustPhiVectorY(VectorXd& y);

};

#endif /* TOOLS_H_ */
