#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
    */
  VectorXd rmse(4);
	rmse << 0,0,0,0;
  unsigned int estimations_size = estimations.size();

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations_size == 0){
    cout << "CalculateRMSE Error() - estimations vector size is 0" << endl;
    return rmse;
  }
	if(estimations_size != ground_truth.size()){
    cout << "CalculateRMSE Error() - estimations/ground_truth vector sizes don't match" << endl;
    return rmse;
  }

	//accumulate squared residuals
	for(int i=0; i < estimations_size; ++i){
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
	}
	//calculate the mean
	rmse = rmse / estimations_size;

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
    * Calculate a Jacobian here.
  */

	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//pre-compute a set of terms to avoid repeated calculation
	float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	//check division by zero
	if(fabs(c1) < 0.0001){
		cout << "CalculateJacobian() - Error - Division by Zero" << endl;
		return Hj;
	}

	//compute the Jacobian matrix
	Hj << (px/c2), (py/c2), 0, 0,
    -(py/c1), (px/c1), 0, 0,
    py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Hj;
}

double Tools::NormalizeAngle(double angle) {
  /**
    * Normalize angle in radian, -pi/2 < angle < pi/2
  */
  if (fabs(angle) > M_PI) {
    angle -= round(angle / (2.0d * M_PI)) * (2.0d * M_PI);
  }
	return angle;
}
