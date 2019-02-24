#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::cout;
using std::endl;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  // accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    // ... your code here
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse /= estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * Calculate a Jacobian here.
   */
  MatrixXd Hj = MatrixXd::Zero(3, 4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // YOUR CODE HERE

  // check division by zero
  float sq_sum = px*px + py*py;
  if (fabs(sq_sum) < 0.0001) {
      cout << "Error - Division by zero" << endl;
      return Hj;
  }

  // compute the Jacobian matrix
  float mag = sqrt(sq_sum);
  float mag_3 = mag * sq_sum;
  Hj(0, 0) = px/mag;
  Hj(0, 1) = py/mag;

  Hj(1, 0) = -py/sq_sum;
  Hj(1, 1) = px/sq_sum;

  Hj(2, 0) = py*(vx*py-vy*px)/mag_3;
  Hj(2, 1) = px*(vy*px-vx*py)/mag_3;
  Hj(2, 2) = px/mag;
  Hj(2, 3) = py/mag;

  return Hj;
}
