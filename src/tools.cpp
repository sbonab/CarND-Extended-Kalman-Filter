#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  
  // Declaring the rmse vector output
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  if ( estimations.size() == 0 
       || estimations.size() != ground_truth.size() )
  {
	cout << "Invalid estimations or ground _truth data" << endl;
    return rmse;
  }
  
  // accumulate squared residuals
  for ( int i=0; i < estimations.size(); ++i )
  {
    VectorXd residual = estimations[i] - ground_truth[i];
    
    // element-wise multiplication
    residual = residual.array() * residual.array();
    
    // updating rmse
    rmse += residual;
  }
  
  // calculate the mean
  rmse = rmse / estimations.size();
  
  // calculate the squared root
  rmse = rmse.array().sqrt();
  
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
  // Declare the jacobian matrix
  MatrixXd Hj(3,4);
  
  // Recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  // Pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px + py*py;
  float c2 = sqrt(c1);
  float c3 = c1*c2;
  
  // Check division by zero
  if ( fabs(c1) < 0.00001 )
  {
    cout << "CalculateJacobian() - Error - Division by Zero" << endl;
    return Hj;
  }
  
  // Compute Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
        -(py/c1), (px/c1), 0, 0,
        py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
  
  return Hj;
}

float Tools::NormAngle(const float &angle){
  
  // defining pi
  float pi = atan(1)*4;
  
  // normalize angle
  float norm_angle;
  if ( angle > pi ){
    norm_angle = angle - floor(angle/pi) * pi;
  }
  else if ( angle < -pi ){
    norm_angle = angle - ceil(angle/pi) * pi;
  }  
  else {
    norm_angle = angle;
  }
  
  return norm_angle;
}