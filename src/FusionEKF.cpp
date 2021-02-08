#include "FusionEKF.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    Hj_ = MatrixXd(3, 4);
    Q_ = MatrixXd(4, 4);

    //measurement covariance matrix - laser
    R_laser_ = MatrixXd(2, 2);
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ = MatrixXd(3, 3);
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    // state transition matrix
    F_ = MatrixXd(4, 4);
    F_ <<   1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    // state covariance matrix
    P_ = MatrixXd(4, 4);
    P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;
    
    // measurement matrix for lidar
    H_laser_ = MatrixXd(2, 4);
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;


    // acceleration noise components
    noise_ax = 9;
    noise_ay = 9;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * Need to convert radar from polar to cartesian coordinates.
     */

    // covariance matrix
    ekf_.P_ = P_;
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // convert radar from polar to cartesian coordinates and initialize state.         
        float rho = measurement_pack.raw_measurements_[0];
        float phi = measurement_pack.raw_measurements_[1];
        float rhodot = measurement_pack.raw_measurements_[2];
        // converting position to cartesian
        ekf_.x_(0) = rho * cos(phi);
        ekf_.x_(1) = rho * sin(phi);
        // although rhodot does not give velocity accurately but might be a good idea to use as initialization
        ekf_.x_(2) = rhodot * cos(phi);
        ekf_.x_(3) = rhodot * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        // Initialize state.
        ekf_.x_(0) = measurement_pack.raw_measurements_[0];
        ekf_.x_(1) = measurement_pack.raw_measurements_[1];
        ekf_.x_(2) = 0;
        ekf_.x_(3) = 0;
    }
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;

    return;
  }

  /**
   * Prediction
   */

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  float dt_2 = dt*dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Modify the F matrix so that the time is integrated
  F_(0,2) = dt;
  F_(1,3) = dt;
  // set the process covariance matrix Q
  Q_ <<  dt_4/4*noise_ax, 0              , dt_3/2*noise_ax, 0              ,
         0              , dt_4/4*noise_ay, 0              , dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0              , dt_2*noise_ax  , 0              ,
         0              , dt_3/2*noise_ay, 0              , dt_2*noise_ay  ;
  ekf_.F_ = F_;
  ekf_.Q_ = Q_;
  ekf_.Predict();
  
  /**
   * Update
   */

  /**
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
