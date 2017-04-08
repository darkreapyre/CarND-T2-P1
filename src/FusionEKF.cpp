#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
//  ekf_.P_ = MatrixXd(4, 4);

//  // measurement covariance matrix - laser
//  R_laser_ << 0.0225, 0,
//              0, 0.0225;

//  // measurement covariance matrix - radar
//  R_radar_ << 0.09, 0, 0,
//              0, 0.0009, 0,
//              0, 0, 0.09;

//  // state covarience matrix P
//  ekf_.P_ << 1, 0, 0, 0,
//             0, 1, 0, 0,
//             0, 0, 1000, 0,
//             0, 0, 0, 1000;
  
//  // H_laser matrix
//  H_laser << 1, 0, 0, 0,
//        0, 1, 0, 0;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/

  if (!is_initialized_) {

    // Initialize state variables x
    //cout << "FusionEKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ = 1, 1, 1, 1;

    // Initialize transition matrix F
    ekf_.F_ = MaxtrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;
    
    // Initialize state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
               0, 1, 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1:

    // Initialize matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << 1, 0, 1, 0,
               0, 1, 0, 1,
               1, 0, 1, 0,
               0, 1, 0, 1;

    // Initialize noise variables
    noise_ax = 9;
    noise_ay = 9;

    // Initialize laser measurement covairance matrix R_laser
    R_laser << 0.0225, 0,
               0, 0.0225;

    // Initialiae laser H Matrix
    H_laser << 1, 0, 0, 0,
               0, 1, 0, 0;

    // Initilize radar measurement covariane matrix R_radar
    R_radar << 0.09, 0, 0,
               0, 0.0009, 0,
               0, 0, 0.09;

    // Initialize jacobian matrix
    Hj_ << 1, 1, 0, 0,
           1, 1, 0, 0,
           1, 1, 1, 1;

    // Set timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    
      // Convert radar from polar to cartesian coordinates and initialize state.
      // Range
      float rho = measurement_pack.raw_measurements_(0);
      // Bearing
      float theta = measurement_pack.raw_measurements_(1);
      ax = cos(theta) * rho;
      ay = sin(theta) * rho;

      if ( (ax == 0) || ( ay == 0) ){
        return;
      }

      ekf_.x_ = ax, ay, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

      if ( (measurement_pack.raw_measurements_[0] == 0) || (measurement_pack.raw_measurements_[1] == 0) ){
        return;
      }

      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      //ekf_.x_ << measurement_pack.raw_measurements_(0),
      //  measurement_pack.raw_measurements_(1), 1, 1;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
    }

    // Measurement
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    if (dt > 0.001) {
      // Update state transistion matrix
      ekf_.F_ << 1, 0, dt, 0,
                 0, 1, 0, dt,
                 0, 0, 1, 0,
                 0, 0, 0, 1;

      ekf_.Q_ << (pow(dt, 4) / 4 * noise_ax), 0, (pow(dt, 3) / 2 * noise_ax), 0,
                  0, (pow(dt, 4) / 4 * noise_ay), 0, (pow(dt, 3) / 2 * noise_ay),
                  (pow(dt, 3) / 2 * noise_ax), 0, pow(dt, 2) * noise_ax, 0,
                  0, (pow(dt, 3) / 2 * noise_ay), 0, pow(dt, 2) * noise_ay;
      

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

//  float noise_ax = 9;
//  float noise_ay = 9;
//  // Time is measured in seconds
//  float delta_t1 = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
//  //previous_timestamp_ = measurement_pack.timestamp_;
//  float delta_t2 = delta_t1 * delta_t1;
//  float delta_t3 = delta_t2 * delta_t1;
//  float delta_t4 = delta_t3 * delta_t1;

//  // F Matrix
//  ekf.F_ = MatrixXd(4, 4);
//  ekf.F_ << 1, 0, delta_t1, 0,
//            0, 1, 0, delta_t1,
//            0, 0, 1, 0,
//            0, 0, 0, 1;
  
//  // Q Matrix
//  ekf.Q_ = MatrixXd(4, 4);
//  ekf.Q_ << delta_t4 / 4 * noise_ax, 0, delta_t3 / 2 * noise_ax, 0,
//            0, delta_t4 / 4 * noise_ay, 0, delta_t3 / 2 * noise_ay,
//            delta_t3 / 2 * noise_ax, 0, delta_t2 * noise_ax, 0,
//            0, delta_t3 / 2 * noise_ay, 0, detlat_t2 * noise_ay;
  
      ekf_.Predict();
    }

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf.R_ = R_radar_;

    // Range
    double rho = sqrt(pow(ekf_.x_[0], 2) + pow(ekf_.x_[1], 2));
    // Bearing
    double theta = atan(ekf_.x_[1] / ekf_.x_[0]);
    // Rate
    double rhodot = ((ekf_.x_[0] * ekf_.x_[2] + ekf_.x_[1] * ekf_.x_[3]) / (sqrt(pow(ekf_.x_[0], 2) + pow(ekf_x_[1], 2))));

    MatrixXd z(3, 1);
    z << rho, theta, rhodot;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_, z)

  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser;
    
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
