#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO: done

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  // Initial time is zero
  time_us_ = 0;

  // state vector dimensions
  n_x_ = 5;

  // augmented state vector dimensions
  n_aug_ = 7;

  // initialise lambda parameter for calculating sigma points
  lambda_ = 3 - n_aug_;

  // initialise weights based on lambda and
  weights_ = VectorXd(n_aug_);

  weights_(0) = lambda_/(lambda_ + n_aug_);
  for (int i = 1; i < 2*n_aug_ + 1; i++) {
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  }

  Xsig_pred_ =  MatrixXd(n_x_, 2*n_aug_ + 1);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */


  // Initialise covariance matrix

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO: in progress

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  SigmaPointPrediction(AugmentedSigmaPoints(), delta_t);
  PredictMeanCovariance();


}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}

MatrixXd UKF::AugmentedSigmaPoints() {

  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);
  x_aug.setZero();
  x_aug.head(n_x_) = x_;

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);
  P_aug.setZero();
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_*std_a_;
  P_aug(6, 6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  // predict sigma points
  MatrixXd Xsig_aug_;
  Xsig_aug_.setZero();
  Xsig_aug_.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug_.col(i+1) = x_aug + sqrt(lambda_ + n_aug_)*A.col(i);
    Xsig_aug_.col(i+n_aug_+1) = x_aug - sqrt(lambda_ + n_aug_)*A.col(i);
  }
  return Xsig_aug_;
}

void UKF::SigmaPointPrediction(MatrixXd Xsig_aug_, double delta_t) {

  //predict sigma points
  for (int i = 0; i < 2*n_aug_ + 1; i++) {

    double v = Xsig_aug_(2, i);
    double psi = Xsig_aug_(3, i);
    double psi_d = Xsig_aug_(4, i);
    double nu_a = Xsig_aug_(5, i);
    double nu_psi_dd = Xsig_aug_(6, i);

    VectorXd x_old = Xsig_aug_.col(i).head(n_x_);
    VectorXd x_1 = VectorXd(5);
    VectorXd x_2 = VectorXd(5);

    //avoid division by zero
    if (!psi_d)
      x_1 << v * cos(psi) * delta_t,
          v * sin(psi) * delta_t,
          0,
          0,
          0;
    else
      x_1 << v / psi_d * (sin(psi + psi_d * delta_t) - sin(psi)),
          v / psi_d * (-cos(psi + psi_d * delta_t) + cos(psi)),
          0,
          psi_d * delta_t,
          0;

    x_2 << 0.5 * pow(delta_t, 2) * cos(psi) * nu_a,
        0.5 * pow(delta_t, 2) * sin(psi) * nu_a,
        delta_t * nu_a,
        0.5 * pow(delta_t, 2) * nu_psi_dd,
        delta_t * nu_psi_dd;

    //write predicted spigma points into right column
    Xsig_pred_.col(i) = x_old + x_1 + x_2;
  }
}

void UKF::PredictMeanCovariance() {

  x_.setZero();
  P_.setZero();
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    P_ += weights_(i) * (Xsig_pred_.col(i) - x_)*(Xsig_pred_.col(i) - x_).transpose();
  }
}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out) {

  const int n_z = 3;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  z_pred.setZero();
  //transform sigma points into measurement space
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    double px =       Xsig_pred_(0, i);
    double py =       Xsig_pred_(1, i);
    double v =        Xsig_pred_(2, i);
    double psi =      Xsig_pred_(3, i);
    double psi_d =    Xsig_pred_(4, i);

    double rho =      sqrt(pow(px, 2) + pow(py, 2));
    double phi =      atan2(py, px);
    double rho_d =    (px*v*cos(psi) + py*v*sin(psi))/rho;

    Zsig.col(i) <<    rho,
                      phi,
                      rho_d;

    //calculate mean predicted measurement
    z_pred += weights_(i) * Zsig.col(i);
  }

  //calculate innovation covariance matrix S
  MatrixXd R = MatrixXd(n_z, n_z);
  R <<  std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0, std_radrd_*std_radrd_;

  S.setZero();
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    S += weights_(i) * (Zsig.col(i) - z_pred) * (Zsig.col(i) - z_pred).transpose();
  }
  S += R;

  *S_out = S;
  *z_out = z_pred;
}

void UKF::PredictLidarMeasurement(VectorXd* z_out, MatrixXd* S_out) {

  const int n_z = 2;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  z_pred.setZero();
  //transform sigma points into measurement space
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    double px =       Xsig_pred_(0, i);
    double py =       Xsig_pred_(1, i);

    Zsig.col(i) <<    px,
                      py;

    //calculate mean predicted measurement
    z_pred += weights_(i) * Zsig.col(i);
  }

  //calculate innovation covariance matrix S
  MatrixXd R = MatrixXd(n_z, n_z);
  R <<  std_laspx_*std_laspx_, 0,
        0, std_laspy_*std_laspy_;

  S.setZero();
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    S += weights_(i) * (Zsig.col(i) - z_pred) * (Zsig.col(i) - z_pred).transpose();
  }
  S += R;

  *S_out = S;
  *z_out = z_pred;
}

void UKF::UpdateRadarState(MatrixXd& Zsig, VectorXd& z_pred, VectorXd& z, MatrixXd& S) {

  const int n_z = 3;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.setZero();
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    Tc += weights_(i) * (Xsig_pred_.col(i) - x_) * (Zsig.col(i) - z_pred).transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = MatrixXd(n_z, n_z);
  K = Tc * S.inverse();

  //update state mean and covariance matrix
  x_ = x_ + K * (z - z_pred);
  P_ = P_ - K * S * K.transpose();
}

void UKF::UpdateLidarState(MatrixXd& Zsig, VectorXd& z_pred, VectorXd& z, MatrixXd& S) {

  const int n_z = 2;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.setZero();
  for (int i = 0; i < 2*n_aug_ + 1; i++) {
    Tc += weights_(i) * (Xsig_pred_.col(i) - x_) * (Zsig.col(i) - z_pred).transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = MatrixXd(n_z, n_z);
  K = Tc * S.inverse();

  //update state mean and covariance matrix
  x_ = x_ + K * (z - z_pred);
  P_ = P_ - K * S * K.transpose();
}