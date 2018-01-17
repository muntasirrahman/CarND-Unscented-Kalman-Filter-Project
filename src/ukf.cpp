#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // State dimension
  n_x_ = 5;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;

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

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  /// Sigma point spreading parameter
  lambda_ = 3 - n_x_;

  /// Augmented state dimension
  n_aug_ = n_x_ + 2;

  // sigma point dimension
  n_sig_ = 2 * n_aug_ + 1;

  /// Weights of sigma points
  weights_ = VectorXd(n_sig_);
  weights_.fill(0.5 / (n_aug_ + lambda_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  /// Measurement noise covariance matrix
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
          0, std_radphi_ * std_radphi_, 0,
          0, 0, std_radrd_ * std_radrd_;

  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << std_laspx_ * std_laspx_, 0,
          0, std_laspy_ * std_laspy_;

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

  if (!is_initialized_) {

    double px;
    double py;
    double v;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

      double rho = meas_package.raw_measurements_[0]; // angle
      double phi = meas_package.raw_measurements_[1]; // bearing
      double rho_dot = meas_package.raw_measurements_[2]; // change of angle

      px = rho * cos(phi);
      py = rho * sin(phi);

      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);
      v = sqrt(vx * vx + vy * vy);

    } else { // For LIDAR measurements
      v = 0;
      px = meas_package.raw_measurements_[0];
      py = meas_package.raw_measurements_[1];

    }

    x_ << px, py, v, 0, 0;
    is_initialized_ = true;

  } else {

    // dt calculation
    double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
    Prediction(dt);

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
      UpdateRadar(meas_package);

    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
      UpdateLidar(meas_package);

    }
  }
  time_us_ = meas_package.timestamp_;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  /// 1. Generate Augmented Sigma Points

  // Augmented mean state
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug << x_, 0, 0;

  // Augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.setZero();
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  // Augmented sigma points
  MatrixXd X_sigma_aug = GenerateSigmaPoints(x_aug, P_aug, lambda_);

  /// 2. Predict Sigma Points, the state, the state covariance matrix
  Xsig_pred_ = PredictSigmaPoints(X_sigma_aug, delta_t, std_a_, std_yawdd_);

  /// 3. Predict Mean and Covariance
  x_ = Xsig_pred_ * weights_;
  P_.fill(0.0);

  for (int i = 0; i < n_sig_; i++) {  //iterate over sigma points

    VectorXd x_diff = Xsig_pred_.col(i) - x_; // state difference

    NormalizeAngleOnComponent(x_diff, 3); // verify value within valid range

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }

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

  /// 1. Predict measurement
  int n_z = 2;
  MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, n_sig_);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);

  for (int i = 0; i < n_sig_; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  S = S + R_lidar_;

  // 2. Update state

  VectorXd z = meas_package.raw_measurements_; //Incoming radar measurement

  //Matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  Tc.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {

    VectorXd z_diff = Zsig.col(i) - z_pred;

    VectorXd x_diff = Xsig_pred_.col(i) - x_; // state vector differences

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain
  MatrixXd K_gain = Tc * S.inverse();

  VectorXd z_diff = z - z_pred; //differences

  // state mean
  x_ = x_ + K_gain * z_diff;

  // covariance matrix
  P_ = P_ - K_gain * S * K_gain.transpose();

  //NIS Lidar Update
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;

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
  // Radar measument dimension
  int n_z = 3;
  /// 1. Predict measurement
  MatrixXd Zsig = MatrixXd(n_z, n_sig_);

  double px, py, v, yaw, v1, v2;

  //transform sigma points into measurement space
  for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points

    px = Xsig_pred_(0, i);
    py = Xsig_pred_(1, i);
    v = Xsig_pred_(2, i);
    yaw = Xsig_pred_(3, i);

    v1 = cos(yaw) * v;
    v2 = sin(yaw) * v;

    // measurement model
    Zsig(0, i) = sqrt(px * px + py * py); //r
    Zsig(1, i) = atan2(py, px); //phi
    Zsig(2, i) = (px * v1 + py * v2) / sqrt(px * px + py * py);   //r_dot
  }

  /// mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {

    VectorXd z_diff = Zsig.col(i) - z_pred; //differences

    NormalizeAngleOnComponent(z_diff, 1); //angle normalization

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  S = S + R_radar_;

  // 2. Update state
  // Incoming radar measurement
  VectorXd z = meas_package.raw_measurements_;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  Tc.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    NormalizeAngleOnComponent(z_diff, 1);

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    NormalizeAngleOnComponent(x_diff, 3);

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain
  MatrixXd K_gain = Tc * S.inverse();

  VectorXd z_diff = z - z_pred;

  //angle normalization
  NormalizeAngleOnComponent(z_diff, 1);

  //update state mean and covariance matrix
  x_ = x_ + K_gain * z_diff;
  P_ = P_ - K_gain * S * K_gain.transpose();

  //Latest NIS value
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}

/**
 * Generate sigma points
 * @param {VectorXd} x State vector
 * @param {MatrixXd} P Covariance matrix.
 * @param {double} lambda Sigma points spreading parameter
 */
MatrixXd UKF::GenerateSigmaPoints(VectorXd x, MatrixXd P, double lambda) {
  int n = x.size();
  //sigma point dimension
  MatrixXd Xsig = MatrixXd(n, n_sig_);

  //square root of P
  MatrixXd A = P.llt().matrixL();

  Xsig.col(0) = x;

  double lambdaNXSqrt = sqrt(lambda + n);
  for (int i = 0; i < n; i++) {
    Xsig.col(i + 1) = x + lambdaNXSqrt * A.col(i);
    Xsig.col(i + 1 + n) = x - lambdaNXSqrt * A.col(i);
  }
  return Xsig;
}

/**
 * Predict sigma points
 * @param {MatrixXd} Xsig Predicted Sigma points
 * @param {double} delta_t Time between k and k+1 in s
 * @param {double} nu_am Process noise SD longitudinal acceleration
 */
MatrixXd UKF::PredictSigmaPoints(MatrixXd Xsig, double delta_t, double nu_am, double nu_yawdd) {

  MatrixXd X_sigma_pred = MatrixXd(n_x_, n_sig_);

  double py, px, v, yaw, yawd, nu_a;
  const double EPS = 0.001;

  for (int i = 0; i < n_sig_; i++) {

    px = Xsig(0, i);
    py = Xsig(1, i);
    v = Xsig(2, i);
    yaw = Xsig(3, i);
    yawd = Xsig(4, i);
    nu_a = Xsig(5, i);
    nu_yawdd = Xsig(6, i);

    double px_p, py_p; //predicted state values

    // prevent division by zero
    if (fabs(yawd) > EPS) {
      px_p = px + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = py + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    } else {
      px_p = px + v * delta_t * cos(yaw);
      py_p = py + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    //write predicted sigma point into right column
    X_sigma_pred(0, i) = px_p;
    X_sigma_pred(1, i) = py_p;
    X_sigma_pred(2, i) = v_p;
    X_sigma_pred(3, i) = yaw_p;
    X_sigma_pred(4, i) = yawd_p;
  }

  return X_sigma_pred;
}

/**
 * Normalized the element-index of vector to fall inside valid range
 * @param {VectorXd} v_container container vector to be normalized
 * @param {int} index element position
 */
void UKF::NormalizeAngleOnComponent(VectorXd v_container, int index) {

  double phi_diff = v_container(index);
  if (phi_diff > M_PI) {
    v_container(index) = fmod(phi_diff, M_PI);
  }
  if (phi_diff < -M_PI) {
    v_container(index) = fmod(phi_diff, -M_PI);
  }
}