#include "ukf.h"
#include "tools.h"
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

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  // To initialize the state covariance matrix P, one option is to start with
  // the identity matrix. Instead of setting each of the diagonal values to 1,
  // you can try setting the diagonal values by how much difference you expect 
  // between the true state and the initialized x state vector (lesson 32)
  P_ = MatrixXd(5, 5);
  P_ = MatrixXd::Identity(5,5) / 2;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // The parameter std_a is the standard deviation of linear acceleration! 
  // Remember from the "CTRV Process Noise Vector" lecture that the linear
  // acceleration is being modeled as a Gaussian distribution with mean 
  // zero and standard deviation σ​a​​. In a Gaussian distribution, 
  // about 95% of your values are within 2*std_aa​​ (lesson 31).
  std_a_ = 1.;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.7;

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

  // time when the state is true, in us
  time_us_ = 0;

  // timestamp from previous measurement
  previous_timestamp_ = 0;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // set measurement dimension, radar can measure r, phi, and r_dot
  n_z_radar_ = 3;
  
  // set measurement dimension, radar can measure r, phi, and r_dot
  n_z_laser_ = 2;

  // Number of sigma points
  n_sigma_ = 2 * n_aug_ + 1;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  //create sigma point matrix
  Xsig_pred_ = MatrixXd(n_x_, n_sigma_);

  // predicted sigma points matrix in measurement space
  Zsig_radar_ = MatrixXd(n_z_radar_, n_sigma_);

  // predicted sigma points matrix in measurement space
  Zsig_laser_ = MatrixXd(n_z_laser_, n_sigma_);

  // predicted mean vector in measurement space
  z_pred_radar_ = VectorXd(n_z_radar_);

  // predicted mean vector in measurement space
  z_pred_laser_ = VectorXd(n_z_laser_);
  
  //measurement covariance matrix S
  S_radar_ = MatrixXd(n_z_radar_, n_z_radar_);

  //measurement covariance matrix S
  S_laser_ = MatrixXd(n_z_laser_, n_z_laser_);

  // Weights of sigma points
  weights_ = VectorXd(n_sigma_);
  // set weights
  weights_(0) = lambda_/(lambda_ + n_aug_);
  for (int i=1; i<n_sigma_; i++) {  //2n+1 weights
    weights_(i) = 0.5/(n_aug_+lambda_);
  }

  // the current NIS for radar
  NIS_radar_ = 0;

  // the current NIS for laser
  NIS_laser_ = 0;

  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage measurement_pack) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
      * Initialize the state  with the first measurement.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      cout << "Kalman Filter Initialization with RADAR values " << endl;
      //set the state with the initial location and velocity
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];
      // Although radar gives velocity data in the form of the range rate
      // rho_dot, a radar measurement does not contain enough information
      // to determine the state variable velocities vx​ and v​y​​ 
      x_ << rho*cos(phi), rho*sin(phi), 0, 0, 0; 
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      cout << "Kalman Filter Initialization with LIDAR values " << endl;
      //set the state with the initial location and zero velocity
      x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0, 0;
    }
    cout << "x input vector: " << endl << x_ << endl;

    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
	//compute the time elapsed between the current and previous measurements
  //dt - expressed in seconds
	double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;
  Prediction(dt);
  if(use_radar_ && measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    PredictRadarMeasurement();
    UpdateRadar(measurement_pack);
  }
  if(use_laser_ && measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    PredictLaserMeasurement();
    UpdateLidar(measurement_pack);
  }
}

Eigen::MatrixXd UKF::GenerateSigmaPoints() {
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sigma_);  

  // set first elements of x_aug = x and all other to 0
  x_aug.head(n_x_) = x_;
  for(int i=n_x_; i<n_aug_; ++i) {
    x_aug(i) = 0;  
  }
  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_,n_x_) = P_;
  P_aug(n_x_,n_x_) = std_a_*std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
  return Xsig_aug;
}

Eigen::MatrixXd UKF::PredictSigmaPoints(Eigen::MatrixXd Xsig_aug, double delta_t) {
  //calculate sqare of delta_t only once
  double delta_t_square = delta_t * delta_t;
  //create matrix with predicted sigma points as columns
  //MatrixXd Xsig_pred = MatrixXd(n_x_, n_sigma_);
  //predict sigma points
  for (int i = 0; i<n_sigma_; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t_square * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t_square * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t_square;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  } 
  return Xsig_pred_;
}

/**
 * Predicts state mean
 */
void UKF::PredictStateMean() {
  x_.fill(0.0);
  for (int i = 0; i < n_sigma_; i++) {  //iterate over sigma points
    x_ = x_+ weights_(i) * Xsig_pred_.col(i);
  }
}

/**
 * Predicts state covariance matrix 
 */
void UKF::PredictStateCovariance() {
  P_.fill(0.0);
  for (int i = 0; i < n_sigma_; i++) {  //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
}

void UKF::PredictRadarMeasurement() {  
  //transform sigma points into measurement space
  for (int i = 0; i < n_sigma_; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig_radar_(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig_radar_(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig_radar_(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  z_pred_radar_.fill(0.0);
  for (int i=0; i < n_sigma_; i++) {
      z_pred_radar_ = z_pred_radar_ + weights_(i) * Zsig_radar_.col(i);
  }

  S_radar_.fill(0.0);
  for (int i = 0; i < n_sigma_; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig_radar_.col(i) - z_pred_radar_;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S_radar_ = S_radar_ + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z_radar_,n_z_radar_);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S_radar_ = S_radar_ + R;
}

void UKF::PredictLaserMeasurement() {  
  //transform sigma points into measurement space
  for (int i = 0; i < n_sigma_; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    // measurement model
    Zsig_laser_(0,i) = p_x;
    Zsig_laser_(1,i) = p_y;
  }

  //mean predicted measurement
  z_pred_laser_.fill(0.0);
  for (int i=0; i < n_sigma_; i++) {
      z_pred_laser_ = z_pred_laser_ + weights_(i) * Zsig_laser_.col(i);
  }

  S_laser_.fill(0.0);
  for (int i = 0; i < n_sigma_; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig_laser_.col(i) - z_pred_laser_;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S_laser_ = S_laser_ + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z_laser_, n_z_laser_);
  R <<    std_laspx_*std_laspx_, 0,
          0, std_laspy_*std_laspy_;
  S_laser_ = S_laser_ + R;
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

  MatrixXd Xsig_aug = GenerateSigmaPoints();
  MatrixXd Xsig_pred_ = PredictSigmaPoints(Xsig_aug, delta_t);
  PredictStateMean();
  PredictStateCovariance();
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
  assert(meas_package.raw_measurements_.size() == 2);
  VectorXd z = VectorXd(n_z_laser_);
  z << meas_package.raw_measurements_[0], 
    meas_package.raw_measurements_[1]; 
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_laser_);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < n_sigma_; i++) {  //2n+1 sigma points

    //residual
    VectorXd z_diff = Zsig_laser_.col(i) - z_pred_laser_;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S_laser_.inverse();

  //residual
  VectorXd z_diff = z - z_pred_laser_;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S_laser_*K.transpose();
  
  NIS_laser_ = z_diff.transpose() * S_laser_.inverse() * z_diff;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */  
  assert(meas_package.raw_measurements_.size() == 3);
  VectorXd z = VectorXd(n_z_radar_);
  z << meas_package.raw_measurements_[0], 
    meas_package.raw_measurements_[1], 
    meas_package.raw_measurements_[2];
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_radar_);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < n_sigma_; i++) {  //2n+1 sigma points

    //residual
    VectorXd z_diff = Zsig_radar_.col(i) - z_pred_radar_;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S_radar_.inverse();

  //residual
  VectorXd z_diff = z - z_pred_radar_;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S_radar_*K.transpose();

  NIS_radar_ = z_diff.transpose() * S_radar_.inverse() * z_diff;
}
