#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* predicted sigma points matrix in measurement space
  MatrixXd Zsig_radar_;

  ///* predicted sigma points matrix in measurement space
  MatrixXd Zsig_laser_;

  ///* predicted mean vector in measurement space
  VectorXd z_pred_radar_;

  ///* predicted mean vector in measurement space
  VectorXd z_pred_laser_;

  ///* measurement noise covariance matrix S
  MatrixXd S_radar_;

  ///* measurement noise covariance matrix S
  MatrixXd S_laser_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* timestamp from previous measurement
  long previous_timestamp_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Measurement dimension
  int n_z_radar_;

  ///* Measurement dimension
  int n_z_laser_;

  ///* Number of sigma points
  int n_sigma_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage measurement_pack);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  /**
   * Generate augmented Sigma Points
   */
  Eigen::MatrixXd GenerateSigmaPoints(void);

  /**
   * Predict Sigma points
   * @param Xsig_aug augmented sigma points
   * @param delta_t Time between k and k+1 in s
   */
  Eigen::MatrixXd PredictSigmaPoints(Eigen::MatrixXd Xsig_aug, double delta_t);

  /**
   * Predict state mean 
   */
  void PredictStateMean(void);

  /**
   * Predict state covariance 
   */
  void PredictStateCovariance(void);

  /**
   * Transformation in measurement space and prediction of radar measurement
   */
  void PredictRadarMeasurement(void);

  /**
   * Transformation in measurement space and prediction of lidar measurement
   */
  void PredictLaserMeasurement(void);
};

#endif /* UKF_H */
