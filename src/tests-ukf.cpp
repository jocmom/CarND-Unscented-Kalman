#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "Eigen/Dense"
#include "ukf.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

TEST_CASE( "Generate sigma points", "[GenerateSigmaPoints]" ) {
  UKF ukf;
  //set state dimension
  ukf.n_x_ = 5;  
  //set augmented dimension
  ukf.n_aug_ = 7;
  //define spreading parameter
  ukf.lambda_ = 3 - ukf.n_aug_;
  //Process noise standard deviation longitudinal acceleration in m/s^2
  ukf.std_a_ = 0.2;
  //Process noise standard deviation yaw acceleration in rad/s^2
  ukf.std_yawdd_ = 0.2;

  //set example state
  ukf.x_ <<   5.7441,
         1.3800,
         2.2049,
         0.5015,
         0.3528;

  //set example covariance matrix
  ukf.P_ <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  MatrixXd Xsig_aug_test = MatrixXd(ukf.n_aug_, ukf.n_sigma_);
  Xsig_aug_test <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
      1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,   1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,   2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,   0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528,  0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
         0,        0,        0,        0,        0,        0,  0.34641,        0,         0,        0,        0,        0,        0, -0.34641,        0,
         0,        0,        0,        0,        0,        0,        0,  0.34641,         0,        0,        0,        0,        0,        0, -0.34641;

  MatrixXd Xsig_pred_test = MatrixXd(ukf.n_x_, ukf.n_sigma_);
  Xsig_pred_test <<
  5.93553, 6.06251, 5.92217, 5.9415, 5.92361, 5.93516, 5.93705, 5.93553, 5.80832, 5.94481, 5.92935, 5.94553, 5.93589, 5.93401, 5.93553,
  1.48939, 1.44673, 1.66484, 1.49719, 1.508, 1.49001, 1.49022, 1.48939, 1.5308, 1.31287, 1.48182, 1.46967, 1.48876, 1.48855, 1.48939,
  2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.23954, 2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 2.17026, 2.2049,
  0.53678, 0.473387, 0.678098, 0.554557, 0.643644, 0.543372, 0.53678, 0.538512, 0.600173, 0.395462, 0.519003, 0.429916, 0.530188, 0.53678, 0.535048,
  0.3528, 0.299973, 0.462123, 0.376339, 0.48417, 0.418721, 0.3528, 0.387441, 0.405627, 0.243477, 0.329261, 0.22143, 0.286879, 0.3528, 0.318159;

  MatrixXd Xsig_aug = ukf.GenerateSigmaPoints();
  MatrixXd Xsig_pred = ukf.PredictSigmaPoints(Xsig_aug_test, 0.1);
  cout << "Xsig_aug:" << endl << Xsig_aug << endl;
  REQUIRE(Xsig_aug.cols() == Xsig_aug_test.cols());
  REQUIRE(Xsig_aug.rows() == Xsig_aug_test.rows());
  REQUIRE((Xsig_aug - Xsig_aug_test).norm() < 1e-4);

  cout << "Xsig_pred:" << endl << Xsig_pred << endl;
  REQUIRE(Xsig_pred.cols() == Xsig_pred_test.cols());
  REQUIRE(Xsig_pred.rows() == Xsig_pred_test.rows());
  REQUIRE((Xsig_pred - Xsig_pred_test).norm() < 1e-4);
}