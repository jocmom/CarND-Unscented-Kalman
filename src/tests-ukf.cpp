#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "Eigen/Dense"
#include "ukf.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

TEST_CASE( "generate sigma points", "[GenerateSigmaPoints]" ) {
  //set state dimension
  UKF ukf;
  ukf.n_x_ = 5;

  //define spreading parameter
  ukf.lambda_ = 3 - ukf.n_x_;

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

  MatrixXd Xsig_aug = ukf.GenerateSigmaPoints();
  REQUIRE(Xsig_aug(0, 0) == 5.7441);
    
}