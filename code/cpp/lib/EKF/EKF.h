#ifndef EKF_H_
#define EKF_H_
//---------------------------------------------------------------------------------------
// EXTENDED KALMAN FILTER LIBRARY
//---------------------------------------------------------------------------------------
// Author:  Enrique Flores
// Date:    15/07/2023
// Import dependencies
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <vector>
#include "../../lib/matrix_operations/matrix_operations.h"

// Euler struct to store Euler angles
struct Euler{
  // Euler angles in rad
  double yaw;
  double pitch;
  double roll;
  // Euler angles in degrees
  double yaw_deg() const;
  double pitch_deg() const;
  double roll_deg() const;
  // Compute Euler angles from attitude quaternion q
  void getEulerangles(const std::vector<double> &q);
  // Compute Euler angles from rotation matrix R
  void Rot2Euler(const std::vector<std::vector<double>> &R);
};
//###################################################
// ------------------- EKF CLASS --------------------
//###################################################
class EKF{
  private:
      // Covarianze matrix P --> P_{k|k}
      std::vector<std::vector<double>> P      = createDiagMatrix(7, 0.01);
      // A priori estimate of the covarianze matrix P_bar --> P_{k|k-1}
      std::vector<std::vector<double>> P_bar  = createDiagMatrix(7, 0.01);
      // Process noise matrix
      std::vector<std::vector<double>> Q      = createDiagMatrix(7, 0.001);
      // Measurements noise matrix
      std::vector<std::vector<double>> R      = createDiagMatrix(6, 0.1);
      // Kalman Gain matrix
      std::vector<std::vector<double>> K;
      // Process matrix F
      std::vector<std::vector<double>> A;
      // Process control matrix G
      std::vector<std::vector<double>> B;
      // Observation matrix H
      std::vector<std::vector<double>> C;
      // A priori estimate of the state vector \hat{x}_{k|k-1}
      std::vector<double> xhat_bar  = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      // Estimate of the state in the previous iteration \hat{x}_{k-1|k-1}
      std::vector<double> xhat_prev = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      // Vector of observed variables computed from a priori estimate of
      // the state
      // Changed on 15/07/2023
      // Previously std::vector<double> yhat_bar = {0.0, 0.0, 0.0}; --> nosense
      std::vector<double> yhat_bar = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      // Reference gravity vector in the Inertial frame
      std::vector<double> accel_ref = {0.0, 0.0, 1.0};
      // Reference magnetic field vector in the Inertial frame
      std::vector<double> mag_ref   = {1.0, 0.0, 0.0};
  public:
    // Attitude quaternion
    std::vector<double> q         = {1.0, 0.0 ,0.0, 0.0};
    // Estimate of the state vector
    std::vector<double> xhat      = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // Compute the observation Jacobian matrix H
    std::vector<std::vector<double>> getJacobianMatrix(const std::vector<double> &ref);
    // Predict the measured variables from the state vector
    std::vector<double> predictAccelMag();
    // Compute the discrete time state matrix F
    void getA(const double dt);
    // Compute the discrete time control matrix G
    void getB(const double dt);
    // Method to perform the predict stage of the Kalman filter:
    // 1. State extrapolation equation eq. 5.41
    // 2. Covariance extrapolation equation eq. 5.42
    void predict(const std::vector<double> &gyro, const double dt);
    // Method to perform the update stage of the Kalman filter:
    // 3. Kalman Gain equation eq. 5.43
    // 4. State update equation eq. 5.44
    // 5. Covariance update equation eq. 5.45
    void update(const std::vector<double> &acc, const std::vector<double> &mag);
};

#endif /* EKF_H_ */


    //   std::vector<std::vector<double>> Q_hat      = {{1.6928999e-07, 0, 0},
    //                                                  {0, 1.7532337e-07, 0},
    //                                                  {0, 0, 1.9694976e-07}};
    //   std::vector<std::vector<double>> R      = {{2.6039008e-05, 0, 0, 0, 0, 0},
    //                                              {0 ,2.9455331e-05, 0, 0, 0, 0},
    //                                              {0, 0, 1.8108870e-05, 0, 0, 0},
    //                                              {0, 0, 0, 3.6906232e-05, 0, 0},
    //                                              {0, 0, 0, 0, 4.1092182e-05, 0},
    //                                              {0, 0, 0, 0, 0, 1.4462859e-05}};