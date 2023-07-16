#ifndef TCF_H_
#define TCF_H_
//---------------------------------------------------------------------------------------
// TRIAD + COMPLEMENTARY FILTER LIBRARY
//---------------------------------------------------------------------------------------
// Author:  Enrique Flores
// Date:    15/07/2023
// Import dependencies
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <vector>
#include "../../lib/matrix_operations/matrix_operations.h"
#include "../../lib/EKF/EKF.h"
#include "../../lib/IMU/IMU.h"

//###################################################
// ------------------- TCF CLASS --------------------
//###################################################
class TCF{

  private:
    // Magnetic field vector reference in mT
    std::vector<double> mag_I           = {23.872, -0.6498 , 39.991};
    // Gravity vector reference
    std::vector<double> acc_I           = {0.0, 0.0, 9.81};
    // R triad matrix
    std::vector<std::vector<double>> A  = createDiagMatrix(3, 1.0);
    // S triad matrix
    std::vector<std::vector<double>> B  = createDiagMatrix(3, 1.0);

  public:
    // Attitude quaternion estimation
    std::vector<double> qHat        = {1.0, 0.0 ,0.0, 0.0};
    // Attitude quaternion estimation from TRIAD algorithm
    std::vector<double> qTriad      = {1.0, 0.0 ,0.0, 0.0};
    std::vector<double> qTriad_prev = {1.0, 0.0 ,0.0, 0.0};
    // Attitude quaternion estimation in previous iteration
    std::vector<double> qHat_prev   = {1.0, 0.0 ,0.0, 0.0};
    // Attitude quaternion estimation from time integration
    std::vector<double> qHat_bar  = {1.0, 0.0 ,0.0, 0.0};
    // Euler angles computed from TRIAD algorithm
    Euler triad_yawpitchroll;
    // Euler angles estimated by the TRIAD+Complementary filter
    // algorithm
    Euler tcf_ypr;
    // Initialize Complementary Filter: Compute R triad and
    // get initial conditions for time integration
    void initialize(IMU &imu);
    // Estimate system attitude using TRIAD algorithm
    void TRIAD(const std::vector<double> &acc, const std::vector<double> &mag );
    // Integrate gyroscope measurements
    void integrate(const std::vector<double> &gyro, const double dt);
    // Combine TRIAD estimation with time integration using a
    // complementary filter
    void combine();
};

#endif /* TCF_H_ */