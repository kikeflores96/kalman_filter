#ifndef EKF_H_
#define EKF_H_

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <vector>
#include "../../lib/matrix_operations/matrix_operations.h"

struct Euler{
    double yaw;
    double pitch;
    double roll;

    double yaw_deg() const;
    double pitch_deg() const;
    double roll_deg() const;

    void getEulerangles(const std::vector<double> &q);
};




class EKF{
  private:
      
      std::vector<std::vector<double>> P      = createDiagMatrix(7, 0.01);
      std::vector<std::vector<double>> P_bar  = createDiagMatrix(7, 0.01);
      std::vector<std::vector<double>> Q      = createDiagMatrix(7, 0.001);
      std::vector<std::vector<double>> R      = createDiagMatrix(6, 0.1);

      std::vector<std::vector<double>> K = createDiagMatrix(7, 1);
      std::vector<std::vector<double>> A = createDiagMatrix(7, 1);
      std::vector<std::vector<double>> B;
      std::vector<std::vector<double>> C;
      // std::vector<std::vector<double>> C(row, std::vector<double>(column, 0.0));
      
      
      std::vector<double> bias = {0.0, 0.0, 0.0};


      
      std::vector<double> xhat_bar  = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      std::vector<double> xhat_prev = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

      std::vector<double> yhat_bar = {0.0, 0.0, 0.0};
      std::vector<double> accel_ref = {0.0, 0.0, 1};
      std::vector<double> mag_ref   = {0.0, 1, 0.0};


  public:
    std::vector<double> q = {1.0, 0.0 ,0.0, 0.0};
    std::vector<double> xhat      = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<std::vector<double>> getJacobianMatrix(const std::vector<double> &ref);
    std::vector<double> predictAccelMag();
    void getA(const int dt);
    void getB(const int dt);
    void predict(std::vector<double> &gyro, const int dt);
    void update(std::vector<double> &acc, std::vector<double> &mag);
};

#endif /* EKF_H_ */



    // double xHat[7];
    // double quaternion[4] {1.0, 0.0, 0.0, 0.0};     // Initial estimate of the quaternion
    // double bias[3] = {0.0, 0.0, 0.0};              // Initial estimate of the gyro bias

    // double xHat[7], xHatBar[7], xHatPrev[7];
    // double yHatBar[3];
    // double P[7][7],PBar[7][7], Q[7][7], R[6][6];
    // double K;
    // double A;
    // double B;
    // double C;
    // double accelReference[3] = {0., 0., -1.};
    // double magReference[3] = {0., -1., 0.};
    // double mag_Ainv[3][3] = {{4.34883093e-03, -4.85734863e-05,  2.71406130e-04},
    //                         {-4.85734863e-05,  4.10542322e-03, -6.63845234e-05},
    //                         {2.71406130e-04, -6.63845234e-05,  4.34471427e-03}};
    // double mag_b[3] = {7.43143287, -95.03272074, 397.19236686};
    // self.p = np.identity(7) * 0.01
    // self.Q = np.identity(7) * 0.001
    // self.R = np.identity(6) * 0.1
    // self.K = None
    // self.A= None
    // self.B = None
    // self.C = None
    // self.pBar = None