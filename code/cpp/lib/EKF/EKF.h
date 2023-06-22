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
    void Rot2Euler(const std::vector<std::vector<double>> &R);
};




class EKF{
  private:
      
      std::vector<std::vector<double>> P      = createDiagMatrix(7, 0.01);
      std::vector<std::vector<double>> P_bar  = createDiagMatrix(7, 0.01);
      std::vector<std::vector<double>> Q      = createDiagMatrix(7, 0.001);
      std::vector<std::vector<double>> R      = createDiagMatrix(6, 0.1);

      std::vector<std::vector<double>> K;
      std::vector<std::vector<double>> A;
      std::vector<std::vector<double>> B;
      std::vector<std::vector<double>> C;

      std::vector<double> xhat_bar  = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      std::vector<double> xhat_prev = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

      std::vector<double> yhat_bar = {0.0, 0.0, 0.0};
      std::vector<double> accel_ref = {0.0, 0.0, 1.0};
      std::vector<double> mag_ref   = {-1.0, 0.0, 0.0};


  public:
    std::vector<double> q = {1.0, 0.0 ,0.0, 0.0};
    std::vector<double> xhat      = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<std::vector<double>> getJacobianMatrix(const std::vector<double> &ref);
    std::vector<double> predictAccelMag();
    void getA(const double dt);
    void getB(const double dt);
    void predict(const std::vector<double> &gyro, const double dt);
    void update(const std::vector<double> &acc, const std::vector<double> &mag);
};

#endif /* EKF_H_ */