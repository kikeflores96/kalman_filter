#ifndef TCF_H_
#define TCF_H_

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <vector>
#include "../../lib/matrix_operations/matrix_operations.h"
#include "../../lib/EKF/EKF.h"
#include "../../lib/IMU/IMU.h"



class TCF{
  private:
      
    std::vector<double> mag_I = {23.872, -0.6498 , 39.991};
    std::vector<double> acc_I = {0.0, 0.0, 9.81};
    std::vector<std::vector<double>> B= createDiagMatrix(3, 1.0);
    std::vector<std::vector<double>> A= createDiagMatrix(3, 1.0);


  public:
    
    std::vector<double> qHat = {1.0, 0.0 ,0.0, 0.0};
    std::vector<double> qTriad = {1.0, 0.0 ,0.0, 0.0};
    std::vector<double> qTriad_prev = {1.0, 0.0 ,0.0, 0.0};
    std::vector<double> qHat_prev = {1.0, 0.0 ,0.0, 0.0};
    std::vector<double> qHat_bar = {1.0, 0.0 ,0.0, 0.0};
    Euler triad_yawpitchroll;
    Euler tcf_ypr;
    void initialize(IMU &imu);
    void TRIAD(const std::vector<double> &acc, const std::vector<double> &mag );
    void integrate(const std::vector<double> &gyro, const double dt);
    void combine();


    


};

#endif /* TCF_H_ */