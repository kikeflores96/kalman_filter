#ifndef IMU_H_
#define IMU_H_

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <vector>

#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5
//-------------------------------------------------
//------------------- MPU9250 ---------------------
//-------------------------------------------------

//---------------- CONFIGURATION ------------------
// MPU9259 is a 9DoFs Inertial Measurement Unit (IMU)
// MPU9250 = Accelerometer + Gyroscope + Magnetometer
// Magnetometer has its own address
#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C
// Gyroscope range options
#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18
// Accelerometer range options
#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

//###################################################
// ------------------- IMU CLASS --------------------
//###################################################
class IMU {
  private:
    // Magnetometer calibration
    double mag_mod = 46.5671;
    std::vector<std::vector<double>> mag_A_inv = {{1.34816508,  0.01820963, -0.09210088},
                                                  {0.01820963,  1.39352166 ,-0.0432281 },
                                                  {-0.09210088, -0.0432281 ,  1.38374719}};
    std::vector<double> mag_bias = {-0.29902116,0.3119834 , -1.30553365};

    // Accelerometer calibration
    double kx = 0.99612761, ky = 0.99079333, kz = 0.99889998;
    double ayz = -0.01358966, azy = -0.05504814, azx = 0.00130575;
    std::vector<std::vector<double>> acc_T = {{1, -ayz, azy},{0, 1, -azx},{0, 0, 1}};
    std::vector<std::vector<double>> acc_S = {{kx, 0, 0},{0, ky, 0},{0, 0, kz}};
    std::vector<double> acc_bias = {0.86002407, -1.50361382, -1.8833217 };
    
    // Gyroscope calibration
    std::vector<double> gyro_bias = {0.0514, -0.0007, 0.24};

  public:
    void initialize();
    void readsensor(int16_t imusensor[3][3]);
    void applyrange(int16_t imusensor[3][3], std::vector<double> &gyro, std::vector<double> &mag, std::vector<double> &acc);
    void applycalibration(std::vector<double> &gyro, std::vector<double> &acc, std::vector<double> &mag);
};

#endif