#ifndef IMU_H_
#define IMU_H_
// Include dependencies
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <vector>
//---------------------------------------------------------------------------------------
// IMU Library
//---------------------------------------------------------------------------------------
// Author:  Enrique Flores
// Date:    15/07/2023
// Define I2C communication port
#define I2C_PORT i2c0
// Define I2C port pins: these corresponds to the physical GPIO in the
// Raspberry pi Pico device
#define I2C_SDA 4
#define I2C_SCL 5
//-------------------------------------------------
//------------------- MPU9250 ---------------------
//-------------------------------------------------

//---------------- CONFIGURATION ------------------
// MPU9259 is a 9DoFs Inertial Measurement Unit (IMU)
// MPU9250 = Accelerometer + Gyroscope + Magnetometer
// Memory address
#define    MPU9250_ADDRESS            0x68
// The magnetometer has a different address
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
    //###################################################
    // Magnetometer calibration
    //###################################################
    // Module of the reference magnetic field vector in mT
    double mag_mod = 46.5671;
    // Calibration matrix eq 4.20
    std::vector<std::vector<double>> mag_A_inv = {{ 1.33336896, 0.02303043,-0.07898045},
                                                  { 0.02303043, 1.33692563,-0.01454740},
                                                  {-0.07898045,-0.01454740, 1.32716053}};
    // Magnetometer bias vector eq 4.20
    std::vector<double> mag_bias = {-0.27334742, 0.20273002, -1.36129358};
    
    //###################################################
    // Accelerometer calibration
    //###################################################
    // Acelerometer scale factors
    double kx = 0.99612761, ky = 0.99079333, kz = 0.99889998;
    // Acelerometer misalignment angles
    double ayz = -0.01358966, azy = -0.05504814, azx = 0.00130575;
    // Misalignment matrix
    std::vector<std::vector<double>> acc_T = {{1, -ayz, azy},{0, 1, -azx},{0, 0, 1}};
    // Scale factor matrix
    std::vector<std::vector<double>> acc_S = {{kx, 0, 0},{0, ky, 0},{0, 0, kz}};
    // Accelerometer bias
    std::vector<double> acc_bias = {0.86002407, -1.50361382, -1.8833217 };
    
    //###################################################
    // Gyroscope calibration
    //###################################################
    // Gyroscope bias
    std::vector<double> gyro_bias = {0.05236351, -0.00233378, 0.23537489};
    
  public:
    // Initialize the IMU: set COM frequency, I2C pins, define sensors ranges
    void initialize();
    // Access to the MPU9250 memory registers
    void readsensor(int16_t imusensor[3][3]);
    // Apply sensor ranges to the int16_t raw measurements
    void applyrange(int16_t imusensor[3][3], std::vector<double> &gyro, std::vector<double> &mag, std::vector<double> &acc);
    // Apply calibration
    void applycalibration(std::vector<double> &gyro, std::vector<double> &acc, std::vector<double> &mag);
};
#endif