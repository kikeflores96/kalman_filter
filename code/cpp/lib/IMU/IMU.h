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
    // Sensor_axis = Sensor_axis_SF*(sensor_axis_raw - sensor_axis_bias)
    // Mag_x = Mag_x_SF*(Mag_x_raw - Mag_x_bias)
    int magxbias = 0, magybias = 0, magzbias = 0;
    int accxbias = 0, accybias = 0, acczbias = 0;
    int gyroxbias = 94, gyroybias = 3, gyrozbias = -58;
    int magxSF = 1, magySF = 1, magzSF = 1;
    int accxSF = 1, accySF = 1, acczSF = 1;
    int gyroxSF = 1, gyroySF = 1, gyrozSF = 1;
    std::vector<std::vector<double>> mag_A_inv = {{1.2535, -0.0207, 0.0725},{-0.0207, 1.2612, -0.0207},{0.0725, -0.0207, 1.2737}};
    std::vector<double> mag_bias = {-0.0724,-0.3814, 1.2595};

    double kx = 0.9957, ky = 0.9898, kz = 0.9666;
    double ayz = 0.01423, azy = 0.05225, azx = 0.00074;
    std::vector<std::vector<double>> acc_T = {{1, -ayz, azy},{0, 1, -azx},{0, 0, 1}};
    std::vector<std::vector<double>> acc_S = {{kx, 0, 0},{0, ky, 0},{0, 0, kz}};
    std::vector<double> acc_bias = {-0.9497,-1.5212, -1.5564};

    std::vector<double> gyro_bias = {0.0504, 0.0017, -0.0309};

  public:
    void initialize();
    void readsensor(int16_t imusensor[3][3]);
    void applyrange(int16_t imusensor[3][3], std::vector<double> &gyro, std::vector<double> &mag, std::vector<double> &acc);
    void applycalibration(std::vector<double> &gyro, std::vector<double> &acc, std::vector<double> &mag);
};

#endif