#ifndef IMU_H_
#define IMU_H_

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

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
    int magxbias = -136, magybias = 12, magzbias = -44;
    int accxbias = 0, accybias = 0, acczbias = 0;
    int gyroxbias = -40, gyroybias = -20, gyrozbias = 24;
    int magxSF = 52, magySF = 50, magzSF = 46;
    int accxSF = 1, accySF = 1, acczSF = 1;
    int gyroxSF = 1, gyroySF = 1, gyrozSF = 1;
  public:
    void initialize();
    void readsensor(int16_t imusensor[3][3]);
    void applycalibration(int16_t imusensor[3][3]);
};

#endif