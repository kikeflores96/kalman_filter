#include "IMU.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t data) {
    // Append register address to front of data packet
    uint8_t msg[2];
    msg[0] = reg;
    msg[1] = data;
    // Write data to register(s) over I2C
    i2c_write_blocking(i2c, addr, msg, 2, false);
}
// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(i2c_inst_t *i2c, const uint addr, const uint8_t reg, const uint8_t nbytes, uint8_t *buf) {
    // Read data from register(s) over I2C
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    i2c_read_blocking(i2c, addr, buf, nbytes, false);
}

void IMU :: initialize(){
    printf("\n\n ---------------- Initializing MPU9250 ------------\n\n");
    printf("Using i2c protocol to communicate with mpu9259\n");
    // I2C  portInitialisation. Using it at 400Khz.
    int i2cfreq = 400000;
    i2c_init(I2C_PORT, i2cfreq);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    printf("\n#################  I2C port initialized ################\n");
    printf("\nI2C frequency %i Hz \n", i2cfreq);
    printf("SDA\tpin:\t%i \n", I2C_SDA);
    printf("SCL\tpin:\t%i \n", I2C_SCL);

    printf("\n--------------------------------------------------------\n");
    printf("MPU9250 settings:\n");
    // MPU9259 initialization
    // Set accelerometers low pass filter at 5Hz
    printf("Accelerometer lowpass filter cutoff frequency = 5Hz\n");
    I2CwriteByte(I2C_PORT, MPU9250_ADDRESS,29, 0x06);
    // Set gyroscope low pass filter at 5Hz
    printf("Gyroscope lowpass filter cutoff frequency = 5Hz\n");
    I2CwriteByte(I2C_PORT, MPU9250_ADDRESS,26, 0x06);
    // Configure gyroscope range
    printf("Gyroscope range set to 1000 deg/s\n");
    I2CwriteByte(I2C_PORT, MPU9250_ADDRESS,27, GYRO_FULL_SCALE_1000_DPS);
    // Configure accelerometers range
    printf("Accelerometer range set to 4g = 4 x 9.81 m/s2\n");
    I2CwriteByte(I2C_PORT, MPU9250_ADDRESS,28, ACC_FULL_SCALE_4_G);
    printf("Configuring magnetometer ...\n");
    // Set by pass mode for the magnetometers
    I2CwriteByte(I2C_PORT, MPU9250_ADDRESS,0x37, 0x02);
    // Request continuous magnetometer measurements in 16 bits
    I2CwriteByte(I2C_PORT, MAG_ADDRESS,0x0A, 0x16);
    printf("Inertial Measurement Unit was succesfully configured\n");
    printf("\n--------------------------------------------------------\n");
  
}



void IMU :: applycalibration(int16_t imusensor[3][3]){
  imusensor[0][0] = accxSF*(imusensor[0][0] - accxbias);
  imusensor[0][1] = -accySF*(imusensor[0][1] - accybias);
  imusensor[0][2] = acczSF*(imusensor[0][2] - acczbias);

  imusensor[1][0] =  -gyroxSF*(imusensor[1][0] - gyroxbias);
  imusensor[1][1] =  gyroySF*(imusensor[1][1] - gyroybias);
  imusensor[1][2] =  -gyrozSF*(imusensor[1][2] - gyrozbias);

  imusensor[2][0] = -magxSF*(imusensor[2][0] - magxbias);
  imusensor[2][1] = magySF*(imusensor[2][1] - magybias);
  imusensor[2][2] = -magzSF*(imusensor[2][2] - magzbias);
  
}



void IMU :: readsensor(int16_t imusensor[3][3]){
    uint8_t Buf[14];
    I2Cread(I2C_PORT, MPU9250_ADDRESS,0x3B, 14, Buf);
    // Create 16 bits values from 8 bits data
    // Accelerometer
    imusensor[0][0]=-(Buf[0]<<8 | Buf[1]);
    imusensor[0][1]=-(Buf[2]<<8 | Buf[3]);
    imusensor[0][2]=Buf[4]<<8 | Buf[5];

    // Gyroscope
    imusensor[1][0]=-(Buf[8]<<8 | Buf[9]);
    imusensor[1][1]=-(Buf[10]<<8 | Buf[11]);
    imusensor[1][2]=Buf[12]<<8 | Buf[13];

    // Magnetometer
    uint8_t Mag[7];  
    I2Cread(I2C_PORT, MAG_ADDRESS,0x03, 7, Mag);
    // Create 16 bits values from 8 bits data
    // Magnetometer
    imusensor[2][0]=-(Mag[3]<<8 | Mag[2]);
    imusensor[2][1]=-(Mag[1]<<8 | Mag[0]);
    imusensor[2][2]=-(Mag[5]<<8 | Mag[4]);
    //applycalibration(imusensor);

  
}