#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include <vector>
#include <iostream>
#include "lib/matrix_operations/matrix_operations.h"
#include "lib/IMU/IMU.h"
#include "lib/EKF/EKF.h"
#include "lib/TCF/TCF.h"

// Define uart port instance
#define UART_PORT uart0

// Define a IMU-type object called imu
IMU imu;

TCF tcf;
EKF ekf;
Euler yawpitchroll;

// Global variables 
int16_t   imusensor[3][3];
char coordinates[3] = {'x', 'y', 'z'};
double    int16bit_range = 32767.5;


// IMU measurements
std::vector<double> gyro(3);    // Gyroscope p,q,r in rad/s
std::vector<double> acc(3);     // Acceleration gx, gy, gz, in m/s^2
std::vector<double> mag(3);     // Magnetic field in mT

Euler euler_ekf;
Euler euler_tcf;




// Main loop timer
struct repeating_timer main_timer;  // Main loop timer
int main_loop_period = -60;          // Main loop period in ms
uint64_t time0_us = time_us_64();
uint64_t time1_us;
uint64_t dt_us;

// Communication with PC

typedef union
{
  int16_t number;
  uint8_t bytes[2];
} INTUNION_t;


typedef union
{
  double number;
  uint8_t bytes[8];
} DOUBLEUNION_t;


// void sendToPC(int16_t data1, int16_t data2, int16_t data3,
//               int16_t data4, int16_t data5, int16_t data6,
//               int16_t data7, int16_t data8, int16_t data9)
// {
//   uint8_t buf[18];
//   int16_t data[9] = {data1, data2, data3, data4, data5, data6, data7, data8, data9};
//   for (int i = 0; i < 9; i++)
//   {
//     INTUNION_t myint;
//     myint.number = data[i];
//     for (int j = 0; j < 2; j++)
//     {
//       buf[i * 2 + j] = myint.bytes[j];
//     }
//   }

//   if (uart_is_writable(UART_PORT))
//   {
//     uart_write_blocking(UART_PORT, buf, 18);
//   }
// }


// void splitDoubleToBuffer(double value, uint8_t* buffer) {
//     uint64_t intValue = *reinterpret_cast<uint64_t*>(&value);
//     for (int i = 0; i < 8; ++i) {
//         buffer[i] = static_cast<uint8_t>(intValue >> (8 * i));
//     }
// }

void splitDoubleToBuffer(double value, uint8_t* buffer) {
    uint64_t* ptr = reinterpret_cast<uint64_t*>(&value);
    for (int i = 0; i < sizeof(double); i++) {
        buffer[i] = static_cast<uint8_t>(*ptr & 0xFF);
        *ptr >>= 8;
    }
}


void sendToPC(std::vector<double> &data)
{
  int n=data.size();
  uint8_t buf[n*8];

  for (int i = 0; i < n; i++){
    uint8_t element_buffer[8];
    // splitDoubleToBuffer(data[i], element_buffer); 
    DOUBLEUNION_t mydouble;
    mydouble.number = data[i];


    for (int j = 0; j < 8; j++)
    {
      buf[i * 8 + j] = mydouble.bytes[j];
    }
  }
  // printf("1=%u\t\t2=%u\t\t3=%u\t\t4=%u\n", buf[0],buf[1],buf[2],buf[3]);
  if (uart_is_writable(UART_PORT))
  {
    uart_write_blocking(UART_PORT, buf, n*8);
  }
}




// Initialization

void initialize()
{
  // Standard Input Output Initialization
  stdio_init_all();
  // Initialize UART PORT
  uart_init(UART_PORT, 115200);

}







bool main_loop(struct repeating_timer *t) {
  // Compute main loop period
  time1_us  = time_us_64();         // Get global time in microseconds
  dt_us     = time1_us - time0_us;  // Get deltaT
  time0_us  = time1_us;             // Reassign time0

  double dt = -double(main_loop_period)/1000;


  imu.readsensor(imusensor);
  imu.applyrange(imusensor, gyro, mag, acc);
  imu.applycalibration(gyro, acc, mag);

  // printf("\nw1[0] = %f\tw1[1] = %f\tw1[2] = %f", mag[0],mag[1],mag[2]);

  ekf.predict(gyro, dt);
  ekf.update(acc, mag);
  tcf.TRIAD(acc, mag);
  tcf.integrate(gyro, dt);
  tcf.combine();

  // printf("\nqhat[0] = %f\tqhat[1] = %f\tqhat[2] = %f\tqhat[3] = %f\t", ekf.q[0],ekf.q[1],ekf.q[2],ekf.q[3]);
  
  euler_ekf.getEulerangles(ekf.q);
  euler_tcf.getEulerangles(tcf.qHat);


  std::vector<double> data = {euler_ekf.roll, euler_ekf.pitch, euler_ekf.yaw, euler_tcf.roll, euler_tcf.pitch, euler_tcf.yaw};

  // std::vector<double> data = concatenateVectors(concatenateVectors(concatenateVectors(gyro, acc), mag), ekf.q);

  // std::vector<double> data = concatenateVectors(concatenateVectors(concatenateVectors(gyro, acc), mag), ekf.q);
  // for(int i=0;i<data.size();i++){
  //   printf("data[%i]=%.3f\t", i, data[i]);
  // }
  // printf("\n");
  sendToPC(data);

  // printf("\nyaw = %.3f\tpitch = %.3f\troll = %.3f", euler.yaw_deg(),euler.pitch_deg(),euler.roll_deg());
  // printf("\nyaw = %.3f\tpitch = %.3f\troll = %.3f", tcf.triad_yawpitchroll.yaw_deg(),tcf.triad_yawpitchroll.pitch_deg(),tcf.triad_yawpitchroll.roll_deg());
  // int n = gyro.size();

  // for(int i=0; i<n; i++){
  //       printf("Acc%c = %.3f\t", coordinates[i], acc[i]);
  // }
  // for(int i=0; i<n; i++){
  //       printf("Gyro%c = %.3f\t", coordinates[i], gyro[i]);
  // }
  // for(int i=0; i<n; i++){
  //       printf("Mag%c = %.3f\t", coordinates[i], mag[i]);
  // }
  // printf("\n");
  // sendToPC(imusensor[1][0], imusensor[1][1], imusensor[1][2],
  //           imusensor[0][0], imusensor[0][1], imusensor[0][2],
  //           imusensor[2][0], imusensor[2][1], imusensor[2][2]);

  // double accx = imusensor[0][0] / int16bit_range * 4.0 * 9.81;
  // double accy = imusensor[0][1] / int16bit_range * 4.0 * 9.81;
  // double accz = imusensor[0][2] / int16bit_range * 4.0 * 9.81;

  // double gyrox = imusensor[1][0] / int16bit_range * 1000;
  // double gyroy = imusensor[1][1] / int16bit_range * 1000;
  // double gyroz = imusensor[1][2] / int16bit_range * 1000;

  // printf("Accx =\t%.2f\tAccy =\t%.2f\tAccz =\t%.2f\tGyrox =\t%.2f\tGyroy =\t%.2f\tGyroz =\t%.2f\tMagx =\t%i\tMagy =\t%i\tMagz =\t%i\n",
  //   acc[0], acc[1], acc[2],
  //   gyro[0], gyro[1], gyro[2],
  //   mag[0], mag[1], mag[2]);

  // printf("Accx =\t%i\tAccy =\t%i\tAccz =\t%i\tGyrox =\t%i\tGyroy =\t%i\tGyroz =\t%i\tMagx =\t%i\tMagy =\t%i\tMagz =\t%i\n",
  //           imusensor[0][0], imusensor[0][1], imusensor[0][2],
  //           imusensor[1][0], imusensor[1][1], imusensor[1][2],
  //           imusensor[2][0], imusensor[2][1], imusensor[2][2]);
  
  
  // uint64_t time2_ml  = time_us_64();         // Get global time in microseconds
  // uint64_t dt2_ml     = time2_ml - time1_ml;  // Get deltaT
  // printf("looptime = %lld\t\n", dt2_ml/1000);
  return true;
}



int main()
{
  // Initializa hardware
  initialize();
  // Initialize IMU
  imu.initialize();

  tcf.initialize(imu);

  // double dt = -double(main_loop_period)/1000;


  // imu.readsensor(imusensor);
  // imu.applyrange(imusensor, gyro, mag, acc);
  // imu.applycalibration(gyro, acc, mag);

  // tcf.integrate(gyro, dt);
  
  // Create a repeating timer for the main loop
  add_repeating_timer_ms(main_loop_period, main_loop, NULL, &main_timer);
  // bool cancelled = cancel_repeating_timer(&main_timer);

  // std::vector<double> w = {7.98753855e-04, -2.49584207e-03 , 1.81871684e-03};
  // std::vector<double> a = {2.80537553e+00, 5.84316071e-02,  9.78944766e+00};
  // std::vector<double> m = {-2.79830467e+01,  1.23371929e+01, -2.82501149e+01};

  // double dt = -double(main_loop_period)/1000;

  // ekf.predict(w, dt);
  // ekf.update(a, m);

  while (true)
  {

  }
  return 0;
}
