#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include <vector>
#include <iostream>
#include "lib/matrix_operations/matrix_operations.h"
#include "lib/IMU/IMU.h"
#include "lib/EKF/EKF.h"

// Define uart port instance
#define UART_PORT uart0

// Define a IMU-type object called imu
IMU imu;

EKF ekf;
Euler yawpitchroll;

// Global variables 
int16_t   imusensor[3][3];
char coordinates[3] = {'x', 'y', 'z'};
double    int16bit_range = 32767.5;

std::vector<double> gyro(3);
std::vector<double> acc(3);
std::vector<double> mag(3);
std::vector<double> cuat(4);



// Main loop timer
struct repeating_timer main_timer;  // Main loop timer
int main_loop_period = -50;          // Main loop period in ms
uint64_t time0_ml = time_us_64();
uint64_t time1_ml;
uint64_t dt_ml;

// Communication with PC

typedef union
{
  int16_t number;
  uint8_t bytes[2];
} INTUNION_t;




void sendToPC(int16_t data1, int16_t data2, int16_t data3,
              int16_t data4, int16_t data5, int16_t data6,
              int16_t data7, int16_t data8, int16_t data9)
{
  uint8_t buf[18];
  int16_t data[9] = {data1, data2, data3, data4, data5, data6, data7, data8, data9};
  for (int i = 0; i < 9; i++)
  {
    INTUNION_t myint;
    myint.number = data[i];
    for (int j = 0; j < 2; j++)
    {
      buf[i * 2 + j] = myint.bytes[j];
    }
  }

  if (uart_is_writable(UART_PORT))
  {
    uart_write_blocking(UART_PORT, buf, 18);
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
  time1_ml  = time_us_64();         // Get global time in microseconds
  dt_ml     = time1_ml - time0_ml;  // Get deltaT
  time0_ml  = time1_ml;             // Reassign time0

  double dt = dt_ml/1000000.;
  // printf("deltaT = %lld\t\n", dt_ml/1000);


  imu.readsensor(imusensor);
  imu.applyrange(imusensor, gyro, mag, acc);
  imu.applycalibration(gyro, acc, mag);

  ekf.predict(gyro, dt);
  ekf.update(acc, mag);
  


  int n = gyro.size();

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
  // Create a repeating timer for the main loop
  add_repeating_timer_ms(main_loop_period, main_loop, NULL, &main_timer);
  // bool cancelled = cancel_repeating_timer(&main_timer);


  while (true)
  {

  }
  return 0;
}
