#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "lib/IMU/IMU.h"
#include "hardware/uart.h"



// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// Define uart port instance
#define UART_PORT uart0

IMU imu;


void initialize(){
    stdio_init_all();

    // Initialize UART PORT
    uart_init(UART_PORT, 115200);


    printf("\n########################################################\n");
    printf("#################### Initialization ####################\n");
    printf("########################################################\n\n");
    // SPI initialisation. This example will use SPI at 1MHz.
    int spifreq = 1000000;
    spi_init(SPI_PORT, spifreq);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    printf("#################  SPI port initialized ################\n");
    printf("\nSPI frequency %i Hz \n", spifreq);
    printf("MISO\tpin:\t%i \n", PIN_MISO);
    printf("CS\tpin:\t%i \n", PIN_CS);
    printf("SCK\tpin:\t%i \n", PIN_SCK);
    printf("MOSI\tpin:\t%i \n", PIN_MOSI);
}


int16_t       imusensor[3][3];

typedef union{
 int16_t number;
 uint8_t bytes[2];
} INTUNION_t;



void sendToPC(int16_t data1, int16_t data2, int16_t data3, 
              int16_t data4, int16_t data5, int16_t data6,
              int16_t data7, int16_t data8, int16_t data9)
{
  uint8_t buf[18];
  int16_t data[9] = {data1, data2, data3, data4, data5, data6, data7, data8, data9};
  //int16_t data[9] = {0,0,0,0,0,0,0,0,0};
  for (int i = 0; i < 9; i++) {
    INTUNION_t myint;
    myint.number = data[i];
    for (int j = 0; j < 2; j++) {
      buf[i*2 + j] = myint.bytes[j];
      //buf[i*2 + j] = 0;
    }
  }

  //buf[0] = 5;
  if (uart_is_writable(UART_PORT)){
    uart_write_blocking(UART_PORT, buf, 18);
  }
  
  
}


#define PARITY    UART_PARITY_NONE

int main()
{
       
    initialize();
    imu.initialize();
    uart_init(UART_PORT, 115200);
    //uart_puts(UART_PORT, "\nUART port is working\n");
    uart_set_format(UART_PORT, 8, 1, PARITY);
    uart_set_hw_flow(UART_PORT, 0,0);
     uart_set_irq_enables(UART_PORT, true, false);
      uart_set_translate_crlf(UART_PORT, true);
      uart_set_fifo_enabled(UART_PORT, true);
    //printf("\nIs uart writable?\t%i", uart_is_writable(UART_PORT));


    // i2c_inst_t *i2c = I2C_PORT;
    // int16_t data[9];

    long time0 = to_us_since_boot(get_absolute_time());
    long time1;
    long dt;

double int16bit_range = 32767.5;

    

    while(true){
        time1 = to_us_since_boot(get_absolute_time());
        dt = time1 - time0;
        time0 = time1;
        imu.readsensor(imusensor);
        sendToPC(   imusensor[1][0],  imusensor[1][1], imusensor[1][2],
                    imusensor[0][0],  imusensor[0][1], imusensor[0][2],
                    imusensor[2][0],  imusensor[2][1], imusensor[2][2]);

        double accx = imusensor[0][0]/int16bit_range*4.0*9.81;
        double accy = imusensor[0][1]/int16bit_range*4.0*9.81;
        double accz = imusensor[0][2]/int16bit_range*4.0*9.81;

        double gyrox = imusensor[1][0]/int16bit_range*1000;
        double gyroy = imusensor[1][1]/int16bit_range*1000;
        double gyroz = imusensor[1][2]/int16bit_range*1000;


        
        
        // printf("Accx =\t%.2f\tAccy =\t%.2f\tAccz =\t%.2f\tGyrox =\t%.2f\tGyroy =\t%.2f\tGyroz =\t%.2f\tMagx =\t%i\tMagy =\t%i\tMagz =\t%i\n", 
        //   accx, accy, accz,
        //   gyrox, gyroy, gyroz,
        //   imusensor[2][0], imusensor[2][1], imusensor[2][2]);

        // printf("Accx =\t%i\tAccy =\t%i\tAccz =\t%i\tGyrox =\t%i\tGyroy =\t%i\tGyroz =\t%i\tMagx =\t%i\tMagy =\t%i\tMagz =\t%i\n", 
        //           imusensor[0][0], imusensor[0][1], imusensor[0][2],
        //           imusensor[1][0], imusensor[1][1], imusensor[1][2],
        //           imusensor[2][0], imusensor[2][1], imusensor[2][2]);
        sleep_ms(50);
    }

    return 0;
}
