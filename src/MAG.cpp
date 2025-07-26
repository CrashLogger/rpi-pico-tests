#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "MAG.h"

// These values are for IMU based corrections!
#define R1 3
#define C1 3
#define R2 3
#define C2 1

#define CALIBRATION_X -12.467
#define CALIBRATION_Y -64.207
#define CALIBRATION_Z -102.78

#define SCALE_X 177.91
#define SCALE_Y 151.05
#define SCALE_Z 178.85

#define MOVE_X 0
#define MOVE_Y 0
#define MOVE_Z -0.5

MAG::~MAG(){
}
MAG::MAG(){
    config();
}
MAG::MAG(uint8_t i2c, uint8_t SCL, uint8_t SDA){
  //Initialising 
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_pull_up(SCL);
    gpio_pull_up(SDA);
  
    printf("Running config:\n");
    //i2c_init(i2c1, 400*1000);
    config();
  }


void MAG::config(){

    sleep_ms(100);
    i2c_write_blocking(i2c1, addr, config_a, 3, true);
    i2c_write_blocking(i2c1, addr, config_b, 3, true);
    i2c_write_blocking(i2c1, addr, mode_continuous, 3, false);
    sleep_ms(6);
    printf("Config done.\n");

}
uint8_t MAG::poll(){
    uint8_t ready[1] = {0};
    uint8_t readBytes = 0;
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};

    i2c_write_blocking(i2c1, addr, rdy_reg, 2, true); // true to keep master control of bus
    readBytes = i2c_read_blocking(i2c1, addr, ready, 1, false);
    //printf("POLL RESULT: %d Read %d bytes.\n", ready[0], readBytes);
    if(ready[0] != 0){

        i2c_write_blocking(i2c1, addr, start_read, 2, true); // true to keep master control of bus
        readBytes = i2c_read_blocking(i2c1, addr, data, 6, true);
        i2c_write_blocking(i2c1, addr, reset_read, 2, false);

        //LSBX, MSBX, LSBY, MSBY, LSBZ, MSBZ
        raw_x = int16_t(data[0]<<8| data[1]);
        raw_y = int16_t(data[2]<<8| data[3]);
        raw_z = int16_t(data[4]<<8| data[5]);


        //MAG_C_X = (MAG_X-mean(MAG_X))/(std(MAG_X)*3);
        norm_x = ((raw_x - CALIBRATION_X)/SCALE_X)+MOVE_X;
        norm_y = ((raw_y - CALIBRATION_Y)/SCALE_Y)+MOVE_Y;
        norm_z = ((raw_z - CALIBRATION_Z)/SCALE_Z)+MOVE_Z;


        //printf("X: %8d\tY: %8d\tZ: %8d\n", raw_x, raw_y,raw_z);
    }
    return(0);
}

double MAG::getHdg(){


    //printf("Magneto polled:\n");
    poll();
    double hdg = (atan(norm_y/norm_x))*(180/MATH_PI);
    //Assuming it is more-or-less horizontal:
    
    return(hdg);
}

double MAG::getRCHdg(double roll){
    //Heading but with roll correction... hopefully!
    poll();

    double m1[R1][C1] = {{ 0, 0, 1 }, {-sin(roll), cos(roll), 0}, {cos(roll), sin(roll), 0  } };
    double m2[R2][C2] = { {norm_x}, {norm_y}, {norm_z} };
    double result[R1][C2];

    for (int i = 0; i < R1; i++) {
        for (int j = 0; j < C2; j++) {
            result[i][j] = 0;

            for (int k = 0; k < R2; k++) {
                result[i][j] += m1[i][k] * m2[k][j];
            }

        }
    }

    double hdg = (atan(result[1][0]/result[0][0]))*(180/MATH_PI);
    return(hdg);

}

float MAG::getNormX(){
    return (MAG::norm_x);
}

float MAG::getNormY(){
    return (MAG::norm_y);
}

float MAG::getNormZ(){
    return (MAG::norm_z);
}


int16_t MAG::getRawX(){
    return (MAG::raw_x);
}

int16_t MAG::getRawY(){
    return (MAG::raw_y);
}

int16_t MAG::getRawZ(){
    return (MAG::raw_z);
}


