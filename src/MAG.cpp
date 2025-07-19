#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "MAG.h"

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

        norm_x = (raw_x - calibration_x)/std_calibrated_x;
        norm_y = (raw_y - calibration_y)/std_calibrated_y;
        norm_z = (raw_z - calibration_z)/std_calibrated_z;


        //printf("X: %8d\tY: %8d\tZ: %8d\n", raw_x, raw_y,raw_z);
    }
    return(0);
}

float MAG::getHdg(){

    //printf("Magneto polled:\n");
    poll();
    //printf("Cocking nora\n");
    return(0.00);
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


