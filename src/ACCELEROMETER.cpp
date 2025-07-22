#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "ACCELEROMETER.h"


ACCELEROMETER::~ACCELEROMETER(){
}
ACCELEROMETER::ACCELEROMETER(){
    config();
}
ACCELEROMETER::ACCELEROMETER(uint8_t i2c, uint8_t SCL, uint8_t SDA){
  //Initialising 
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_pull_up(SCL);
    gpio_pull_up(SDA);
  
    i2c_init(i2c1, 100*1000);
    config();
  }


void ACCELEROMETER::config(){

    i2c_write_blocking(i2c1, addr, rst_1, 2, false);
    sleep_ms(200);                                          //Precautionary, works with it, haven't tried it without it
    i2c_write_blocking(i2c1, addr, rst_2, 2, false);
    sleep_ms(200);
    i2c_write_blocking(i2c1, addr, configuration, 2, false);
    sleep_ms(200);
    i2c_write_blocking(i2c1, addr, gyro_conf, 2, false);
    sleep_ms(200);
    i2c_write_blocking(i2c1, addr, accel_conf, 2, false);
    sleep_ms(200);
}
void ACCELEROMETER::update(){
    int16_t accel[3]; int16_t gyro[3];
    uint8_t buffer[6];


    ACCELEROMETER::accel_reg = 0x3B;
    i2c_write_blocking(i2c1, addr, &(ACCELEROMETER::accel_reg), 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c1, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    ACCELEROMETER::AccX=(float)accel[0]/4096-0.02;
    ACCELEROMETER::AccY=(float)accel[1]/4096;
    ACCELEROMETER::AccZ=(float)accel[2]/4096+0.04; 


    ACCELEROMETER::gyro_reg = 0x43;
    i2c_write_blocking(i2c1, addr, &(ACCELEROMETER::gyro_reg), 1, true);
    i2c_read_blocking(i2c1, addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }
    //Based on specsheet
    ACCELEROMETER::RollRate=(float)gyro[0]/65.5;
    ACCELEROMETER::PitchRate=(float)gyro[1]/65.5;
    ACCELEROMETER::YawRate=(float)gyro[2]/65.5;


    //Basic trigonometric functions for readability
    //RollAngle=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(M_PI/180);
    //PitchAngle=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(M_PI/180);

    ACCELEROMETER::RollAngle=atan( ACCELEROMETER::AccY/sqrt( ACCELEROMETER::AccX* ACCELEROMETER::AccX+ ACCELEROMETER::AccZ* ACCELEROMETER::AccZ))*1/(M_PI/180);
    ACCELEROMETER::PitchAngle=-atan( ACCELEROMETER::AccX/sqrt( ACCELEROMETER::AccY* ACCELEROMETER::AccY+ ACCELEROMETER::AccZ* ACCELEROMETER::AccZ))*1/(M_PI/180);
}

float ACCELEROMETER::getPitch(){
    update();
    return ACCELEROMETER::PitchAngle;
}

float ACCELEROMETER::getRoll(){
    update();
    if(AccY < 0 && AccZ < 0){
        return 180 - abs(ACCELEROMETER::RollAngle);
    }
    else if(AccY >= 0 && AccZ < 0){
        return -180 +abs(ACCELEROMETER::RollAngle);
    }
    else if(AccY >= 0 && AccZ > 0){
        return -abs(ACCELEROMETER::RollAngle);
    }
    else{
        return abs(ACCELEROMETER::RollAngle);
    }
    
}

float ACCELEROMETER::getRollUD(){
    update();
    if(AccY >= 0 && AccZ > 0){
        return 180 - abs(ACCELEROMETER::RollAngle);
    }
    else if(AccY >= 0 && AccZ < 0){
        return abs(ACCELEROMETER::RollAngle);
    }
    else if(AccY < 0 && AccZ < 0){
        return -abs(ACCELEROMETER::RollAngle);
    }
    else{
        return -180 +abs(ACCELEROMETER::RollAngle);
    }
    
}

float ACCELEROMETER::getRawX(){
    return ACCELEROMETER::AccX;
}

float ACCELEROMETER::getRawY(){
    return ACCELEROMETER::AccY;
}

float ACCELEROMETER::getRawZ(){
    return ACCELEROMETER::AccZ;
}