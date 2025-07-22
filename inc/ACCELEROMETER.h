

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"


#ifndef __ACCELEROMETER_H_
#define __ACCELEROMETER_H_

class ACCELEROMETER{

    private:
        //VARS
        float AccX,AccY,AccZ;
        float RollRate,PitchRate,YawRate;
        double RollAngle,PitchAngle,YawAngle; //Unable to calculate Yaw precisely without a compass or magnetometer
        const uint8_t addr=0x68;
        uint8_t accel_reg=0x3B;
        uint8_t gyro_reg=0x43;



        const uint8_t rst_1[2]={0x6B, 0x80}; //2-step reset to avoid starting in sleep mode
        const uint8_t rst_2[2]={0x6B, 0x00};

        const uint8_t configuration[2]= {0x1A, 0x05};
        const uint8_t gyro_conf[2]= {0x1B, 0x8};             //Various configurations {Register number, Value}
        const uint8_t accel_conf[2]= {0x1C, 0x10};

        //FUNCS
        void update();
        void config();

    public:
        //VARS

        //FUNCS
        float getPitch();
        float getRoll();
        float getRollUD();

        float getRawX();
        float getRawY();
        float getRawZ();

    public:
    ACCELEROMETER(uint8_t i2c, uint8_t SCL, uint8_t SDA);
    ACCELEROMETER();
    ~ACCELEROMETER();
};



#endif