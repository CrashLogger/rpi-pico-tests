#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#define MATH_PI 3.141592653

#ifndef __MAG_H_
#define __MAG_H_

class MAG{

    private:
      //VARS
        
        const uint8_t addr=0x1E;

        uint8_t config_reg=0x00;
        // POSSIBLE CONFIGS:
        /*
            STANDBY: 0x00
            CONSTANT READING: 0x01
        */
        //uint8_t temp_reg_0 = 0x07;
        //uint8_t temp_reg_1 = 0x08;
        //uint8_t reset_reg = 0x0A;
        //uint8_t set_reset_reg = 0x0B;

        const uint8_t rdy_reg[1] = {0x09};
        const uint8_t start_read[1] = {0x06};
        const uint8_t reset_read[1] = {0x03};
        
        uint8_t gyro_reg=0x43;

        const uint8_t config_a[2]={0x00, 0x70}; //8-average, 15Hz, normal measurement
        const uint8_t config_b[2]={0x01, 0xA0}; //Gain = 5
        const uint8_t mode_continuous[2]={0x02, 0x00};

        // Calibration step 1: Centering the results to 0,0,0
        const float calibration_x = -33.5;
        const float calibration_y = 51.5;
        const float calibration_z = -217;
        
        // Calibration step 2: Assigning the maximum +- values from the center, to normalise values
        const float max_calibrated_x = 148.5;
        const float max_calibrated_y = 171.5;
        const float max_calibrated_z = 154.0;

        // Calibration step 2: Assigning the maximum +- values from the center, to normalise values
        const float std_calibrated_x = 81.321;
        const float std_calibrated_y = 88.885;
        const float std_calibrated_z = 78.733;

        //3 Axis raw readings from the sensor
        int16_t raw_x = 0;
        int16_t raw_y = 0;
        int16_t raw_z = 0;

        //Normalised 3 axis readings from the sensor
        float norm_x = 0;
        float norm_y = 0;
        float norm_z = 0;


        //FUNCS
        void config();
        uint8_t poll();

    public:
        //VARS

        //FUNCS
        double getHdg();
        double getRCHdg(double roll);

        int16_t getRawX();
        int16_t getRawY();
        int16_t getRawZ();

        float getNormX();
        float getNormY();
        float getNormZ();

    public:
    MAG(uint8_t i2c, uint8_t SCL, uint8_t SDA);
    MAG();
    ~MAG();

};



#endif
