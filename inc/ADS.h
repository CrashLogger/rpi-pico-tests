//v1.1
//2024-08-05

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#ifndef __ADS_H_
#define __ADS_H_

#define I2C_ADDR 0x48

class ADS{

    private:
        //VARS
        
        //Hard-coded commands. 7 bit i2c address R/!W bit.
        const uint8_t read_from_address = 0b10010001;
        const uint8_t write_to_address = 0b10010000;

        //Hard-coded pointers
        const uint8_t config_ptr = 0x01;
        const uint8_t conver_ptr = 0x00;
        
        //Object-wide i2c_port pointer
        i2c_inst_t* i2c_port;

        //FUNCS
        void ads_init(i2c_inst_t *i2c_port, uint8_t resol);
        void ads_set_mux(i2c_inst_t* i2c_port, uint8_t channel);
        void ads_set_pga(i2c_inst_t * i2c_port, uint8_t resol);
        void ads_set_mode(i2c_inst_t* i2c_port, uint8_t mode);
        void ads_trig_read(i2c_inst_t* i2c_port);
        void ads_set_speed(i2c_inst_t* i2c_port, uint8_t speed);
        void ads_write_config(i2c_inst_t* i2c_port, uint16_t config);
        
        uint8_t convertAngle(uint16_t receivedPos);
        
        uint16_t ads_read_config(i2c_inst_t *i2c_port);
        uint16_t ads_read_channel(i2c_inst_t* i2c_port, uint8_t channel);

    public:
        //VARS
        
        //FUNCS
        uint16_t readChannel(uint8_t channel);
        uint8_t readShortChannel(uint8_t channel);

        ADS(i2c_inst_t* i2c_port, uint8_t SCL, uint8_t SDA);
        ~ADS();

};

#endif
