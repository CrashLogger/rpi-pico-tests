#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "string.h"

#ifndef __GPS_H_
#define __GPS_H_

struct gps_data{
    char protocol[15];
    char time[15];
    char status;
    double latitude;
    char NSIndicator;
    double longitude;
    char EWIndicator;
    double speed;
    double course;
    char date[15]; //only uses 6 characters, one by one
    char magneticVar[15];
    char EWdegreeIndicator[15];
    char mode[15];
    char checksum[15];
};

class GPS{

    private:
    //VARS
        const uint8_t addr = 0x10;
        const int max_read = 250;
        char buffer[250];
        

    //FUNCS
        void init();
        void write_commmand(char command[], int command_length);
        void read();
        
    public:
    //VARS


    //FUNCS
        GPS(uint8_t i2c, uint8_t SCL, uint8_t SDA);
        GPS();
        ~GPS();
        struct gps_data parse_string();

};


#endif