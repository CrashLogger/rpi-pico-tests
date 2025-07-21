#include <stdio.h>
#include <string.h>

#ifndef __GPSPARSE_H_
#define __GPSPARSE_H_
// This code is just parsing what is received from other sources.
// It should be compatible with both UART based and I2C based GPS modules
// That said, it was written and tested on the ublox NEO-6M-0-001 type over UART

class GPSPARSE{
    
    private:
    //VARS


    //FUNCS

    public:
    //VARS
        float latitude;
        float longitude;

    //FUNCS
        uint8_t parse_location(char* sentence);

};

#endif