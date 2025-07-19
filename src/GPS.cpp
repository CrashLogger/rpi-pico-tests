#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "string.h"
#include "GPS.h"

GPS::~GPS(){

}
GPS::GPS(){
    init();
}
GPS::GPS(uint8_t i2c, uint8_t SCL, uint8_t SDA){

    i2c_init(i2c0, 400*1000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);
    
    init();
}

void GPS::init(){
    char init_command[] = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
    printf("Initialising GPS module...\n");
    write_commmand(init_command,sizeof(init_command));

}
void GPS::write_commmand(char command[], int command_length){
    uint8_t command_in_byte;

    for(int i=0;i<command_length;i++){
        command_in_byte = command[i];
        i2c_write_blocking(i2c0,addr,&command_in_byte,1,true);
    }
}
void GPS::read(){
    uint8_t buffer[max_read];
 
    int i = 0;
    bool complete = false;

    i2c_read_blocking(i2c_default, addr, buffer, max_read, false);

    // Convert bytes to characters
    while (i < max_read && complete == false) {
        GPS::buffer[i] = buffer[i];
        // Stop converting at end of message 
        if (buffer[i] == 10 && buffer[i + 1] == 10) {
            complete = true;
        }
        i++;
    }
}
struct gps_data GPS::parse_string(){
    read();
    char protocol[]="GNRMC";
    // Finds location of protocol message in output
    char *com_index = strstr(buffer, protocol);
    int p = com_index - buffer;
 
    // Splits components of output sentence into array
    #define NO_OF_FIELDS 14
    #define MAX_LEN 15
 
    int n = 0;
    int m = 0;
 
    char gps_data[NO_OF_FIELDS][MAX_LEN];
    memset(gps_data, 0, sizeof(gps_data));
 
    bool complete = false;
    while (buffer[p] != '$' && n < MAX_LEN && complete == false) {
        if (buffer[p] == ',' || buffer[p] == '*') {
            n += 1;
            m = 0;
        } else {
            gps_data[n][m] = buffer[p];
            // Checks if sentence is complete
            if (m < NO_OF_FIELDS) {
                m++;
            } else {
                complete = true;
            }
        }
        p++;
    }
    
    struct gps_data gdata;
      

    sprintf(gdata.protocol,"%s",gps_data[0]);
    sprintf(gdata.time,"%s", gps_data[1]);
    gdata.status = gps_data[2][0];
    char lat_deg[3], lat_min[8], lon_deg[4], lon_min[8];
    
    lat_deg[0] = gps_data[3][0];
    lat_deg[1] = gps_data[3][1];
    lat_deg[2] = '\0';

    lat_min[0] = gps_data[3][2];
    lat_min[1] = gps_data[3][3];
    lat_min[2] = gps_data[3][4];
    lat_min[3] = gps_data[3][5];
    lat_min[4] = gps_data[3][6];
    lat_min[5] = gps_data[3][7];
    lat_min[6] = gps_data[3][8];
    lat_min[7] = '\0';

    lon_deg[0] = gps_data[5][0];
    lon_deg[1] = gps_data[5][1];
    lon_deg[2] = gps_data[5][2];    
    lon_deg[3] = '\0';

    lon_min[0] = gps_data[5][3];
    lon_min[1] = gps_data[5][4];
    lon_min[2] = gps_data[5][5];
    lon_min[3] = gps_data[5][6];
    lon_min[4] = gps_data[5][7];
    lon_min[5] = gps_data[5][8];
    lon_min[6] = gps_data[5][9];
    lon_min[7] = '\0';

    double temp_lat_deg = atof (lat_deg);
    double temp_lat_min = atof (lat_min);
    gdata.latitude = temp_lat_deg + (temp_lat_min/60);
    gdata.NSIndicator = gps_data[4][0];
    double temp_lon_deg = atof (lon_deg);
    double temp_lon_min = atof (lon_min);
    gdata.longitude = temp_lon_deg + (temp_lon_min/60);
    gdata.EWIndicator = gps_data[6][0];
    gdata.speed = atof(gps_data[7]);
    gdata.course = atof(gps_data[8]);
    sprintf(gdata.date,"%s", gps_data[9]);

    //printf("---- GPSLIB ---- LAT: %4.4f, LON: %4.4f\n");

    return gdata;
}
