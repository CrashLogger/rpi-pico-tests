/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include <stdio.h>
#include "ACCELEROMETER.h"
#include "GPS.h"
#include "MAG.h"
#include "string.h"
#include "ADS.h"

//GPS NEO6M

#define UART_ID uart1
#define BAUD_RATE 9600
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

#define UART_TX_PIN 4
#define UART_RX_PIN 5

static int chars_rxed = 0;

char prefix[6];
char sentence[128];

float latitude = 0.0f;
float longitude = 0.0f;

uint8_t clear_array(char* array, uint8_t size){
    for(int i = 0; i<size; i++){
        array[i] = 0;
    }
    return(0);
}

double coord_clean(char* raw_numeric, char direction){
    char loc_deg[4];
    double loc_final;

    uint8_t decimalPos = 0;
    //Just in case a different compiler doesn't clear assigned arrays.
    clear_array(loc_deg, 4);
    
    //Determining if it is a latitude or longitude value by locating the decimal point
    //We get DDMM.mmmm for latitude and DDDMM.mmmm for longitude
    //We could identify them by the direction character too...!
    decimalPos = (uint8_t)(strchr(raw_numeric, '.') - raw_numeric);
    memcpy(loc_deg, raw_numeric, decimalPos-2);
    loc_final = atoi(loc_deg)+(atof(&raw_numeric[decimalPos-2]))/60.0f;

    //N and E are both positive and before the letter O
    //S and W are both negative and after the letter O
    if(direction > 'O'){
        loc_final = loc_final - 2*loc_final;;
    }
    printf("Local coordinate:\t%4.4lf", loc_final);
    return(loc_final);
}

void parse_sentence(){
    strncpy(prefix, sentence + 1,5);
    //printf("%s\n",prefix);

    uint8_t gmt_hour;
    uint8_t gmt_minute;
    uint8_t gmt_second;

    float lat;
    float lon;
    
    //strcmp returns 0 when both are equal!
    if(!strcmp(prefix, "GNRMC")){
        printf("%s", sentence);
        char sentencePart[128];
        memcpy(sentencePart,&sentence[7],strlen(sentence)-6);
        
        char buffer[128];
        uint8_t field_counter = 0;

        for(int i = 0; i<strlen(sentence); i++){

            if(sentence[i]==','){

                if(field_counter==3){
                   latitude = coord_clean(buffer, sentence[i+1]);
                }
                else if (field_counter==5){
                    longitude = coord_clean(buffer, sentence[i+1]);
                }

                printf("\n");
                field_counter++;
                //Clearing the buffer between sentence sections
                memset(buffer, 0, sizeof buffer);
                printf("%d\t",field_counter);
            }
            else{
                buffer[strlen(buffer)] = sentence[i];
            }
        }

    }
    else if(!strcmp(prefix, "GPTXT")){
        //printf("Diagnostics sentence!");
    }
}


void on_uart_rx() {

    while (uart_is_readable(UART_ID)) {
        uint8_t ch = uart_getc(UART_ID);

        sentence[strlen(sentence)] = ch;
        if(ch == '\n'){
            parse_sentence();
            for(int i = 0; i<sizeof(sentence); i++){
                sentence[i] = 0;
            }            
        }
    }
}

//GPS NEO6M

uint32_t ms_since_boot = 0;
uint32_t ms_last_read = 0;
uint32_t ms_last_change = 0;
uint32_t ms_last_loc = 0;
uint32_t ms_last_hdg = 0;
uint32_t ms_last_print = 0;
uint32_t ms_last_joy = 0;

uint8_t led_state = 0;

struct location_data{
    float lat;
    float lon;
    float roll;
    float pitch;
    int16_t magX;
    int16_t magY;
    int16_t magZ;
};

struct joystick_data{
    uint16_t x0 = 0;
    uint16_t x1 = 0;
    uint16_t y0 = 0;
    uint16_t y1 = 0;
};

location_data locdata;
joystick_data joydata;

// Perform initialisation
int pico_led_init(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
}

uint8_t toggle(uint8_t var){
    if(var > 0){
        return 0;
    }
    else{
        return 1;
    }
}

// Turn the led on or off
void pico_set_led() {
    if( ms_since_boot - ms_last_change >= 1000){
        ms_last_change = ms_since_boot;
        led_state = toggle(led_state);
        gpio_put(PICO_DEFAULT_LED_PIN, led_state);
        
    }
}

void read_accel(ACCELEROMETER accel) {
    if( ms_since_boot - ms_last_read >= 50){
        ms_last_read = ms_since_boot;
        //printf("Roll:%3.3f\tPitch:%3.3f\t\tX: %-2.3f\tY: %-2.3f\tZ: %-2.3f\n",accel.getRoll(),accel.getPitch(), accel.getRawX(), accel.getRawY(), accel.getRawZ());
        locdata.roll = accel.getRoll();
        locdata.pitch = accel.getPitch();
    }
}

void read_gps(GPS gps, gps_data gdata) {
    if( ms_since_boot - ms_last_loc >= 50){
        ms_last_loc = ms_since_boot;
        gdata = gps.parse_string();
        locdata.lat = gdata.latitude;
        locdata.lon = gdata.longitude;
        //printf("LAT:\t%4.4f\tLON:\t%4.4f\t%s\n", gdata.latitude, gdata.longitude, gdata.time);
    }
}

uint8_t read_mag(MAG mag) {
    if( ms_since_boot - ms_last_hdg >= 100){
        ms_last_hdg = ms_since_boot;

        //printf("Mag check\n");
        float heading = mag.getHdg();
        locdata.magX = mag.getRawX();
        locdata.magY = mag.getRawY();
        locdata.magZ = mag.getRawZ();
        //printf("AAAAA\n");
    }
    return(0);
}

uint8_t read_joy(ADS ads) {
    if( ms_since_boot - ms_last_joy >= 100){
        sleep_ms(50);
        printf("Hello!\n");
        printf("We are calling the ADS to read some stuff for us.\n");
        ms_last_joy = ms_since_boot;
        joydata.x0 = ads.readChannel(1);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        //joydata.x1 = ads.readShortChannel(1);
        //joydata.y0 = ads.readShortChannel(2);
        //joydata.y1 = ads.readShortChannel(3);
        
        printf("%d, %d, %d, %d\n", joydata.x0, joydata.x1, joydata.y0, joydata.y1);

        //printf("AAAAA\n");
    }
    return(0);
}

uint8_t printReadings() {
    if( ms_since_boot - ms_last_print >= 500){
        ms_last_print = ms_since_boot;
        //printf("%4.4f\t%4.4f\t\t%4.4fº\t%4.4fº\t%1.4f\t%1.4f\t%1.4f\n", locdata.lat, locdata.lon, locdata.roll, locdata.pitch, locdata.magX, locdata.magY, locdata.magZ);
        printf("%d\t%d\t%d\n", locdata.magX, locdata.magY, locdata.magZ);
    }
    return(0);
}

int main() {
    stdio_init_all();
    sleep_ms(1000);
    printf("Hello world!");
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    printf("Starting IMU\n");
    ACCELEROMETER accel(1, 26, 27);
    MAG mag(1, 26, 27);
    sleep_ms(100);
    pico_set_led();
    //ADS ads(i2c1, 15, 14);
    
    //sleep_ms(1000);
    //printf("Starting GPS\n");
    //sleep_ms(1000);
    //GPS gps(1,26,27);
    //printf("GPS Started\n");
    //sleep_ms(1000);
    
    //gps_data gdata;


    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    //Uart parity, fifo, format... settings
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, false);

    //UART receiving causes interrupts
    int UART_IRQ = UART_ID == uart1 ? UART1_IRQ : UART0_IRQ;
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    //Emptying out the sentence to make strlen work
    for(int i = 0; i<sizeof(sentence); i++){
        sentence[i] = 0;
    }


    while (true) {
        ms_since_boot = to_ms_since_boot(get_absolute_time());
        
        //read_accel(accel);
        //printf("Accel read!");
        //read_mag(mag);
        //read_gps(gps, gdata);
        //read_joy(ads);
        pico_set_led();
        //printReadings();

    }
}
