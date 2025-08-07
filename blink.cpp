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

#define TAIL_LIGHT 13
#define STARBOARD_LIGHT 12
#define PORT_LIGHT 11
#define STROBES 15

//GPS NEO6M

#define UART_ID uart1
#define TAP_UART_ID uart0
#define BAUD_RATE 9600
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

#define UART_TX_PIN 4
#define UART_RX_PIN 5

static int chars_rxed = 0;

uint32_t ms_since_boot = 0;
uint32_t ms_last_read = 0;
uint32_t ms_last_change = 0;
uint32_t ms_last_loc = 0;
uint32_t ms_last_hdg = 0;
uint32_t ms_last_print = 0;
uint32_t ms_last_tap = 0;
uint32_t ms_last_joy = 0;
uint32_t ms_strobe = 0;
uint32_t ms_last_rx_poll = 0;

uint8_t led_state = 0;

struct location_data{
    double lat;
    double lon;
    double roll;
    double pitch;
    double heading;
    double magX;
    double magY;
    double magZ;
};

struct joystick_data{
    uint16_t x0 = 0;
    uint16_t x1 = 0;
    uint16_t y0 = 0;
    uint16_t y1 = 0;
};

struct TAP{
  uint8_t targetID = 0;
  uint8_t sourceID = 0;
  uint8_t length = 0;
  uint8_t typeID = 0;
};

TAP tapHeader;

location_data locdata;
joystick_data joydata;

//Storing and detecting practical GPS sentences
    //Sentence prefix to identify exact GNSS service
    char prefix[16];
    //Uart sentence storage
    char sentence[128];

    //Working sentence storage buffer
    char gpsSentenceBuffer[128];

    //Semaphore for working sentence storage
    bool gpsSentenceBlock = false;


//TAP command receiving
    //Semaphore for working buffer storage
    bool tapBufferBlock = false;
    //Working buffer
    char tapBuffer[255];
    //Indx for the tap buffer, we can't trust that it will not contain any 0s!
    uint8_t tapBufferIdx = 0;
    //Buffer for reading from serial
    char tapCommand[255];
    //Index for the tap command, we can't trust that it will not contain any 0s!
    uint8_t tapCommandIdx = 0;

uint8_t clear_array(uint8_t* array, uint8_t size){
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
    clear_array((uint8_t*)loc_deg, sizeof(loc_deg));
    
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
    //printf("Local coordinate:\t%4.4lf", loc_final);
    return(loc_final);
}

//Semaphore UP is blocking!
uint8_t semaphore_up(bool sem){
    if(!sem){
        sem = true;
        return(0);
    }
    return(1);
}

//Semaphore DOWN is NOT blocking!
uint8_t semaphore_down(bool sem){
    if(sem){
        sem = false;
        return(0);
    }
    return(1);
}

uint8_t tap_to_buffer(){
    if(semaphore_up(tapBufferBlock)==0){
        memcpy(tapBuffer, tapCommand, sizeof(tapCommand)-1);
        semaphore_down(tapBufferBlock);
        printf(" <-> Copied %d bytes\n", sizeof(tapBuffer));
    }
    else{
        printf("TAP Message semaphore - Copy operation prohibited.\n");
    }

    return(0);
}

uint8_t parse_tap_command(){
    if( ms_since_boot - ms_last_rx_poll >= 50){
        ms_last_rx_poll = ms_since_boot;
        uint8_t receivedTapPayload[255];

        TAP receivedTapHeader;
        if(semaphore_up(tapBufferBlock)==0){
            memcpy((uint8_t*)&receivedTapHeader, tapBuffer, sizeof(receivedTapHeader));
            memcpy((uint8_t*)&receivedTapPayload, tapBuffer+4, tapBuffer[2]);
            semaphore_down(tapBufferBlock);

        }
        else{

        }
        printf("DETECTED TYPE:%d\n",receivedTapHeader.typeID);
    }
    return(0);
}


uint8_t sentence_to_buffer(){
    memcpy(prefix, &sentence[1],5);

    if(!strcmp(prefix, "GNRMC")){
        //printf("Got a location!\n");
        if(semaphore_up(gpsSentenceBlock)==0){
            strcpy(gpsSentenceBuffer, sentence);
            semaphore_down(gpsSentenceBlock);
        }
        else{
            //printf("GPS Sentence semaphore - Parse operation prohibited.\n");
        }   
    }
    return(0);
}

uint8_t parse_sentence(){

    char sentencePart[128];
    if(semaphore_up(gpsSentenceBlock)==0){
        strcpy(sentencePart,&gpsSentenceBuffer[7]);
        semaphore_down(gpsSentenceBlock);
    }
    else{
        //printf("GPS Sentence semaphore - Copy operation prohibited.\n");
    }

    char buffer[128];
    memset(buffer, 0, sizeof(buffer));
    uint8_t field_counter = 0;
    
    for(int i = 0; i<strlen(sentencePart); i++){

        if(sentencePart[i]==','){
            
            field_counter++;
            //printf("%d\n", field_counter);

            switch(field_counter){
                case 3:
                    //printf("LATITUDE LINE:%s\n",buffer);
                    //We assume any string with fewer than 9 characters is not a valid coordinate value
                    if(strlen(buffer)>9){
                        locdata.lat = coord_clean(buffer, sentencePart[i+1]);
                    }
                    
                    break;
                case 5:
                    //printf("LONGITUDE LINE:%s\n",buffer);
                    //We assume any string with fewer than 9 characters is not a valid coordinate value
                    if(strlen(buffer)>9){
                        locdata.lon = coord_clean(buffer, sentencePart[i+1]);
                    }
                    break;
                default:
                    //printf("Other!\n");
                    break;
            }
            memset(buffer, 0, sizeof(buffer));               
        }
        else{
            buffer[strlen(buffer)] = sentencePart[i];
            //buffer[strlen(buffer)] = sentencePart[i];
        }
        
    }
        //Clearing the buffer between sentence sections
        memset(buffer, 0, sizeof(buffer));
        //printf("%d\t",field_counter);

    //clear_array((uint8_t*)prefix, sizeof(prefix));
    
    //printf("Sentence to process:\t%s\n",sentencePart);
    //printf("Done!\n");
    return(0);

}

void on_gps_rx() {
    while (uart_is_readable(UART_ID)) {
        //printf("Reading a char from UART!\n");
        uint8_t ch = uart_getc(UART_ID);
        //printf("%c", ch);

        sentence[strlen(sentence)] = ch;
        if(ch == '\n'){
            //printf("Received a sentence!\n");
            //printf("This one: %s\n", sentence);
            sentence_to_buffer();
            clear_array((uint8_t*)sentence, sizeof(sentence));          
        }
    }
}

//TODO
//WIP
void on_tap_rx(){
    while (uart_is_readable(uart0)) {
        //printf("Reading a char from UART!\n");
        uint8_t ch = uart_getc(uart0);

        tapCommand[tapCommandIdx] = ch;
        tapCommandIdx++;
        printf("Received:%d\n",(uint8_t)ch);

        //if(!strcmp(tapCommand + strlen(tapCommand-2), {(char)170, (char)170, (char)0})){
        if(tapCommand[tapCommandIdx-1] == (char)170 && tapCommand[tapCommandIdx-2] == (char)170){
            printf("!!!!!\n");
            tap_to_buffer();
            clear_array((uint8_t*)tapCommand, sizeof(tapCommand));
            tapCommandIdx = 0; 
        }
    }
}

//GPS NEO6M

// Perform initialisation
int pico_led_init(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_init(TAIL_LIGHT);
    gpio_set_dir(TAIL_LIGHT, GPIO_OUT);
    gpio_init(STARBOARD_LIGHT);
    gpio_set_dir(STARBOARD_LIGHT, GPIO_OUT);
    gpio_init(PORT_LIGHT);
    gpio_set_dir(PORT_LIGHT, GPIO_OUT);
    gpio_init(STROBES);
    gpio_set_dir(STROBES, GPIO_OUT);
    return PICO_OK;
}

void pico_set_led() {
    if(ms_since_boot - ms_last_change <= 1000){
        if((ms_since_boot - ms_last_change >= 200 && ms_since_boot - ms_last_change <= 275)||(ms_since_boot - ms_last_change >= 325 && ms_since_boot - ms_last_change <= 400)){
            gpio_put(PICO_DEFAULT_LED_PIN, true);
        }
        else{
            gpio_put(PICO_DEFAULT_LED_PIN,false);
        }
    }
    else{
        ms_last_change = ms_since_boot;
    }
}

void strobes(){
    if(ms_since_boot - ms_strobe <= 1000){
        if((ms_since_boot - ms_strobe >= 200 && ms_since_boot - ms_strobe <= 275)||(ms_since_boot - ms_strobe >= 325 && ms_since_boot - ms_strobe <= 400)){
            gpio_put(STROBES, true);
        }
        else{
            gpio_put(STROBES,false);
        }

    }
    else{
        ms_strobe = ms_since_boot;
    }
}


void read_accel(ACCELEROMETER accel) {
    if( ms_since_boot - ms_last_read >= 50){
        ms_last_read = ms_since_boot;
        //printf("Roll:%3.3f\tPitch:%3.3f\t\tX: %-2.3f\tY: %-2.3f\tZ: %-2.3f\n",accel.getRoll(),accel.getPitch(), accel.getRawX(), accel.getRawY(), accel.getRawZ());
        locdata.roll = accel.getRollUD();
        locdata.pitch = accel.getPitch();
    }
}

void process_gps_uart() {
    if( ms_since_boot - ms_last_loc >= 750){
        ms_last_loc = ms_since_boot;
        parse_sentence();
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

        //Regular heading using the X and Y axis from the magnetometer.
        //locdata.heading = mag.getHdg();

        //Roll correction but the MPU6050 I have is... weirdly oriented.
        //locdata.heading = mag.getRCHdg(locdata.pitch);
        locdata.heading = mag.getHdg();

        locdata.magX = mag.getNormX();
        locdata.magY = mag.getNormY();
        locdata.magZ = mag.getNormZ();
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

//Sending sensor readings over TAP for telemetry
uint8_t tapReadings() {
    if( ms_since_boot - ms_last_tap >= 500){
        ms_last_tap = ms_since_boot;

        //WE NEED FLOATS FOR TAP, NOT DOUBLES!
        float tmp_lat = (float)locdata.lat;
        float tmp_lon = (float)locdata.lon;

        //printf("%4.4f\t%4.4f\t\t%4.4fº\t%4.4fº\t%1.4f\t%1.4f\t%1.4f\n", locdata.lat, locdata.lon, locdata.roll, locdata.pitch, locdata.magX, locdata.magY, locdata.magZ);
        uint8_t buffer[128];
        memcpy(buffer, (uint8_t*)&tmp_lat, sizeof(float));
        memcpy(buffer + (1*sizeof(float)), (uint8_t*)&tmp_lon, sizeof(float));
        buffer[8] = 170;
        buffer[9] = 170;
        //sprintf(buffer, "%c%c", (char)170, (char)170);
        //sprintf(buffer, "GPS\t%lf\t%lf\t\tMAG\t%f\t%f\t%f\t%f\t\tACC\t%4.4lf\t%4.4lf%c%c",locdata.lat, locdata.lon, locdata.heading, locdata.magX, locdata.magY, locdata.magZ, locdata.roll, locdata.pitch, (char)170, (char)170);
        uart_puts(uart0, (char*)buffer);
        
        //printf("%f = %s\n",tmp_lon,(char*)buffer+4);
    }
    return(0);
}

uint8_t printReadings() {
    if( ms_since_boot - ms_last_print >= 500){
        ms_last_print = ms_since_boot;
        //printf("%4.4f\t%4.4f\t\t%4.4fº\t%4.4fº\t%1.4f\t%1.4f\t%1.4f\n", locdata.lat, locdata.lon, locdata.roll, locdata.pitch, locdata.magX, locdata.magY, locdata.magZ);
        
        //DEBUG:
        //printf("GPS\t%lf\t%lf\t\tMAG\t%f\t%f\t%f\t%f\t\tACC\t%4.4lf\t%4.4lf\n",locdata.lat, locdata.lon, locdata.heading, locdata.magX, locdata.magY, locdata.magZ, locdata.roll, locdata.pitch);
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

    //UART1 - GPS MODULE
    // ==================================================================================== //
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    //Uart parity, fifo, format... settings
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, false);

    //UART receiving causes interrupts
    int UART_IRQ = UART_ID == uart1 ? UART1_IRQ : UART0_IRQ;
    irq_set_exclusive_handler(UART_IRQ, on_gps_rx);
    irq_set_enabled(UART_IRQ, true);
    uart_set_irq_enables(UART_ID, true, false);

    //UART0 - TAP COMMUNICATION
    // ==================================================================================== //
    uart_init(TAP_UART_ID, 115200);
    gpio_set_function(0, UART_FUNCSEL_NUM(TAP_UART_ID, 0));
    gpio_set_function(1, UART_FUNCSEL_NUM(TAP_UART_ID, 1));

    //Uart parity, fifo, format... settings
    uart_set_hw_flow(TAP_UART_ID, false, false);
    uart_set_format(TAP_UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(TAP_UART_ID, false);

    
    //UART receiving causes interrupts
    int UART_IRQ_0 = TAP_UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ_0, on_tap_rx);
    irq_set_enabled(UART_IRQ_0, true);
    uart_set_irq_enables(TAP_UART_ID, true, false);
    

    // ==================================================================================== //

    //Emptying out the sentence to make strlen work
    memset(sentence, 0, sizeof(sentence));


    
    while (true) {
        ms_since_boot = to_ms_since_boot(get_absolute_time());
        
        read_accel(accel);
        //printf("Accel read!");
        read_mag(mag);
        //read_gps(gps, gdata);
        process_gps_uart();
        //read_joy(ads);
        pico_set_led();
        strobes();
        printReadings();
        tapReadings();
        parse_tap_command();

    }
}
