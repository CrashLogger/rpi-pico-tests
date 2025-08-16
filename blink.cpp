#include "pico/stdlib.h"
#include <stdio.h>
#include "ACCELEROMETER.h"
#include "MAG.h"
#include "string.h"
#include "ADS.h"
#include "SERVO.h"

#define TAIL_LIGHT 13
#define STARBOARD_LIGHT 12
#define PORT_LIGHT 11
#define RADIO_LINK_LOSS_INDICATOR 14
#define STROBES 15

//GPS NEO6M DEFINTIONS
#define UART_ID uart1
#define TAP_UART_ID uart0
#define BAUD_RATE 9600
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

#define UART_TX_PIN 4
#define UART_RX_PIN 5

static int chars_rxed = 0;

#define VTAIL_GAIN 0.5

//SCHEDULING

uint32_t ms_last_read = 0;
uint32_t ms_last_change = 0;
uint32_t ms_last_loc = 0;
uint32_t ms_last_hdg = 0;
uint32_t ms_last_print = 0;
uint32_t ms_last_tap = 0;
uint32_t ms_last_joy = 0;
uint32_t ms_strobe = 0;
uint32_t ms_last_rx_poll = 0;
uint32_t ms_servo_update = 0;

//Communication watchdog!

uint32_t ms_last_rx = 0;

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

struct TAP_D_COMMAND{
    uint16_t bools = 0;
    uint16_t throttle = 0;
    uint16_t ail_roll = 0;
    uint16_t rud_yaw = 0;
    uint16_t ele_pitch = 0;
    uint16_t aux_flaps = 0; 
};

TAP tapHeader;
TAP_D_COMMAND tapDCommand;
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

//Servo controls
    //Regular margins: 200, 1200

    //ESC
SERVO ESC (22, 200, 1200);

    //AIL0 (left) should be 600 900. 600 is all down, 900 is all up (a tad above resting)
SERVO ail0 (17, 600, 900);
    //AIL1 (right) should be 950 600. 950 is all down, 600 is all up (a tad above resting) 
SERVO ail1 (16, 950, 600);

    // RUD0(left) should be 600 900. 600 is all down, 900 is all up (a tad above resting)
//SERVO rud0 (19, 1000, 250);
SERVO rud0 (19, 250, 1000);
    //RUD1 (right) should be 950 600. 950 is all down, 600 is all up (a tad above resting) 
SERVO rud1 (18, 1050, 400);

double coord_clean(char* raw_numeric, char direction){
    char loc_deg[4];
    double loc_final;

    uint8_t decimalPos = 0;
    //Just in case a different compiler doesn't clear assigned arrays.
    memset((uint8_t*)loc_deg, 0, sizeof(loc_deg));
    
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
    return(loc_final);
}

//Although the change is quick enough not to break 99.9999% of the time, this could be atomic-ised.
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
        //We are always copying the same size buffer after all. 255 bytes is not that long
        //printf(" <-> Copied %d bytes\n", sizeof(tapBuffer));
        ms_last_rx = to_ms_since_boot(get_absolute_time());
    }
    else{
        //printf("TAP Message semaphore - Copy operation prohibited.\n");
    }

    return(0);
}

uint8_t parse_tap_command(){

        uint8_t receivedTapPayload[255];

        TAP receivedTapHeader;
        if(semaphore_up(tapBufferBlock)==0){
            memcpy((uint8_t*)&receivedTapHeader, tapBuffer, sizeof(receivedTapHeader));
            memcpy((uint8_t*)&receivedTapPayload, tapBuffer+4, tapBuffer[2]);
            semaphore_down(tapBufferBlock);
        }
        else{

        }

        switch(receivedTapHeader.typeID){
            case 0:
            // A Direct command message, we need to use the right struct for this!
                memcpy((uint8_t*)&tapDCommand, receivedTapPayload, receivedTapHeader.length);
                
                /*
                printf("Bools: 0x%x\n", tapDCommand.bools);
                printf("Throt: 0x%x\n", tapDCommand.throttle);
                printf("Roll:  0x%x\n", tapDCommand.ail_roll);
                printf("Yaw:   0x%x\n", tapDCommand.rud_yaw);
                printf("Pitch: 0x%x\n", tapDCommand.ele_pitch);
                printf("Other: 0x%x\n", tapDCommand.aux_flaps);
                break;
                */

            // We avoid dealing with message types we don't expect
            default:
                break;
        }
        //printf("DETECTED TYPE:%d\n",receivedTapHeader.typeID);
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

uint8_t parse_gps_sentence(){

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
                    //We assume any string with fewer than 9 characters is not a valid coordinate value
                    if(strlen(buffer)>9){
                        locdata.lat = coord_clean(buffer, sentencePart[i+1]);
                    }
                    
                    break;
                case 5:
                    //We assume any string with fewer than 9 characters is not a valid coordinate value
                    if(strlen(buffer)>9){
                        locdata.lon = coord_clean(buffer, sentencePart[i+1]);
                    }
                    break;
                default:
                    //We can check for other fields' data in the future, such as the time, heading or speed!
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
            memset(sentence, 0, sizeof(sentence));      
        }
    }
}

//TODO
//WIP
void on_tap_rx(){
    while (uart_is_readable(uart0)) {
        uint8_t ch = uart_getc(uart0);
        //printf("ch: %d\n", ch);
        tapCommand[tapCommandIdx] = ch;
        tapCommandIdx++;
        //printf("Received:%d\n",(uint8_t)ch);

        //if(!strcmp(tapCommand + strlen(tapCommand-2), {(char)170, (char)170, (char)0})){
        if(tapCommand[tapCommandIdx-1] == (char)170 && tapCommand[tapCommandIdx-2] == (char)170){
            tap_to_buffer();
            memset((uint8_t*)tapCommand, 0, sizeof(tapCommand));
            tapCommandIdx = 0; 
        }
    }
}

uint8_t adjustServos(){
    ail0.moveServo((uint8_t)tapDCommand.ail_roll);
    ail1.moveServo((uint8_t)tapDCommand.ail_roll);

    uint8_t left_mix = (((tapDCommand.rud_yaw)+(tapDCommand.ele_pitch))*VTAIL_GAIN);
    uint8_t right_mix = (((255-tapDCommand.rud_yaw)+(tapDCommand.ele_pitch))*VTAIL_GAIN);

    printf("L: %d R: %d\n",left_mix, right_mix);
    rud1.moveServo(right_mix);
    rud0.moveServo(left_mix);
    return(0);
}

//GPS NEO6M

// Initialisation
int pico_led_init(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_init(TAIL_LIGHT);
    gpio_set_dir(TAIL_LIGHT, GPIO_OUT);
    gpio_init(STARBOARD_LIGHT);
    gpio_set_dir(STARBOARD_LIGHT, GPIO_OUT);
    gpio_init(PORT_LIGHT);
    gpio_set_dir(PORT_LIGHT, GPIO_OUT);
    gpio_init(RADIO_LINK_LOSS_INDICATOR);
    gpio_set_dir(RADIO_LINK_LOSS_INDICATOR, GPIO_OUT);
    gpio_init(STROBES);
    gpio_set_dir(STROBES, GPIO_OUT);
    return PICO_OK;
}

// Decor LED
uint8_t pico_set_led() {
    if(to_ms_since_boot(get_absolute_time()) - ms_last_change <= 1000){
        if((to_ms_since_boot(get_absolute_time()) - ms_last_change >= 200 && to_ms_since_boot(get_absolute_time()) - ms_last_change <= 275)||(to_ms_since_boot(get_absolute_time()) - ms_last_change >= 325 && to_ms_since_boot(get_absolute_time()) - ms_last_change <= 400)){
            gpio_put(PICO_DEFAULT_LED_PIN, true);
        }
        else{
            gpio_put(PICO_DEFAULT_LED_PIN,false);
        }
    }
    else{
        ms_last_change = to_ms_since_boot(get_absolute_time());
    }
    return(0);
}

// Strobe lights
uint8_t strobes(){
    if(to_ms_since_boot(get_absolute_time()) - ms_strobe <= 1000){
        if((to_ms_since_boot(get_absolute_time()) - ms_strobe >= 200 && to_ms_since_boot(get_absolute_time()) - ms_strobe <= 275)||(to_ms_since_boot(get_absolute_time()) - ms_strobe >= 325 && to_ms_since_boot(get_absolute_time()) - ms_strobe <= 400)){
            gpio_put(STROBES, true);
        }
        else{
            gpio_put(STROBES,false);
        }
    }
    else{
        ms_strobe = to_ms_since_boot(get_absolute_time());
    }
    return(0);
}


uint8_t read_accel(ACCELEROMETER accel) {
    locdata.roll = accel.getRollUD();
    locdata.pitch = accel.getPitch();
    return(0);
}

uint8_t read_mag(MAG mag) {
    locdata.heading = mag.getHdg();

    locdata.magX = mag.getNormX();
    locdata.magY = mag.getNormY();
    locdata.magZ = mag.getNormZ();
    return(0);
}

//WIP - some issues... somehow
uint8_t read_joy(ADS ads) {
    if( to_ms_since_boot(get_absolute_time()) - ms_last_joy >= 100){
        sleep_ms(50);
        printf("Hello!\n");
        printf("We are calling the ADS to read some stuff for us.\n");
        ms_last_joy = to_ms_since_boot(get_absolute_time());
        joydata.x0 = ads.readChannel(1);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        //joydata.x1 = ads.readShortChannel(1);
        //joydata.y0 = ads.readShortChannel(2);
        //joydata.y1 = ads.readShortChannel(3);
        
        printf("%d, %d, %d, %d\n", joydata.x0, joydata.x1, joydata.y0, joydata.y1);
    }
    return(0);
}

//Sending sensor readings over TAP for telemetry
uint8_t tapReadings() {
    //WE NEED FLOATS FOR TAP, NOT DOUBLES!
    float tmp_lat = (float)locdata.lat;
    float tmp_lon = (float)locdata.lon;

    printf("%f, %f\n", tmp_lat, tmp_lon);

    uint8_t buffer[128];

    //TODO: Look to fix this stuff, the ESP8266 on the other end receives nothing.
    buffer[0] = 'a';
    buffer[1] = 'a';
    buffer[2] = 'a';
    buffer[3] = 'a';
    buffer[4] = 'a';
    buffer[5] = 'a';
    buffer[6] = 'a';
    buffer[7] = 'a';
    buffer[8] = 170;
    buffer[9] = 170;
    buffer[10] = 0;

    uart_puts(uart0, (char*)buffer);
    return(0);
}

uint8_t printReadings() {
    if( to_ms_since_boot(get_absolute_time()) - ms_last_print >= 500){
        ms_last_print = to_ms_since_boot(get_absolute_time());
        printf("GPS\t%lf\t%lf\t\tMAG\t%f\t%f\t%f\t%f\t\tACC\t%4.4lf\t%4.4lf\n",locdata.lat, locdata.lon, locdata.heading, locdata.magX, locdata.magY, locdata.magZ, locdata.roll, locdata.pitch);
    }
    return(0);
}

uint8_t pico_uart_init(){
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

    //Emptying out the sentence buffer to make strlen work when working with GPS
    memset(sentence, 0, sizeof(sentence));

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

    return(0);
}

int main() {
    stdio_init_all();
    sleep_ms(1000);
    printf("Hello world!");
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    pico_uart_init();

    printf("Starting IMU\n");
    ACCELEROMETER accel(1, 26, 27);
    MAG mag(1, 26, 27);
    sleep_ms(100);
    pico_set_led();
    //ADS ads(i2c1, 15, 14);

    

    
    while (true) {
        //Read IMU values 
        if( to_ms_since_boot(get_absolute_time()) - ms_last_read >= 50){
            ms_last_read = to_ms_since_boot(get_absolute_time());   
            read_accel(accel);
        }

        //Read magnetometer to (attempt to) acquire heading.
        if( to_ms_since_boot(get_absolute_time()) - ms_last_hdg >= 100){
            ms_last_hdg = to_ms_since_boot(get_absolute_time());
            read_mag(mag);
        }

        //Process the last valid string received via UART from the GPS module
        if( to_ms_since_boot(get_absolute_time()) - ms_last_loc >= 750){
            ms_last_loc = to_ms_since_boot(get_absolute_time());
            parse_gps_sentence();
        }
        //read_joy(ads);

        //Transmit telemetry data using TAP
        if( to_ms_since_boot(get_absolute_time()) - ms_last_tap >= 500){
            ms_last_tap = to_ms_since_boot(get_absolute_time());
            tapReadings();
        }

        // Parse any received TAP commands into usable data
        if(to_ms_since_boot(get_absolute_time()) - ms_last_rx_poll >= 25){
            ms_last_rx_poll = to_ms_since_boot(get_absolute_time());
            parse_tap_command();
        }

        // Enable the failsafe if the radio communication gets lost
        if(to_ms_since_boot(get_absolute_time()) - ms_last_rx >= 600){
            gpio_put(RADIO_LINK_LOSS_INDICATOR, true);
        }
        else{
            gpio_put(RADIO_LINK_LOSS_INDICATOR, false);
        }

        
        if(to_ms_since_boot(get_absolute_time()) - ms_servo_update >= 50){
            ms_servo_update = to_ms_since_boot(get_absolute_time());
            adjustServos();
        }

        //Lighting effects, internally scheduled. To be improved.
        pico_set_led();
        strobes();

        //Debug printing, internally scheduled
        //printReadings();

    }
}
