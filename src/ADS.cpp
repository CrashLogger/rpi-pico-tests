//v1.1
//2024-08-05

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "ADS.h"

  ADS::~ADS(){
  }

  ADS::ADS(i2c_inst_t* i2c_port, uint8_t SCL, uint8_t SDA){

    this -> i2c_port = i2c_port;

    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_pull_up(SCL);
    gpio_pull_up(SDA);

    //4 = 4V Resolution
    ads_init(i2c_port, 4);
  }

  void ADS::ads_init(i2c_inst_t *i2c_port, uint8_t resol){
    i2c_init(i2c_port, 400*1000);
    ads_set_mode(i2c_port, 1);
    ads_set_pga(i2c_port, resol);
    ads_set_mux(i2c_port, 0);
    ads_set_speed(i2c_port, 7);
  }

//We're assuming there's just one connected, on the i2c bus that we are giving it
uint16_t  ADS::ads_read_config(i2c_inst_t *i2c_port){
  printf("Reading config!\n");
  uint8_t preConf[2];
  i2c_write_blocking(i2c_port, I2C_ADDR, (&read_from_address, &config_ptr), 2, true);
  i2c_read_blocking(i2c_port, I2C_ADDR, preConf, 2, false);
  
  //config = (preConf[0] << 8) | preConf[1];
  printf("Read the following:%d\n",(preConf[0] << 8) | preConf[1]);
  return (preConf[0] << 8) | preConf[1];
  
  //printf("Read config: %#04x %#04x - %#06x \n", preConf[0], preConf[1], config);
}


//We're only going to allow single ADC measurements, differential measurements are not necessary
//This function will get called every time we read from the device
void ADS::ads_set_mux(i2c_inst_t* i2c_port, uint8_t channel){
  uint16_t channelMask = 0x7000;
  uint16_t config = ads_read_config(i2c_port);
  uint16_t channels[4] = {0x4000, 0x5000, 0x6000, 0x7000};
  
  /*IMPORTANT*/
  /*
  This works the same way every time, so check this out:
  First we clear all bits selected by a mask with the &= ~ operator combination
  Then we binary-or with what we want to write. Bosh! B)
  */
  
  config &= ~channelMask;
  config |= channels[channel];
  printf("ads_set_mux:\n%d\nconfig\n");
  ads_write_config(i2c_port, config);
}

//We're only going to allow the 4V and 5V options because they're the ones we can use at 3.3V and 5V VCC
//This one, as opposed to the mux setting, will only get called during initialisation
void ADS::ads_set_pga(i2c_inst_t* i2c_port, uint8_t resol){
  uint16_t pgaMask = 0x0E00;
  uint16_t config = ads_read_config(i2c_port);
  config &= ~pgaMask;
  switch(resol){
    case 5:
      //Weird cases where 5V is necessary happen here
      config |= 0x0000;
      break;
    default:
      //3.3V operation happens here
      config |= 0x0200;
      break;
  }
  
  ads_write_config(i2c_port, config);
}


void ADS::ads_set_mode(i2c_inst_t* i2c_port, uint8_t mode){
  uint16_t config = ads_read_config(i2c_port);
  uint16_t modeMask = 0x0100;
  config &= ~modeMask;
  if(mode > 0){
    //Default
    //Setting the operation mode to be single-fire
    config |= 0x0100;
  }
  
  //We also:
  // - bit 4-Forcibly set the comparator mode to regular, instead of windowed comparator mode
  // - bit 3: Set the comaprator polarity to default (Active Low)
  // - bit 2: Set the comparator latches to disable
  uint16_t compMask = 0x001F;
  config &= ~compMask;
  // - bits 0:1: Disable all the comparator nonsense. The previous defaults shouldn't matter because of this, but still.
  config |= 0x0003;
  
  ads_write_config(i2c_port, config);
}

void ADS::ads_trig_read(i2c_inst_t* i2c_port){
  uint16_t config = ads_read_config(i2c_port);
  uint16_t readMask = 0x8000;
  
  config &= ~readMask;
  config |= readMask;
  printf("ads_trig_read");
  ads_write_config(i2c_port, config);
}

void ADS::ads_set_speed(i2c_inst_t* i2c_port, uint8_t speed){
  uint16_t config = ads_read_config(i2c_port);
  uint16_t speedMask = 0x00E0;
  uint16_t speedOpts[8] = {0x0000, 0x0020, 0x0040, 0x0060, 0x0080, 0x00A0, 0x00C0, 0x00E0};
  
  config &= ~speedMask;
  config |= speedOpts[speed];
  
  ads_write_config(i2c_port, config);
}

uint16_t ADS::readChannel(uint8_t channel){
  return ads_read_channel(i2c_port, channel);
}

uint16_t ADS::ads_read_channel(i2c_inst_t* i2c_port, uint8_t channel){

  printf("I am ads_read_channel, I haven't shit the bed yet. Channel: %d\n", channel);

  //Set the desired channel in the input multiplexer
  ads_set_mux(i2c_port, channel);
  
  //TODO: Test if this ruins anything in continuous conversion mode
  //Trigger the reading, only necessary when in single-fire mode
  ads_trig_read(i2c_port);
  
  uint16_t config = ads_read_config(i2c_port);
  //If it's not ready to be read from, we just wait
  while(config < 0x8000){
    config = ads_read_config(i2c_port);
    printf("Waiting!\t%d\n",config);
    sleep_ms(500);
  }
  
  uint8_t res[2];
  uint16_t result;
  
  i2c_write_blocking(i2c_port, I2C_ADDR, (&read_from_address, &conver_ptr), 2, true);
  i2c_read_blocking(i2c_port, I2C_ADDR, res, 2, false);
  
  result = (res[0] << 8) | res[1];
  
  printf("Hello I am ads_read_channel again. Retreived value: %d\n",result);
  return result;
}

uint8_t ADS::readShortChannel(uint8_t channel){
  printf("I am readShortChannel, I haven't shit the bed. Channel: %d\n", channel);
  return(convertAngle(ads_read_channel(i2c_port, channel)));
}

//Literally the map function from arduino
uint8_t ADS::convertAngle(uint16_t receivedPos){

  printf("I am convertAngle, I haven't shit the bed yet. Received potentiometer position: %d\n", receivedPos);

    //receivedPos   : The input value we want to convert
    //in_min        : Lower limit of all possible inputs
    //in_max        : Upper limit of all possible inputs
    //min           : Lower limit of the desired outputs
    //max           : Upper limit of the desired outputs

    uint8_t min = 0;
    uint8_t max = 255;

    uint16_t in_min = 0;
    uint16_t in_max = 26200;

    return(uint8_t)( (receivedPos - in_min) * (max - min) / (in_max - in_min) + min);
}

void ADS::ads_write_config(i2c_inst_t* i2c_port, uint16_t config){
  uint8_t tmp[3];
  
  //When writing an array to i2c, the first thing we write is where the array goes, and then the contents of the array.
  tmp[0] = config_ptr;
  tmp[1] = (uint8_t)(config >> 8);
  tmp[2] = (uint8_t)(config & 0xff);
  
  //printf("Write config: %#04x %#04x \n", tmp[1], tmp[2]);
  
  i2c_write_blocking(i2c_port, I2C_ADDR, (&write_to_address, tmp), 4, false);
}
