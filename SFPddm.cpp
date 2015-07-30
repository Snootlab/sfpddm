/*
  SFPddm.cpp - SFPddm library

Copyright 2013 Luka Mustafa - Musti, musti@wlan-si.net

This file is part of the SFPddm library for Arduino

The SFPddm library is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your
option) any later version.

The SFPddm library is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with the SFPddm library. If not, see http://www.gnu.org/licenses/.

*/


// Includes

#include <Arduino.h>
#include "SFPddm.h"
#include <inttypes.h>
#include <i2c_t3.h>

// Definitions

#define INFOADDR    0x50 // addr A0/1
#define DDMADDR     0x51 // addr A2/3


// Private variables

// error variable
uint8_t error;
// calibration data variables

struct _cal{
  uint16_t txc_slope;
  int16_t txc_off;
  uint16_t txp_slope;
  int16_t txp_off;
  uint16_t t_slope;
  int16_t t_off;
  uint16_t v_slope;
  int16_t v_off;
};
_cal cal_general = {1,0,1,0,1,0,1,0};

float cal_rxpower[5];
//raw measurement buffer
uint8_t raw_buffer[22];
//raw alarm and warnign buffer
uint8_t raw_alarmwarning[6];
//measurement values

struct _meas {
  int16_t  temperature; //reg A2/96-97
  uint16_t voltage; //reg A2/98-99
  uint16_t TXcurrent; //reg A2/100-101
  uint16_t TXpower; //reg A2/102-103
  uint16_t RXpower; //reg A2/104-105
  uint32_t RESERVED; //reg A2/106-109
  uint8_t  RESERVED2; //reg A2/111 Intentional swapping due to eandiness adjustment in writing!
  uint8_t  status; //reg A2/110 Intentional swapping due to eandiness adjustment in writing!
  uint16_t alarms; //reg A2/112-113
  uint16_t RESERVED3; //reg A2/114-115
  uint16_t warnings; //reg A2/116-117
};
_meas measdata={0,0,0,0,0,0,0,0,0,0,0};
//supported modes flags
uint8_t supported;
//contains register A0/92

// Constructor /////////////////////////////////////////////////////////////////

SFPddm::SFPddm()
{
    //reset error code
    error=0x00;
}

// Public Methods //////////////////////////////////////////////////////////////

// The function initializes the communication, checks if the module is present, retrieves necessary information
uint8_t SFPddm::begin(void){

  Wire.beginTransmission(INFOADDR);
  Wire.write(92);
  Wire.endTransmission();

  Wire.requestFrom(INFOADDR, 1);
  supported = Wire.read();

  return 0;
}

// The function returns an OR or all error codes detected, should be 0x00 if data is to be valid
// Can be used to check if the module is present
uint8_t SFPddm::getStatus(){
  // Do a test write to register pointer.
  Wire.beginTransmission(INFOADDR);
  Wire.write(0);
  Wire.endTransmission();

  return error;
}

// This function can be used to get the supported information
uint8_t SFPddm::getSupported(){
  return supported;
}

// The function acquires the measurements and returns an error code if sth went wrong, 0x00 is OK
uint8_t SFPddm::readMeasurements(){
  int i;
  //read diagnostic measurements registers 96-105 of 0xA2, store them in buffer

  Wire.beginTransmission(DDMADDR);
  Wire.write(96);
  Wire.endTransmission();

  Wire.requestFrom(DDMADDR, 22);
  for (i=0;i<22;i++) {
	  raw_buffer[i] = Wire.read();
  }

  //copy raw measurements to results union
  uint8_t *p_meas = (uint8_t*)&measdata;
  for(i=0;i<22;i+=2){
    *p_meas++ =raw_buffer[i+1];
    *p_meas++ =raw_buffer[i];
  }

return error;
}

// The function gets the value of the control register (0xA2 memory, register 110)
uint8_t SFPddm::getControl(){
  return measdata.status;
}
// The function sets the value of the control register (0xA2 memory, register 110)
void SFPddm::setControl(uint8_t data) {

  Wire.beginTransmission(DDMADDR);
  Wire.write(110);
  Wire.write(data&0xff);
  Wire.endTransmission();

}

// The function returns the temperature , signed.
int16_t SFPddm::getTemperature(){
  return measdata.temperature;
}

// The function returns the supply voltage , unsigned
uint16_t SFPddm::getVoltage(){
  return measdata.voltage;
}

// The function returns the supply current , unsigned
uint16_t SFPddm::getTXcurrent(){
  return measdata.TXcurrent;
}

// The function returns the TX power , unsigned
uint16_t SFPddm::getTXpower(){
  return measdata.TXpower;
}

// The function returns the RX power , unsigned
uint16_t SFPddm::getRXpower(){
  return measdata.RXpower;
}

uint16_t SFPddm::getAlarms(){
  return measdata.alarms;
}

uint16_t SFPddm::getWarnings(){
  return measdata.warnings;
}

