/*
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
#include "Arduino.h"
#include <SFPddm.h>
#include <i2c_t3.h>

// Creating an object of SFP DDM library
SFPddm sfp;

byte supp;

// counter
int count = 0;

// Setup
void setup()
{
  Serial.begin(9600);

  Wire.begin(I2C_MASTER,0,I2C_PINS_18_19,I2C_PULLUP_INT, I2C_RATE_400);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  //initialize
  Serial.print("Initialized 0x");
  sfp.begin();
  digitalWrite(13, LOW);
  supp = sfp.getSupported();
  Serial.print("Supported 0x");
  Serial.println(supp,HEX);

}

// Main operation loop
void loop() {

  if(supp & 0x40){
    digitalWrite(13, HIGH);
    if( ! sfp.readMeasurements() ) {
      Serial.println("");
      Serial.print("SFPddm monitoring:");
      Serial.println(count++);

      Serial.print("Temperature:");
      Serial.println(sfp.getTemperature()/256,DEC);

      Serial.print("Voltage:");
      Serial.println(sfp.getVoltage(),DEC);

      Serial.print("TX current:");
      Serial.println(sfp.getTXcurrent(),DEC);

      Serial.print("TX power:");
      Serial.println(sfp.getTXpower(),DEC);

      Serial.print("RX power:");
      Serial.println(sfp.getRXpower(),DEC);
    }
    digitalWrite(13, LOW);
  }
  delay(1000);
}
