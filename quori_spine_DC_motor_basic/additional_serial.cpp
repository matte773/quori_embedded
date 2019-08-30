#include <Arduino.h>
#include <SPI.h>
#include <wiring_private.h>
#include "additional_serial.h"


void serial_setup()
{
  //Serial.begin(57600);
  
  //Serial1.begin(2000000);           // Begin Serial1
  Serial1.begin(115200);  
  //Serial3.begin(2000000); // Begin Serial3
}

void serial_motor_controls(int motor_tag,
                           float DIR_PACK,
                           int serial_val)
{
  serial_val = abs(serial_val);
//  Serial.println(serial_val);
  Serial1.write((byte)0xAA); // recognize baud rate 
  Serial1.write((byte)motor_tag); // identify which motor to send packet to 
  Serial1.write((byte)DIR_PACK); // forwards or backwards 
  Serial1.write(serial_val & 0x1F); // speed packet 1
  Serial1.write(serial_val >> 5); // speed packet 2 
}




