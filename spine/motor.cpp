// motor.cpp
// deals with setting speed of motors as well as calculating speed from 
// encoder readings

// For debugging purposes, if you need to override the PID control and just do
// direct velocity control via voltage, comment lines labeled 1 and 2 in 
// set_speed and set_speed_of_turret and uncomment lines 3. 
#include "motor.h"
#include "Arduino.h"
#include "constants.h"
#include "additional_serial.h" 

#define MAX_ROT_SPEED 180.0
#define FORWARD_DRIVE 0x05
#define REVERSE_DRIVE 0x06 

float DIR_PACK = 0x00;

void set_volts(float* cmd_v,
                        int* serial_val) {
    // Sets speed of turret
    // Only thing that's different between the above function and this is the 
    // -1 in *cmd_v, which is to take into consideration the differing
    // directions of the motor controlling the turret and the turret itself.
    *serial_val = round((*cmd_v)*(3200)); //2;
    *serial_val = constrain(*serial_val, -3200, 3200);

    if (*serial_val < 0) {
        // reverse direction
        //set_direction(pin_driver_dir, !M1_FORWARD);
        DIR_PACK = REVERSE_DRIVE; 
    } else {
        // forward direction
        //set_direction(pin_driver_dir, M1_FORWARD);
        DIR_PACK = FORWARD_DRIVE; 
    }
    serial_motor_controls (1, 
                           DIR_PACK, 
                           *serial_val);
} 



