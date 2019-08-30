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

//void set_direction(int pin_driver_dir, bool dir) {
  //digitalWrite(pin_driver_dir, dir);
//}

void set_speed(float* speed_cmd,
               int* serial_val,
               int motor_tag) {
    *serial_val = round((*speed_cmd)*3200); // 2
    *serial_val = constrain(*serial_val, -3200, 3200); 

    if (*serial_val < 0) {
    //if (*pwm_val < 0) {
        // reverse direction
        // set_direction(pin_driver_dir, !M1_FORWARD);
        DIR_PACK = REVERSE_DRIVE;
    } else {
        // forward direction
        // set_direction(pin_driver_dir, M1_FORWARD);
        DIR_PACK = FORWARD_DRIVE;
    }
  serial_motor_controls(motor_tag, 
                        DIR_PACK,
                        *serial_val); 
} 

void set_volts_of_turret(float* speed_cmd,
                        int* serial_val) {
    // Sets speed of turret
    // Only thing that's different between the above function and this is the 
    // -1 in *speed_cmd, which is to take into consideration the differing
    // directions of the motor controlling the turret and the turret itself.
    *serial_val = round((*speed_cmd)*(-3200)); //2;
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

/**
 * Calculates speed from encode counts
 * 
 * @param difference Difference between the previous and current encoder counts
 * @param time_elapsed_secs_inv 1/time in 1/seconds
 * @returns Speed from the readings. 
 */
float get_speed_from_difference(float difference,
                                float time_elapsed_secs_inv) {
     return (float)difference*DIST_PER_TICK_s*time_elapsed_secs_inv;
//    return (float)difference*DIST_PER_TICK_mS2S*time_elapsed_inv;
    //return ((((float) difference) / ticks_per_rev) * dist_per_rev) / (time_elapsed / 1000.0);
}

float get_ang_speed_from_difference(float difference,
                  float time_elapsed_secs_inv) {
    // Calculates angular speed of encoder in degrees/s
    // difference: difference between prev and current encoder counts
    // time_elapsed_secs_inv: 1/time in 1/seconds
    return (float)difference*DEG_PER_REV_s*time_elapsed_secs_inv; 
    //return -1 * 360 * (((float) difference) / ticks_per_rev) / (time_elapsed / 1000.0);
}




