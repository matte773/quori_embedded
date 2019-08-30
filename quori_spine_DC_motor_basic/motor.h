#ifndef Motor_h
#define Motor_h

#include "pid.h"

#define DIR_FORWARD 1
#define DIR_BACKWARD 0
#define REVERSE 0x85
#define FORWARD 0x86

void set_direction(int pin_driver_inA, int pin_driver_inB, bool dir);

/*
void set_speed(PID_Vars* pid, 
               float speed_req, 
               float speed_act, 
               float* speed_cmd,
               float t_elapsed, 
               //int* pwm_val,
               int* serial_val,
               //int pin_driver_dir,
               int motor_tag,
               //int pin_pwm,
               float* pidError);*/
void set_speed(float* speed_cmd,
               int* serial_val,
               int motor_tag);
/*
void set_speed_of_turret(PID_Vars* pid, 
               float speed_req, 
               float speed_act, 
               float* speed_cmd,
               float t_elapsed, 
               //int* pwm_val,
               int* serial_val,
               //int pin_driver_dir,
               //int pin_pwm,
               float* pidError);
*/
void set_volts_of_turret(float* speed_cmd,
                         int* serial_val);
                         
float get_speed_from_difference(float difference, 
                float time_elapsed_secs_inv);

float get_ang_speed_from_difference(float difference,
                    float time_elapsed_secs_inv);

                    
#endif




