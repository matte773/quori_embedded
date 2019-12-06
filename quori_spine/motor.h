#ifndef Motor_h
#define Motor_h

#include "pid.h"

#define DIR_FORWARD 1
#define DIR_BACKWARD 0
#define REVERSE 0x85
#define FORWARD 0x86


void set_volts(float* cmd_v,
                         int* serial_val);


                    
#endif




