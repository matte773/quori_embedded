#ifndef CONSTANTS_h
#define CONSTANTS_h
#define M1_FORWARD 1
#define M2_FORWARD 1
#define MT_COUNTER 0
/* interrupt pins */
#define PIN_MT_ENCODER_A 62
#define PIN_MT_ENCODER_B 63

/*
 * Robot Physical Constants. MODIFY THESE TO REFLECT NEW ROBOT
 */
#define TICKS_PER_REV_DDRIVE  16383.0                  // number of encoder ticks in one full rotation on diff drive motor
#define TICKS_PER_REV_TURRET  16383.0//16383.0 * (225.0/24.0)*16.0  // actual measured 1024.0 * (225/24) (16383 the new encoder, 225/24 being the other gear, 16 being the turret motor gearbox). Note this was 220/24 for some reason.
#define WHEEL_DIAMETER        0.1524//0.12192                 // actually 4.85 ' diameter // in meters (4.8' diameter)  
#define WHEEL_RADIUS          (WHEEL_DIAMETER / 2.0)  // wheel radius, in meters
#define WHEEL_DIST            0.12284075//0.246634//0.270413 //0.328168                // distance between diff drive wheels, in meters (12.92')
#define DIST_PER_REV          (PI*WHEEL_DIAMETER)     // circumference of wheel in meters
#define LOOPTIME              10000                    // Desired speed of the main loop in microseconds
#define DT                    1.0/(LOOPTIME/1000000.0)    //dt in seconds
#define DT_INV                1.0/DT                   //inv dt in 1/seconds
#define LOOPHZ                1.0/(LOOPTIME/1000000.0)    // Desired Hz of the main loop
#define DEBUGTIME             100000                  // Desired speed of the main loop in microseconds
#define COMMAND_TIMEOUT       250000                 // UDP packect timeout
#define UPDATE_MSG_TIME       10000 
//#define UPDATE_MSG_TIME       10000                 // desired time to send robot update to PC. it might help system speed to have slower update rate
#define RAD2DEG               180/PI
#define TICKS2REV             1.0/TICKS_PER_REV_TURRET // Convert a encoder value to angle measurement in degrees
#define DIST_PER_TICK_mS2S    1000.0*DIST_PER_REV/TICKS_PER_REV_DDRIVE   //used to calculate speed of motors
#define DEG_PER_REV_mS2S      1000.0*-1.0*360.0/TICKS_PER_REV_DDRIVE                           // used to calculate speed of turret  
#define DIST_PER_TICK_uS2uS   1000000.0*DIST_PER_REV/TICKS_PER_REV_DDRIVE   //used to calculate speed of motors
#define DEG_PER_REV_uS2uS     1000000.0*-1.0*360.0/TICKS_PER_REV_DDRIVE                           // used to calculate speed of turret     
#define DIST_PER_TICK_s       1.0*DIST_PER_REV/TICKS_PER_REV_DDRIVE   //used to calculate speed of motors
#define DEG_PER_REV_s         1.0*-1.0*360.0/TICKS_PER_REV_DDRIVE                           // used to calculate speed of turret                      

// Constant parameters for motor or turret(not turret motor) velocity limits. units are deg/s and m/s. (rmp)/(to rev per second)*(radians/s)*(limit adjustment)
#define MAX_MT_VEL            200//(8700.0/60.0)*360.0*(24.0/225.0)*(1.0/16.0)*2.0/3.0   //Note 2/3 is used to estimate the turret reduced speed due to load.
#define MAX_M1_VEL            0.8//(110/60.0)*WHEEL_DIAMETER*3.1415*0.75         //Note 0.95  is used to estimate the motor's reduced speed due to load. 0.8777351
#define MAX_M2_VEL            0.8//(110/60.0)*WHEEL_DIAMETER*3.1415*0.75         //Note 0.95  is used to estimate the motor's reduced speed due to load 0.8777351


/**
 * Pin Definitions
 * M1 = Right motor. actually i think this is left: andrew
 * M2 = Left motor. actuall i think this is right: andrew
 * MT = Turret motor. cw is positive.
 */
/* Motor Driver Pinouts */
#define M2_PWM_PIN 5
#define M2_DIR_PIN 0

#define M1_PWM_PIN 4
#define M1_DIR_PIN 1

#define MT_PWM_PIN 3
#define MT_DIR_PIN 2

#endif




