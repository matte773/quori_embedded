#ifndef CONSTANTS_h
#define CONSTANTS_h


/*
 * Robot Physical Constants. MODIFY THESE TO REFLECT NEW ROBOT
*/

#define LOOPTIME              4000                    // Desired speed of the main loop in microseconds
#define DT                    1.0/(LOOPTIME/1000000.0)    //dt in seconds
#define DT_INV                1.0/DT                   //inv dt in 1/seconds
#define LOOPHZ                1.0/(LOOPTIME/1000000.0)    // Desired Hz of the main loop
#define DEBUGTIME             100000                  // Desired speed of the main loop in microseconds
#define CMD_TIMEOUT           250000                  // microseconds for robot to declare timeout
#define UPDATE_MSG_TIME       10000                  // ROS communication rate set in microseconds
#define RAD2DEG               180/PI

#define G_RATIO 5.0

#define JOINT_UPP_LIMIT  27.0*3.1415/180.0
#define JOINT_LOW_LIMIT  -12.1*3.1415/180.0
#define MOTOR_UPP_LIMIT  27.0*3.1415/180.0*G_RATIO 
#define MOTOR_LOW_LIMIT  -12.1*3.1415/180.0*G_RATIO 
#define SENSOR_MAX 16383.0
#define SENSOR_HALF 8191.5


#define COAST_STATE 1
#define MOVE_STATE 0



#endif
