/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */
#define USE_USBCON
#include <Arduino.h>                              // required before wiring_private.h
#include <wiring_private.h>


#define UPDATE_MSG_TIME 10000
#define MODE_DURATION 5000000
#define LOOPTIME 10000
#define POS 0
#define VEL 1
#define TICKS_PER_REV 16383
#define R1 12.70
#define R4 75.001
#define R2  85.09
#define R3  38.74
#define G_RATIO1 12.18//(R4 * R2 )/ (R1 * R3 ) //12.971262261 
#define G_RATIO2 12.17//(R4 * R2 )/ (R1 * R3 ) //12.971262261 
#define DT 1.0/LOOPTIME
#define READ_DELAY 2
#define JOINT_2_UPP_LIMIT  1.2
#define JOINT_2_LOW_LIMIT -1.2
#define SENSOR_MAX 16383.0
#define SENSOR_HALF 8191.5
#define JOINT_1_CALI 12856
#define LED 13


// IQinetics Library
#include "src/libiqinetics/inc/bipbuffer.h"
#include "src/libiqinetics/inc/communication_interface.h"
#include "src/libiqinetics/inc/byte_queue.h"
#include "src/libiqinetics/inc/packet_finder.h"
#include "src/libiqinetics/inc/crc_helper.h"
#include "src/libiqinetics/inc/generic_interface.hpp"
#include "src/libiqinetics/inc/multi_turn_angle_control_client.hpp"


/**********************/
/*       IQ-Control   */
/**********************/
// Create GenericInterface and MultiTurnAngleControlClient for 2 motors, the obj_id are 0 as default
// 0 - Inner, 1 - Outer
GenericInterface com[2];
MultiTurnAngleControlClient angle_ctrl_client[2] = {
  MultiTurnAngleControlClient(0),
  MultiTurnAngleControlClient(0)
};

// Write communication buffer
uint8_t write_communication_buffer[256];
uint8_t write_communication_length;
int write_communication_length_int;


// Read communication buffer
uint8_t read_communication_buffer[256];
uint8_t read_communication_length;
int read_communication_length_int;
unsigned long communication_time_last = 0;

//Motor gains information. starting gains were:
//kp 10
//ki 100
//kd 0

/************************/
/*        Timing        */
/************************/
unsigned long last_command_time     = 0;
unsigned long last_recorded_time    = 0;
unsigned long last_update_msg_time  = 0;
unsigned long last_recorded_control_timer = 0;
const unsigned long command_timeout = 500000;// half second
unsigned long blink_timer = 0;// half second

/***************************/
/*    State Variables      */
/***************************/
float joint_1_pos_cmd = 0; // cmd in arm coordinate fram in radians
float joint_2_pos_cmd = 0; //command input value
float joint_1_pos_sync_0 = 0; // cmd in arm coordinate fram in radians
float joint_2_pos_sync_0 = 0; //command input value
float joint_1_pos_meas,joint_2_pos_meas; //measured input value 0 to 1
float joint_1_vel_cmd = 0;//radians/second
float joint_2_vel_cmd = 0;//^

float joint1_max_over = 0;// joint overflow limit varibles
float joint1_min_over = 0;

float motor_1_pos = 0;
float  motor_2_pos = 0;
float motor_1_pos_cmd = 0;
float motor_2_pos_cmd = 0;
float motor_1_pos_offset = 0;
float motor_2_pos_offset = 0;
float motor_1_pos_dt = 0.01;
float motor_2_pos_dt = 0.01;
bool cmd_timeout = 0;
bool led_state = 0;
bool control_mode = 1;// 0 is position 1 is speed
bool override_safety = 0;
String state_data = "booting";
char state_leftarm = 1;
int speed_mag = 56;
#define COAST_STATE 1
#define MOVE_STATE 0


/***************************************************/
/*                                                 */
/*                      SETUP                      */
/*                                                 */
/***************************************************/
void setup()
{

  // Initialize UART serial ports
  Serial.begin(115200);   // for communication with PC
  Serial1.begin(115200);  // for motor 1 (inner)
  Serial3.begin(115200);  // for motor 2 (outter)

  pinMode(LED, OUTPUT);
  
  delay(500);
  //TODO: add motor gain set command
  /*motor 0 Kp:10.00
motor 0 Ki:100.00
motor 0 Kd:0.00
motor 1 Kp:10.00
motor 1 Ki:100.00
motor 1 Kd:0.00
*/
  //set_motor_gains(0, float kp, float ki, float kd);
  //set_motor_gains(0, 10 , 100, 0);
 //set_motor_gains(1, 10 , 100, 0);
  
  Serial.flush();
}

/***************************************************/
/*                                                 */
/*                      MAIN                       */
/*                                                 */
/***************************************************/
void loop(){
  if ((micros() - last_recorded_time) >= LOOPTIME) { // ensures stable loop time
    last_recorded_time = micros();

    if (micros()-last_recorded_control_timer>MODE_DURATION){
      control_mode = !control_mode;
      last_recorded_control_timer = micros();
      Serial.println("Direction change speed at:");
      Serial.println(speed_mag);
    }

    if (control_mode ==0){
      
      set_motor_speed(0, speed_mag);
      set_motor_speed(1, speed_mag); 
    }
    else if (control_mode==1){
      
      set_motor_speed(0, -speed_mag);
      set_motor_speed(1, -speed_mag);
    }

    if (micros()-blink_timer>100000){
      blink_timer = micros();
      digitalWrite(LED, led_state=!led_state);
      //Serial.println("LED state change");
      //Serial.println(led_state);
    }


}
}
/***************************/
/*          Support        */
/***************************/

/***************************/
/*   IQ-Motor Functions    */
/***************************/

// Creates and sends message to motors requesting a coast cmd.
void set_motor_coast(int id)
{
  angle_ctrl_client[id].ctrl_coast_.set(com[id]);
  com[id].GetTxBytes(write_communication_buffer,write_communication_length);
  switch(id){
    case 0:
      Serial1.write((uint8_t*)write_communication_buffer,sizeof(uint8_t[static_cast<int>(write_communication_length)]));
      break;
    case 1:
      Serial3.write((uint8_t*)write_communication_buffer,sizeof(uint8_t[static_cast<int>(write_communication_length)]));
      break;
  }
}

// Wrapper function to coast both left arm motors
void coast_left_arm(){
  set_motor_coast(0);
  set_motor_coast(1);
}

// Creates and sends a message to a motor requesting a velocity be achieved by the motor.
void set_motor_speed(int id, float value)
{
  if (value>100){
    value = 100;// limit the speed to only what the motor can achieve. currently its 100 rads/sec
  }
  angle_ctrl_client[id].ctrl_velocity_.set(com[id], value);
  com[id].GetTxBytes(write_communication_buffer,write_communication_length);
  switch(id){
    case 0:
      Serial1.write((uint8_t*)write_communication_buffer,sizeof(uint8_t[static_cast<int>(write_communication_length)]));
      break;
    case 1:
      Serial3.write((uint8_t*)write_communication_buffer,sizeof(uint8_t[static_cast<int>(write_communication_length)]));
      break;
  }
}
