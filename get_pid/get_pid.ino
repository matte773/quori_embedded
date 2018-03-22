/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */
#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <Arduino.h>                              // required before wiring_private.h
#include <wiring_private.h>
#include <AS5048A.h>    //documentation: https://github.com/ZoetropeLabs/AS5048A-Arduino

#define UPDATE_MSG_TIME 10000
#define LOOPTIME 10000
#define POS 0
#define VEL 1
#define TICKS_PER_REV 16383
#define R1 12.70
#define R4 75.001
#define R2  85.09
#define R3  38.74
#define G_RATIO 13.0276//(R4 * R2 )/ (R1 * R3 ) //12.971262261 
#define DT 1.0/LOOPTIME
#define READ_DELAY 10
#define JOINT_2_UPP_LIMIT  1.2
#define JOINT_2_LOW_LIMIT -1.2
#define SENSOR_MAX 16383.0
#define SENSOR_HALF 8191.5
#define JOINT_1_CALI 12856


// IQinetics Library
#include "src/libiqinetics/inc/bipbuffer.h"
#include "src/libiqinetics/inc/communication_interface.h"
#include "src/libiqinetics/inc/byte_queue.h"
#include "src/libiqinetics/inc/packet_finder.h"
#include "src/libiqinetics/inc/crc_helper.h"
#include "src/libiqinetics/inc/generic_interface.hpp"
#include "src/libiqinetics/inc/multi_turn_angle_control_client.hpp"
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>


/**********************/
/*       Sensor       */
/**********************/
AS5048A angleSensor1(9);  // sensor for joint 1 //Andrew- swapped these two to reflect current equations
AS5048A angleSensor2(10);  // sensor for joint 2


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
const unsigned long command_timeout = 500000;// half second

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
bool control_mode = 1;// 0 is position 1 is speed
bool override_safety = 0;
String state_data = "booting";
char state_leftarm = 1;
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
  Serial1.begin(115200);
  Serial3.begin(115200);
  Serial.flush();
}

/***************************************************/
/*                                                 */
/*                      MAIN                       */
/*                                                 */
/***************************************************/
void loop(){
  Serial.println(" ");
  Serial.print("motor 0 Kp:");
  get_motor_gains(0, 1,0,0);
  Serial.print("motor 0 Ki:");
  get_motor_gains(0, 0,1,0);
  Serial.print("motor 0 Kd:");
  get_motor_gains(0, 0,0,1);
  delay(1000);
  Serial.println(" ");
  Serial.print("motor 1 Kp:");
  get_motor_gains(1, 1,0,0);
  Serial.print("motor 1 Ki:");
  get_motor_gains(1, 0,1,0);
  Serial.print("motor 1 Kd:");
  get_motor_gains(1, 0,0,1);
  delay(1000);
  Serial.println(" ");
}


/***************************/
/*   IQ-Motor Functions    */
/***************************/

bool get_motor_gains(int id, float kp, float ki, float kd)
{
  uint8_t *rx_data; // temporary pointer to received type+data bytes
  uint8_t rx_length; // number of received type+data bytes
  if (kp>0){
    angle_ctrl_client[id].angle_Kp_.get(com[id]);
    com[id].GetTxBytes(write_communication_buffer,write_communication_length);
    switch(id){
      case 0:
        Serial1.write((uint8_t*)write_communication_buffer,sizeof(uint8_t[static_cast<int>(write_communication_length)]));
        //May need to play with this delay.
      delay(READ_DELAY);
      // Reads however many bytes are currently available
      read_communication_length = Serial1.readBytes(read_communication_buffer, Serial1.available());
      // Puts the recently read bytes into com’s receive queue
      com[id].SetRxBytes(read_communication_buffer,read_communication_length);
      // while we have message packets to parse
      while(com[id].PeekPacket(&rx_data,&rx_length)){
        // Remember time of received packet
        communication_time_last = millis();
        // Share that packet with all client objects
        angle_ctrl_client[id].ReadMsg(com[id],rx_data,rx_length);
        // Once we’re done with the message packet, drop it
        com[id].DropPacket();
      }
        // Check if we have any fresh data
        // Checking for fresh data is not required, it simply
        // lets you know if you received a message that you
        // have not yet read.
        if(angle_ctrl_client[id].angle_Kp_.IsFresh()) {
          motor_2_pos = angle_ctrl_client[id].angle_Kp_.get_reply();
          Serial.println(motor_2_pos);
          return 1; //successful get
        }
        else{
          return 0;
        }
        break;
      case 1:
        Serial3.write((uint8_t*)write_communication_buffer,sizeof(uint8_t[static_cast<int>(write_communication_length)]));
        //May need to play with this delay.
      delay(READ_DELAY);
      // Reads however many bytes are currently available
      read_communication_length = Serial3.readBytes(read_communication_buffer, Serial3.available());
      // Puts the recently read bytes into com’s receive queue
      com[id].SetRxBytes(read_communication_buffer,read_communication_length);
      // while we have message packets to parse
      while(com[id].PeekPacket(&rx_data,&rx_length)){
        // Remember time of received packet
        communication_time_last = millis();
        // Share that packet with all client objects
        angle_ctrl_client[id].ReadMsg(com[id],rx_data,rx_length);
        // Once we’re done with the message packet, drop it
        com[id].DropPacket();
      }
        // Check if we have any fresh data
        // Checking for fresh data is not required, it simply
        // lets you know if you received a message that you
        // have not yet read.
        if(angle_ctrl_client[id].angle_Kp_.IsFresh()) {
          motor_2_pos = angle_ctrl_client[id].angle_Kp_.get_reply();
          Serial.println(motor_2_pos);
          return 1; //successful get
        }
        else{
          return 0;
        }
        break;
    }
  }
  if (ki>0){
    angle_ctrl_client[id].angle_Ki_.get(com[id]);
    com[id].GetTxBytes(write_communication_buffer,write_communication_length);
    switch(id){
      case 0:
        Serial1.write((uint8_t*)write_communication_buffer,sizeof(uint8_t[static_cast<int>(write_communication_length)]));
        //May need to play with this delay.
      delay(READ_DELAY);
      // Reads however many bytes are currently available
      read_communication_length = Serial1.readBytes(read_communication_buffer, Serial1.available());
      // Puts the recently read bytes into com’s receive queue
      com[id].SetRxBytes(read_communication_buffer,read_communication_length);
      // while we have message packets to parse
      while(com[id].PeekPacket(&rx_data,&rx_length)){
        // Remember time of received packet
        communication_time_last = millis();
        // Share that packet with all client objects
        angle_ctrl_client[id].ReadMsg(com[id],rx_data,rx_length);
        // Once we’re done with the message packet, drop it
        com[id].DropPacket();
      }
        // Check if we have any fresh data
        // Checking for fresh data is not required, it simply
        // lets you know if you received a message that you
        // have not yet read.
        if(angle_ctrl_client[id].angle_Ki_.IsFresh()) {
          motor_2_pos = angle_ctrl_client[id].angle_Ki_.get_reply();
          Serial.println(motor_2_pos);
          return 1; //successful get
        }
        else{
          return 0;
        }
        break;
      case 1:
        Serial3.write((uint8_t*)write_communication_buffer,sizeof(uint8_t[static_cast<int>(write_communication_length)]));
        //May need to play with this delay.
      delay(READ_DELAY);
      // Reads however many bytes are currently available
      read_communication_length = Serial3.readBytes(read_communication_buffer, Serial3.available());
      // Puts the recently read bytes into com’s receive queue
      com[id].SetRxBytes(read_communication_buffer,read_communication_length);
      // while we have message packets to parse
      while(com[id].PeekPacket(&rx_data,&rx_length)){
        // Remember time of received packet
        communication_time_last = millis();
        // Share that packet with all client objects
        angle_ctrl_client[id].ReadMsg(com[id],rx_data,rx_length);
        // Once we’re done with the message packet, drop it
        com[id].DropPacket();
      }
        // Check if we have any fresh data
        // Checking for fresh data is not required, it simply
        // lets you know if you received a message that you
        // have not yet read.
        if(angle_ctrl_client[id].angle_Ki_.IsFresh()) {
          motor_2_pos = angle_ctrl_client[id].angle_Ki_.get_reply();
          Serial.println(motor_2_pos);
          return 1; //successful get
        }
        else{
          return 0;
        }
        break;
    }
  }

  if (kd>0){
    angle_ctrl_client[id].angle_Kd_.get(com[id]);
    com[id].GetTxBytes(write_communication_buffer,write_communication_length);
    switch(id){
      case 0:
        Serial1.write((uint8_t*)write_communication_buffer,sizeof(uint8_t[static_cast<int>(write_communication_length)]));
        //May need to play with this delay.
      delay(READ_DELAY);
      // Reads however many bytes are currently available
      read_communication_length = Serial1.readBytes(read_communication_buffer, Serial1.available());
      // Puts the recently read bytes into com’s receive queue
      com[id].SetRxBytes(read_communication_buffer,read_communication_length);
      // while we have message packets to parse
      while(com[id].PeekPacket(&rx_data,&rx_length)){
        // Remember time of received packet
        communication_time_last = millis();
        // Share that packet with all client objects
        angle_ctrl_client[id].ReadMsg(com[id],rx_data,rx_length);
        // Once we’re done with the message packet, drop it
        com[id].DropPacket();
      }
        // Check if we have any fresh data
        // Checking for fresh data is not required, it simply
        // lets you know if you received a message that you
        // have not yet read.
        if(angle_ctrl_client[id].angle_Kd_.IsFresh()) {
          motor_2_pos = angle_ctrl_client[id].angle_Kd_.get_reply();
          Serial.println(motor_2_pos);
          return 1; //successful get
        }
        else{
          return 0;
        }
        break;
      case 1:
        Serial3.write((uint8_t*)write_communication_buffer,sizeof(uint8_t[static_cast<int>(write_communication_length)]));
        //May need to play with this delay.
      delay(READ_DELAY);
      // Reads however many bytes are currently available
      read_communication_length = Serial3.readBytes(read_communication_buffer, Serial3.available());
      // Puts the recently read bytes into com’s receive queue
      com[id].SetRxBytes(read_communication_buffer,read_communication_length);
      // while we have message packets to parse
      while(com[id].PeekPacket(&rx_data,&rx_length)){
        // Remember time of received packet
        communication_time_last = millis();
        // Share that packet with all client objects
        angle_ctrl_client[id].ReadMsg(com[id],rx_data,rx_length);
        // Once we’re done with the message packet, drop it
        com[id].DropPacket();
      }
        // Check if we have any fresh data
        // Checking for fresh data is not required, it simply
        // lets you know if you received a message that you
        // have not yet read.
        if(angle_ctrl_client[id].angle_Kd_.IsFresh()) {
          motor_2_pos = angle_ctrl_client[id].angle_Kd_.get_reply();
          Serial.println(motor_2_pos);
          return 1; //successful get
        }
        else{
          return 0;
        }
        break;
    }
  }

}

