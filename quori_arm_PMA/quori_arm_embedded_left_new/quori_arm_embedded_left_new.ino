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
//#include <AS5048A.h>    //documentation: https://github.com/ZoetropeLabs/AS5048A-Arduino
#include "src/MLX90363/MLX90363.h"
#include <SPI.h>

#define LOOPTIME 10000
#define POS 0
#define VEL 1
#define TICKS_PER_REV 16383
#define READ_DELAY 2
#define JOINT_2_UPP_LIMIT  1.3
#define JOINT_2_LOW_LIMIT -1.3
#define READ_TIMEOUT 4


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
MLX90363 angleSensor1(9);
MLX90363 angleSensor2(10);


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


/************************/
/*        Timing        */
/************************/
unsigned long last_command_time     = 0;
unsigned long last_recorded_time    = 0;
const unsigned long command_timeout = 500000;// half second

/***************************/
/*    State Variables      */
/***************************/
float joint_1_pos_cmd = 0; // cmd in arm coordinate fram in radians
float joint_2_pos_cmd = 0; //command input value
float joint_1_pos_meas,joint_2_pos_meas; //measured input value 0 to 1

float motor_1_pos = 0;
float  motor_2_pos = 0;
float motorReadCode = 0; // 1 = only motor 1, 2= only motor 2, 3= motor 1 and 2, 0= no motor read
float motor_1_pos_cmd = 0;
float motor_2_pos_cmd = 0;
float motor_1_pos_offset = 0;
float motor_2_pos_offset = 0;
float motor_1_pos_dt = 0.01;
float motor_2_pos_dt = 0.01;
bool control_mode = 1;// 0 is position 1 is speed
bool override_safety = 0;
String state_data = "booting";
char state_leftarm = 1;
#define COAST_STATE 1
#define MOVE_STATE 0

/***********************/
/*     ROS     zzh     */
/***********************/
ros::NodeHandle nh;

geometry_msgs::Vector3 lp3_msg;
ros::Publisher pub_posLeft("/quori/arm_left/pos_status", &lp3_msg);

geometry_msgs::Vector3 lm3_msg;
ros::Publisher pub_MLeft("/quori/arm_left/motor_status", &lm3_msg);

std_msgs::String state_msg;
ros::Publisher pub_State("/quori/arm_left/state", &state_msg);


/******************************/
/* ROS callback functions zzh */
/******************************/


void CallbackLeftArmPosDir (const geometry_msgs::Vector3& cmd_msg){
  // directly send value to motors
  
  if (cmd_msg.z <= -1){
    set_motor_coast(0);
    set_motor_coast(1);
    state_data += "coasting request";
    state_leftarm = COAST_STATE;
  }
  else{
    last_command_time=micros();
    state_leftarm = MOVE_STATE;
    motor_1_pos_cmd = cmd_msg.x;
    motor_2_pos_cmd = cmd_msg.y;
    motor_1_pos_dt  = cmd_msg.z;
    if (motor_1_pos_dt <0.01){// catch timing input error. bound it by 10ms
      motor_1_pos_dt = 0.01;
    }
    motor_2_pos_dt = motor_1_pos_dt;//TODO change the message type to have 4 slots instead of 3 so both times can be unique.
    control_mode = POS;
  } 
}


void CallbackSyncLeftArmOverride (const std_msgs::Bool& cmd_msg){
    override_safety = cmd_msg.data;

}


ros::Subscriber<geometry_msgs::Vector3> sub_leftarmposdir("/quori/arm_left/cmd_pos_dir", CallbackLeftArmPosDir); //subscriber 
ros::Subscriber<std_msgs::Bool> sub_leftarmoverride("/quori/arm_left/limit_override", CallbackSyncLeftArmOverride); //


/***************************************************/
/*                                                 */
/*                      SETUP                      */
/*                                                 */
/***************************************************/
void setup()
{
  //ROS
  nh.initNode();
  nh.advertise(pub_posLeft);
  nh.advertise(pub_MLeft);
  nh.advertise(pub_State);
  
  nh.subscribe(sub_leftarmposdir);
  nh.subscribe(sub_leftarmoverride);

  // Start SPI (MOSI=11, MISO=12, SCK=13)
  MLX90363::InitializeSPI(11,12,13);  // InitializeSPI only once for all sensors (ZXie)


  // Initialize UART serial ports
  Serial.begin(115200);   // for communication with PC
  Serial1.begin(115200);  // for motor 1 (inner)
  Serial3.begin(115200);  // for motor 2 (outter)
  
  delay(500);
  //update arm positions
  update_states();

  //set zero position
  angleSensor1.SetZeroPosition(map(-2.736, -PI, PI, -8192, 8191));
  angleSensor2.SetZeroPosition(map(-2.270, -PI, PI, -8192, 8191));

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
    //update arm positions
    update_states();

    nh.spinOnce();
 
   // kill all motors if timeout. later it will check buffer for positions it may have run out of.
  if (shoulderAngleLimiter ()){
    //shoulderAngleLimiter will coast motors
    state_data += "angle limit";
  }
  else if (micros() - last_command_time > command_timeout){
    coast_left_arm();
    state_data += "cmd timeout";
  }
  else if (state_leftarm == COAST_STATE){
    coast_left_arm();
  }
  else if (state_leftarm == MOVE_STATE){
    //Update arm
    move_arms();
    state_data += "moving arms";
  }
  ros_telemetry(); 
  }
}


/***************************/
/*          Support        */
/***************************/

// Prepares and sends ROS rotopic messages
void ros_telemetry(){

  lp3_msg.x=joint_1_pos_meas;
  lp3_msg.y=joint_2_pos_meas;
  lp3_msg.z=0;
  pub_posLeft.publish(&lp3_msg);
  
  lm3_msg.x=motor_1_pos;
  lm3_msg.y=motor_2_pos;
  lm3_msg.z=motorReadCode;
  pub_MLeft.publish(&lm3_msg);
  
  char charBuf[50];
  state_data.toCharArray(charBuf, 50); 
  state_msg.data = charBuf;
  pub_State.publish(&state_msg);
  state_data = " ";

  }

// Read and adjust the position of the arm joints.
void update_states(){
  angleSensor1.SendGET3();
  angleSensor2.SendGET3();
  joint_1_pos_meas =(int)angleSensor1.ReadAngle();
  joint_2_pos_meas =(int)angleSensor2.ReadAngle();

  joint_1_pos_meas = joint_1_pos_meas/TICKS_PER_REV;
  joint_2_pos_meas = joint_2_pos_meas/TICKS_PER_REV;
  
  //convert to radians
  joint_1_pos_meas = joint_1_pos_meas*2*PI;
  joint_2_pos_meas = joint_2_pos_meas*2*PI;

  // get motor positions. may not be nessesary now, but will help cath drive slip
  motorReadCode = get_all_motor_pos();
}

//commands arms to move. Currently position and velocity modes are possible, but we only plan to use position.
void move_arms(){
    //position control code.
    // Load next knots in the buffer 
   set_motor_pos(0,motor_1_pos_cmd,motor_1_pos_dt);
   set_motor_pos(1,motor_2_pos_cmd,motor_2_pos_dt);
  
}


// Checks if the arm is close to colliding with itself. If so, the arm is cmded to coast and function returns true;
bool shoulderAngleLimiter (){
  if (((joint_2_pos_meas > JOINT_2_UPP_LIMIT) || (joint_2_pos_meas < JOINT_2_LOW_LIMIT)) && !override_safety){
    coast_left_arm();
    return 1;
  }
  else{
    return 0;
  }
}


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

// Creates and sends message to a motor rloading a trajectory to follow.
void set_motor_pos(int id, float value, float motor_id_pos_dt)
{
  //angle_ctrl_client[id].trajectory_angular_displacement_.set(com[id],value);// trajectory cmd
  //angle_ctrl_client[id].trajectory_duration_.set(com[id],motor_id_pos_dt);// ^
  angle_ctrl_client[id].ctrl_angle_.set(com[id],value); // position control with no duration or speed set. Use either the two trajectory cmds or this
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

// Creates and sends a message to a motor requesting the motor reply with its position. If the reply is read sucessfully the value is stored and the function returns a code revealing which motors were successful.
char get_all_motor_pos()
{
  //Send request to each motor
  for (int id=0; id<=1; id++) {
    angle_ctrl_client[id].obs_angular_displacement_.get(com[id]);
    com[id].GetTxBytes(write_communication_buffer,write_communication_length);
    switch (id){
      case 0:
        Serial1.write((uint8_t*)write_communication_buffer,sizeof(uint8_t[static_cast<int>(write_communication_length)]));
      case 1:
        Serial3.write((uint8_t*)write_communication_buffer,sizeof(uint8_t[static_cast<int>(write_communication_length)]));
    }
  }
  // attempts to read from each motor in serial
  char MotorsReadCode = 0;
  for (int id=0; id<=1; id++) {
    unsigned long read_start_time = millis();
    switch(id){
    case 0:
      //May need to play with this delay.
      read_communication_length = Serial1.readBytes(read_communication_buffer, Serial1.available());
      parseSerialMsg(id);
      while (~angle_ctrl_client[id].obs_angular_displacement_.IsFresh() && (millis()-read_start_time) <  READ_TIMEOUT) {
        read_communication_length = Serial1.readBytes(read_communication_buffer, Serial1.available());
        parseSerialMsg(id);
      }
      MotorsReadCode = MotorsReadCode + getMotorPosMsg(id);
    case 1:
      //May need to play with this delay.
      read_communication_length = Serial3.readBytes(read_communication_buffer, Serial3.available());
      parseSerialMsg(id);
      while (~angle_ctrl_client[id].obs_angular_displacement_.IsFresh() && (millis()-read_start_time) <  READ_TIMEOUT) { 
        read_communication_length = Serial3.readBytes(read_communication_buffer, Serial3.available());
        parseSerialMsg(id);
      }
      MotorsReadCode = MotorsReadCode + getMotorPosMsg(id)*2;
  }
 }
 return MotorsReadCode;
}

void parseSerialMsg(int id){
    uint8_t *rx_data; // temporary pointer to received type+data bytes
    uint8_t rx_length; // number of received type+data bytes
    com[id].SetRxBytes(read_communication_buffer,read_communication_length);
    while(com[id].PeekPacket(&rx_data,&rx_length)){
      // Share that packet with all client objects
      angle_ctrl_client[id].ReadMsg(com[id],rx_data,rx_length);
      // Once we’re done with the message packet, drop it
      com[id].DropPacket();
    }
}

bool getMotorPosMsg(int id){
  if(angle_ctrl_client[id].obs_angular_displacement_.IsFresh()) {
        motor_1_pos = angle_ctrl_client[id].obs_angular_displacement_.get_reply();
        state_data += "m" + String(id) +"_got";
        return 1; //successful get
      }
      else{
        return 0;
      }
}
