
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
#include "src/MLX90363/MLX90363.h"


#define UPDATE_MSG_TIME 10000
#define LOOPTIME 10000
#define POS 0
#define VEL 1
#define TRAJ 2
#define TICKS_PER_REV 16383
#define G_RATIO 15.5
#define DT 1.0/LOOPTIME
#define READ_DELAY 2
#define JOINT_UPP_LIMIT  29.0*3.1415/180.0
#define JOINT_LOW_LIMIT -14.0*3.1415/180.0
#define SENSOR_MAX 16383.0
#define SENSOR_HALF 8191.5
#define JOINT_1_CALI 12856
#define ANALOGPIN 23
#define MAX_VEL 100 // radians per second


// IQinetics Library
//#include <bipbuffer.h>
//#include <communication_interface.h>
//#include <byte_queue.h>
//#include <packet_finder.h>
//#include <crc_helper.h>
//#include <generic_interface.hpp>
//#include <multi_turn_angle_control_client.hpp>

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
#define CMD_TIMEOUT 500000 // one half second
unsigned long command_timeout = CMD_TIMEOUT;

/***************************/
/*    State Variables      */
/***************************/
float joint_2_pos_meas; //measured input value 0 to 1
float motor_1_vel_cmd = 0;//radians/second


int waist_pos = 0;

float motor_1_pos = 0;
float motor_1_pos_cmd = 0;
float motor_1_pos_dt = 0.01;
bool cmd_timeout = 0;
int dur = 1;
char control_mode = 1;// 0 is position 1 is speed
bool override_safety = 0;
String state_data = "booting";
char state_waist = 1;
#define COAST_STATE 1
#define MOVE_STATE 0

#define OLD_TRAJ 0
#define NEW_TRAJ 1
char trajectory_state = OLD_TRAJ;

/***********************/
/*     ROS     zzh     */
/***********************/
ros::NodeHandle nh;

geometry_msgs::Vector3 lp3_msg;
ros::Publisher pub_posLeft("/quori/waist/pos_status", &lp3_msg);

geometry_msgs::Vector3 lm3_msg;
ros::Publisher pub_MLeft("/quori/waist/motor_status", &lm3_msg);

std_msgs::String state_msg;
ros::Publisher pub_State("/quori/waist/state", &state_msg);


/******************************/
/* ROS callback functions zzh */
/******************************/
void CallbackLeftArmVel (const geometry_msgs::Vector3& cmd_msg){
  // TODO: later this will take in a series of points and load it into a buffer
  
  if (cmd_msg.z <= -1){
    set_motor_coast(0);
    state_waist = COAST_STATE;
    state_data += "coasting request";
  }
  else{
    last_command_time=micros();
    state_waist = MOVE_STATE;
    motor_1_vel_cmd  = cmd_msg.x;
    control_mode = VEL;
  } 
}

void CallbackLeftArmPosDir (const geometry_msgs::Vector3& cmd_msg){
  // directly send value to motors
  
  if (cmd_msg.z <= -1){
    set_motor_coast(0);
    state_data += "coasting request";
    state_waist = COAST_STATE;
  }
  else{
    last_command_time=micros();
    state_waist = MOVE_STATE;
    if (cmd_msg.z == 0){
       motor_1_pos_cmd = cmd_msg.x;
       control_mode = POS;
    }
    else{
      control_mode = TRAJ;
      if (cmd_msg.x == motor_1_pos_cmd  && cmd_msg.z == motor_1_pos_dt){
        trajectory_state = OLD_TRAJ;//I assume you are just checking in
      }
      else{
        motor_1_pos_cmd = cmd_msg.x;
        motor_1_pos_dt  = cmd_msg.z;
        if (motor_1_pos_dt <0.01){// catch timing input error. bound it by 10ms
          motor_1_pos_dt = 0.01;
        }
        trajectory_state = NEW_TRAJ;//
      }
    }
  } 
}


void CallbackLeftArmSetPID1 (const geometry_msgs::Vector3& cmd_msg){
    set_motor_gains(0, cmd_msg.x, cmd_msg.y, cmd_msg.z);

}

ros::Subscriber<geometry_msgs::Vector3> sub_leftarmvel("/quori/waist/cmd_vel", CallbackLeftArmVel); //subscriber 
ros::Subscriber<geometry_msgs::Vector3> sub_leftarmPID1("/quori/waist/set_pid1", CallbackLeftArmSetPID1); //subscriber  
ros::Subscriber<geometry_msgs::Vector3> sub_leftarmposdir("/quori/waist/cmd_pos_dir", CallbackLeftArmPosDir); //subscriber 


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
  nh.subscribe(sub_leftarmvel);
  nh.subscribe(sub_leftarmposdir);
  nh.subscribe(sub_leftarmPID1);

  // Start SPI (MOSI=11, MISO=12, SCK=13)
  MLX90363::InitializeSPI(11,12,13);  // InitializeSPI only once for all sensors (ZXie)

  // Initialize UART serial ports
  Serial.begin(115200);   // for communication with PC
  Serial1.begin(115200);  // for motor 1 (inner)
  Serial3.begin(115200);  // for motor 2 (outter)
  
  delay(500);
  //update arm positions
  update_states();
  angleSensor2.SetZeroPosition(map(-2.966, -PI, PI, -8192, 8191));//TODO: set this gain
  angle_ctrl_client[0].timeout_.set(com[0],0.5);
  angle_ctrl_client[0].timeout_.save(com[0]);
  angle_ctrl_client[1].timeout_.set(com[1],0.5);
  angle_ctrl_client[1].timeout_.save(com[1]);
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
    //update 
    update_states();

    // send ROS Messages
    if ((micros()-last_update_msg_time)>UPDATE_MSG_TIME or 1){ // or 1 makes the telemetry excute every loop
      last_update_msg_time = micros();//
      ros_telemetry(); 
    }
    nh.spinOnce();
 
   // kill all motors if timeout. later it will check buffer for positions it may have run out of.
   if (control_mode == TRAJ){
      command_timeout = motor_1_pos_dt+CMD_TIMEOUT;
   }
    else{
      command_timeout = CMD_TIMEOUT;
    }
    
  if (AngleSafetyChecker ()){
    // coast motors
    coast_motor();
    state_data += "angle limit";
  }
  else if (micros() - last_command_time > command_timeout){
    coast_motor();
    cmd_timeout = 1;
    state_data += "cmd timeout";
  }
  else if (state_waist == COAST_STATE){
    coast_motor();
  }
  else if (state_waist == MOVE_STATE){
    cmd_timeout = 0;
    //Update arm
    move_motor();
    state_data += "moving waist";
  }
  }
}


/***************************/
/*          Support        */
/***************************/

// Prepares and sends ROS rotopic messages
void ros_telemetry(){

  lp3_msg.x= waist_pos;//joint_1_pos_meas;
  lp3_msg.y=joint_2_pos_meas;
  lp3_msg.z=0;
  pub_posLeft.publish(&lp3_msg);
  
  lm3_msg.x=motor_1_pos;
  lm3_msg.y=0;
  lm3_msg.z=0;
  pub_MLeft.publish(&lm3_msg);
  
  char charBuf[50];
  state_data.toCharArray(charBuf, 50); 
  pub_State.publish(&state_msg);
  state_data = " ";

  }

// Read and adjust the position of the arm joints.
void update_states(){
  angleSensor2.SendGET3();
  waist_pos =(int)angleSensor2.ReadAngle();
  
  // get motor positions. may not be nessesary now, but will help cath drive slip
  state_data += "getting_mpos";
  get_motor_pos(0);
}

//commands arms to move. Currently position and velocity modes are possible, but we only plan to use position.
void move_motor(){
  if (control_mode == POS){
    //position control code.
    // Load next knots in the buffer 
    set_motor_pos(0,motor_1_pos_cmd);
  }
  else if (control_mode == TRAJ){
    //trajectory control code.
    // Load next knots in the buffer 
    if (trajectory_state == NEW_TRAJ){
      set_motor_pos_dt(0,motor_1_pos_cmd,motor_1_pos_dt);
      trajectory_state = OLD_TRAJ;// NEW_TRAJ
    }
    
  }
  else if (control_mode == VEL ){
    // velocity control
    if (safe2move()){
      set_motor_speed(0,motor_1_vel_cmd);
    }
  }
  
}

//function place holder. TODO: write function that limits or saves arm from being cmded into a collision
bool safe2move(){
  /*float j_est = joint_1_pos_meas*6.283185+joint_1_vel_cmd*DT;// in radians
  if (j_est>LEFT_UPPER_LIMIT||j_est<LEFT_LOWER_LIMIT){
    return 0;
  }
  else{
    return 1; 
  }
  */
  return 1;// todo: figure out if this is needed for safety
}

// Checks if the arm is close to colliding with itself. If so, the arm is cmded to coast and function returns true;
bool AngleSafetyChecker (){
  if (((joint_2_pos_meas > JOINT_UPP_LIMIT) || (joint_2_pos_meas < JOINT_LOW_LIMIT)) && !override_safety){
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
void coast_motor(){
  set_motor_coast(0);
}

// Creates and sends message to a motor rloading a trajectory to follow.
void set_motor_pos(int id, float value)
{
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


void set_motor_pos_dt(int id, float value, float motor_id_pos_dt)
{
  angle_ctrl_client[id].trajectory_angular_displacement_.set(com[id],value);// trajectory cmd
  angle_ctrl_client[id].trajectory_duration_.set(com[id],motor_id_pos_dt);// ^
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

// Creates and sends a message to a motor requesting the motor reply with its position. If the reply is read sucessfully the value is stored and the function returns true.
bool get_motor_pos(int id)
{
  angle_ctrl_client[id].obs_angular_displacement_.get(com[id]);
  com[id].GetTxBytes(write_communication_buffer,write_communication_length);
  uint8_t *rx_data; // temporary pointer to received type+data bytes
  uint8_t rx_length; // number of received type+data bytes

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
    if(angle_ctrl_client[id].obs_angular_displacement_.IsFresh()) {
      motor_1_pos = angle_ctrl_client[id].obs_angular_displacement_.get_reply();
      state_data += "m1_got";
      return 1; //successful get
    }
    else{
      return 0;
    }

  
}


// if value <0 then ignore set
// Creates and sends message to a motor setting the angle cmd pid gains. This function probably takes <3ms to run
void set_motor_gains(int id, float kp, float ki, float kd)
{
  if (kp>=0){
    angle_ctrl_client[id].angle_Kp_.set(com[id], kp);
    angle_ctrl_client[id].angle_Kp_.save(com[id]);
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
  if (ki>=0){
    angle_ctrl_client[id].angle_Ki_.set(com[id], ki);
    angle_ctrl_client[id].angle_Ki_.save(com[id]);
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
  if (kd>=0){
    angle_ctrl_client[id].angle_Kd_.set(com[id], kd);
    angle_ctrl_client[id].angle_Kd_.save(com[id]);
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
}

// Creates and sends a message to a motor requesting a velocity be achieved by the motor.
void set_motor_speed(int id, float value)
{
  if (value>MAX_VEL){
    value = MAX_VEL;// limit the speed to only what the motor can achieve. currently its 100 rads/sec
  }
  else if (value<-100){
    value = -MAX_VEL;
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
