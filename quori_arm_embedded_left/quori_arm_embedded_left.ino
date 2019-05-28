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
#include <SPI.h>

#define UPDATE_MSG_TIME 10000
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
#define JOINT_2_UPP_LIMIT  1.35
#define JOINT_2_LOW_LIMIT -1.35
#define SENSOR_MAX 16383.0
#define SENSOR_HALF 8191.5
#define JOINT_1_CALI 12856
#define MAX_I 0.225/0.1447 //max torque/1.447. this is an estimate. The correlation might not hold at higher speeds


// IQinetics Library
#include "src/libiqinetics/inc/bipbuffer.h"
#include "src/libiqinetics/inc/communication_interface.h"
#include "src/libiqinetics/inc/byte_queue.h"
#include "src/libiqinetics/inc/packet_finder.h"
#include "src/libiqinetics/inc/crc_helper.h"
#include "src/libiqinetics/inc/generic_interface.hpp"
#include "src/libiqinetics/inc/safe_brushless_drive_client.hpp"
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

SafeBruhslessDriveClient safe_drive_client[2] = {
   SafeBruhslessDriveClient(0),
   SafeBruhslessDriveClient(0)
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

float joint_1_pos_meas,joint_2_pos_meas; //measured input value 0 to 1


float joint1_max_over = 0;// joint overflow limit varibles
float joint1_min_over = 0;

float motor_1_vel_cmd = 0;//radians/second
float motor_2_vel_cmd = 0;//^

float motor_1_current = 0;
float  motor_2_current = 0;

float motor_max_I = MAX_I;

float motor_1_pos = 0;
float  motor_2_pos = 0;
float motor_1_pos_cmd = 0;
float motor_2_pos_cmd = 0;
float motor_1_pos_dt = 0.01;
float motor_2_pos_dt = 0.01;
bool cmd_timeout = 0;
bool control_mode = 1;// 0 is position 1 is speed
bool override_safety = 0;
bool motor_max_request = 0;
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

//geometry_msgs::Vector3 lmg3_msg;
//ros::Publisher pub_MlefttGoal("/quori/arm_left/motor_goal", &lmg3_msg);

geometry_msgs::Vector3 lmc3_msg;
ros::Publisher pub_MleftCurrent("/quori/arm_left/current", &lmc3_msg);


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


void CallbackLeftArmSetCurrentLimit (const geometry_msgs::Vector3& cmd_msg){
    motor_max_I = cmd_msg.x;
    motor_max_request = 1;

}

void CallbackLeftArmSetPID2 (const geometry_msgs::Vector3& cmd_msg){
    set_motor_gains(1, cmd_msg.x, cmd_msg.y, cmd_msg.z);

}
void CallbackLeftArmSetPID1 (const geometry_msgs::Vector3& cmd_msg){
    set_motor_gains(0, cmd_msg.x, cmd_msg.y, cmd_msg.z);

}

void CallbackSyncLeftArmOverride (const std_msgs::Bool& cmd_msg){
    override_safety = cmd_msg.data;

}

void CallbackLeftArmSetZero (const std_msgs::Empty& msg) {
  angleSensor1.SetZeroPosition();
  angleSensor2.SetZeroPosition();
}

void CallbackLeftArmSetZeroPos (const geometry_msgs::Vector3& msg) {
  int16_t offsert1 = map(msg.x, -PI, PI, -8192, 8191);
  int16_t offsert2 = map(msg.y, -PI, PI, -8192, 8191);
  angleSensor1.SetZeroPosition(offsert1);
  angleSensor2.SetZeroPosition(offsert2);
}


ros::Subscriber<geometry_msgs::Vector3> sub_leftarmCur("/quori/arm_left/set_Current_Limit", CallbackLeftArmSetCurrentLimit); //
ros::Subscriber<geometry_msgs::Vector3> sub_leftarmPID1("/quori/arm_left/set_pid1", CallbackLeftArmSetPID1); //subscriber  
ros::Subscriber<geometry_msgs::Vector3> sub_leftarmPID2("/quori/arm_left/set_pid2", CallbackLeftArmSetPID2); //
ros::Subscriber<geometry_msgs::Vector3> sub_leftarmposdir("/quori/arm_left/cmd_pos_dir", CallbackLeftArmPosDir); //
ros::Subscriber<std_msgs::Bool> sub_leftarmoverride("/quori/arm_left/limit_override", CallbackSyncLeftArmOverride); //
ros::Subscriber<std_msgs::Empty> sub_setzero("/quori/arm_left/setzero", CallbackLeftArmSetZero);
ros::Subscriber<geometry_msgs::Vector3> sub_setzeropos("/quori/arm_left/setzeropos", CallbackLeftArmSetZeroPos);


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
//  nh.advertise(pub_MleftGoal);
  nh.advertise(pub_MleftCurrent);
  
  nh.subscribe(sub_leftarmCur);
  nh.subscribe(sub_leftarmposdir);
  nh.subscribe(sub_leftarmPID1);
  nh.subscribe(sub_leftarmPID2);
  nh.subscribe(sub_leftarmoverride);
  nh.subscribe(sub_setzero);
  nh.subscribe(sub_setzeropos);

  // Start SPI (MOSI=11, MISO=12, SCK=13)
  MLX90363::InitializeSPI(11,12,13);  // InitializeSPI only once for all sensors (ZXie)
//  angleSensor1.setZeroPosition(5033);  //(3295) Set zero positions based on calibration
//  angleSensor2.setZeroPosition(13735);

  // Initialize UART serial ports
  Serial.begin(115200);   // for communication with PC
  Serial1.begin(115200);  // for motor 1 (inner)
  Serial3.begin(115200);  // for motor 2 (outter)
  
  delay(500);
  //update arm positions
  update_states();

  angleSensor1.SetZeroPosition(map(0.663, -PI, PI, -8192, 8191));
  angleSensor2.SetZeroPosition(map(0.586, -PI, PI, -8192, 8191));

  while ((get_motor_maxI_(0)== MAX_I) && (get_motor_maxI_(1) == MAX_I)){
    send_max_I_motor_msg(0,MAX_I);
    send_max_I_motor_msg(1,MAX_I);
  }
  state_data += "current_limit_set";

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
    //update arm positions
    update_states();

    // send ROS Messages
    if ((micros()-last_update_msg_time)>UPDATE_MSG_TIME or 1){ // or 1 makes the telemetry excute every loop
    last_update_msg_time = micros();//
    ros_telemetry(); 
    }
    nh.spinOnce();
  if (motor_max_request){
    int count_out = 0;
    while ( count_out <=10 && !((abs(get_motor_maxI_(0)-motor_max_I)<0.01) && (abs(get_motor_maxI_(1)-motor_max_I)<0.01) )){// change to exit logic after first successful message
      safe_drive_client[0].motor_I_max_.set(com[0],motor_max_I);
      send_max_I_motor_msg(0,motor_max_I);
      send_max_I_motor_msg(1,motor_max_I);
      count_out = count_out+1;
    }
    motor_max_request = 0;
    state_data += "current_limit_set";
    lmc3_msg.z= get_motor_maxI_(0);
  }
   // kill all motors if timeout. later it will check buffer for positions it may have run out of.
  if (shoulderAngleLimiter ()){
    //shoulderAngleLimiter will coast motors
    state_data += "angle limit";
  }
  else if (micros() - last_command_time > command_timeout){
    coast_left_arm();
    cmd_timeout = 1;
    state_data += "cmd timeout";
  }
  else if (state_leftarm == COAST_STATE){
    coast_left_arm();
  }
  else if (state_leftarm == MOVE_STATE){
    cmd_timeout = 0;
    //Update arm
    move_arms();
    //state_data += "moving arms";
  }
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
  lm3_msg.z=0;
  pub_MLeft.publish(&lm3_msg);

  lmc3_msg.x=motor_1_current;
  lmc3_msg.y=motor_2_current;
  //lmc3_msg.z=0;
  pub_MleftCurrent.publish(&lmc3_msg);
  
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
  
  if (joint_1_pos_meas>=0){
    joint_1_pos_meas = joint_1_pos_meas/TICKS_PER_REV;
  }
  else{
    //joint_1_pos_meas = 1.0+joint_1_pos_meas/TICKS_PER_REV;
    joint_1_pos_meas = joint_1_pos_meas/TICKS_PER_REV;
  }
  //joint_1_pos_meas = 1.0-joint_1_pos_meas;// corrects for reference frame

  if (joint_2_pos_meas>=0){
    joint_2_pos_meas = joint_2_pos_meas/TICKS_PER_REV;
  }
  else{
    //joint_2_pos_meas = 1.0+joint_2_pos_meas/TICKS_PER_REV;
    joint_2_pos_meas = joint_2_pos_meas/TICKS_PER_REV;
  }
  //convert to radians
  joint_1_pos_meas = joint_1_pos_meas*2*PI;
  joint_2_pos_meas = joint_2_pos_meas*2*PI;
  
  //to reverse direction pos = 1-pos;

  // get motor positions. may not be nessesary now, but will help cath drive slip
  state_data += "getting_mpos";
  get_motor_pos(0);
  get_motor_pos(1);
  get_motor_current(0);
  get_motor_current(1);
  // get updated cmds from buffer. prep new commands to be sent

  
}

//commands arms to move. Currently position and velocity modes are possible, but we only plan to use position.
void move_arms(){
  if (control_mode == POS){
    //position control code.
    // Load next knots in the buffer 
    set_motor_pos(0,motor_1_pos_cmd,motor_1_pos_dt);
    set_motor_pos(1,motor_2_pos_cmd,motor_2_pos_dt);
  }
  else if (control_mode == VEL ){
    // velocity control
    if (safe2moveLeft()){
      set_motor_speed(0,motor_1_vel_cmd);
      set_motor_speed(1,motor_2_vel_cmd);
    }
  }
  
}

//function place holder. TODO: write function that limits or saves arm from being cmded into a collision
bool safe2moveLeft(){
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

void send_max_I_motor_msg(int id, float value)
{
  safe_drive_client[id].motor_I_max_.set(com[id],value);
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
  
  bool debug_cmd = 0;
  uint8_t *rx_data; // temporary pointer to received type+data bytes
  uint8_t rx_length; // number of received type+data bytes
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
  //angle_ctrl_client[id].trajectory_angular_displacement_.set(com[id],value);// trajectory cmd
  //angle_ctrl_client[id].trajectory_duration_.set(com[id],motor_id_pos_dt);// ^
  angle_ctrl_client[id].ctrl_angle_.set(com[id],value); // position control with no duration or speed set. Use either the two trajectory cmds or this
  com[id].GetTxBytes(write_communication_buffer,write_communication_length);
  
  bool debug_cmd = 1;
  uint8_t *rx_data; // temporary pointer to received type+data bytes
  uint8_t rx_length; // number of received type+data bytes
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
        if(angle_ctrl_client[id].obs_angular_displacement_.IsFresh()) {
          motor_1_pos = angle_ctrl_client[id].obs_angular_displacement_.get_reply();
          //state_data += "m1_got";
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
        if(angle_ctrl_client[id].obs_angular_displacement_.IsFresh()) {
          motor_2_pos = angle_ctrl_client[id].obs_angular_displacement_.get_reply();
          //state_data += "m2_got";
          return 1; //successful get
        }
        else{
          return 0;
        }
        break;
  }
  return 0;
}

// Creates and sends a message to a motor requesting the motor reply with its estimated torque. If the reply is read sucessfully the value is stored and the function returns true.
bool get_motor_current(int id)
{
  
  safe_drive_client[id].est_motor_amps_.get(com[id]);
  com[id].GetTxBytes(write_communication_buffer,write_communication_length);
  uint8_t *rx_data; // temporary pointer to received type+data bytes
  uint8_t rx_length; // number of received type+data bytes
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
        safe_drive_client[id].ReadMsg(com[id],rx_data,rx_length);
        // Once we’re done with the message packet, drop it
        com[id].DropPacket();
      }
        // Check if we have any fresh data
        // Checking for fresh data is not required, it simply
        // lets you know if you received a message that you
        // have not yet read.
        if(safe_drive_client[id].est_motor_amps_.IsFresh()) {
          motor_1_current = safe_drive_client[id].est_motor_amps_.get_reply();
          state_data += "m1c_got";
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
        safe_drive_client[id].ReadMsg(com[id],rx_data,rx_length);
        // Once we’re done with the message packet, drop it
        com[id].DropPacket();
      }
        // Check if we have any fresh data
        // Checking for fresh data is not required, it simply
        // lets you know if you received a message that you
        // have not yet read.
        if(safe_drive_client[id].est_motor_amps_.IsFresh()) {
          motor_2_current = safe_drive_client[id].est_motor_amps_.get_reply();
          state_data += "m2c_got";
          return 1; //successful get
        }
        else{
          return 0;
        }
        break;
  }
  return 0;
}

// Creates and sends a message to a motor requesting the motor reply with its max current limit. If the reply is read sucessfully the value is stored and the function returns true.
float get_motor_maxI_(int id)
{
  
  safe_drive_client[id].motor_I_max_.get(com[id]);
  com[id].GetTxBytes(write_communication_buffer,write_communication_length);
  uint8_t *rx_data; // temporary pointer to received type+data bytes
  uint8_t rx_length; // number of received type+data bytes
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
        safe_drive_client[id].ReadMsg(com[id],rx_data,rx_length);
        // Once we’re done with the message packet, drop it
        com[id].DropPacket();
      }
        // Check if we have any fresh data
        // Checking for fresh data is not required, it simply
        // lets you know if you received a message that you
        // have not yet read.
        if(safe_drive_client[id].motor_I_max_.IsFresh()) {
          state_data += "m1I_got";
          return safe_drive_client[id].motor_I_max_.get_reply(); //successful get
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
        safe_drive_client[id].ReadMsg(com[id],rx_data,rx_length);
        // Once we’re done with the message packet, drop it
        com[id].DropPacket();
      }
        // Check if we have any fresh data
        // Checking for fresh data is not required, it simply
        // lets you know if you received a message that you
        // have not yet read.
        if(safe_drive_client[id].motor_I_max_.IsFresh()) {
          state_data += "m2I_got";
          return safe_drive_client[id].motor_I_max_.get_reply(); //successful get
        }
        else{
          return 0;
        }
        break;
  }
  return 0;
}

// if value <0 then ignore set
// Creates and sends message to a motor setting the angle cmd pid gains. This function probably takes <3ms to run
void set_motor_gains(int id, float kp, float ki, float kd)
{
  if (kp>=0){
    angle_ctrl_client[id].angle_Kp_.set(com[id], kp);
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
