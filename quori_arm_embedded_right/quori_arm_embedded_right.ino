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
//AS5048A angleSensor1(9);  // sensor for joint 1 //Andrew- swapped these two to reflect current equations
//AS5048A angleSensor2(10);  // sensor for joint 2
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
char state_rightarm = 1;
#define COAST_STATE 1
#define MOVE_STATE 0

/***********************/
/*     ROS     zzh     */
/***********************/
ros::NodeHandle nh;

geometry_msgs::Vector3 lp3_msg;
ros::Publisher pub_posright("/quori/arm_right/pos_status", &lp3_msg);

geometry_msgs::Vector3 lm3_msg;
ros::Publisher pub_Mright("/quori/arm_right/motor_status", &lm3_msg);

geometry_msgs::Vector3 lmo3_msg;
ros::Publisher pub_MrightOffset("/quori/arm_right/motor_offset_status", &lmo3_msg);

geometry_msgs::Vector3 lmg3_msg;
ros::Publisher pub_MrightGoal("/quori/arm_right/motor_goal", &lmg3_msg);

std_msgs::String state_msg;
ros::Publisher pub_State("/quori/arm_right/state", &state_msg);


/******************************/
/* ROS callback functions zzh */
/******************************/
void CallbackrightArmPos (const geometry_msgs::Vector3& cmd_msg){
  // TODO: later this will take in a series of points and load it into a buffer
  
  if (cmd_msg.z <= -1){
    set_motor_coast(0);
    set_motor_coast(1);
    state_rightarm = COAST_STATE;
    state_data += "coasting request";
  }
  else{
    last_command_time=micros();
    state_rightarm = MOVE_STATE;
    motor_1_pos_dt  = cmd_msg.z;
    //if (motor_1_pos_dt <0.01){// catch timing input error. bound it by 10ms
    //  motor_1_pos_dt = 0.025;
    //}
    if ((cmd_msg.y>JOINT_2_UPP_LIMIT) || (cmd_msg.y<JOINT_2_LOW_LIMIT)){
      state_data += "bad position requested"; 
      }
    else{
      motor_1_pos_cmd = arm2motor_1(cmd_msg.x-joint_1_pos_sync_0,cmd_msg.y-joint_2_pos_sync_0)+motor_1_pos_offset;
      motor_2_pos_cmd = arm2motor_2(cmd_msg.x-joint_1_pos_sync_0,cmd_msg.y-joint_2_pos_sync_0)+motor_2_pos_offset;      
      //motor_1_pos_cmd = float(G_RATIO)*(cmd_msg.x  + cmd_msg.y);
      //motor_2_pos_cmd = float(G_RATIO)*(cmd_msg.x  - cmd_msg.y);
    }
    motor_2_pos_dt = motor_1_pos_dt;//TODO change the message type to have 4 slots instead of 3 so both times can be unique.
    control_mode = POS;
  } 
}

void CallbackrightArmPosDir (const geometry_msgs::Vector3& cmd_msg){
  // directly send value to motors
  
  if (cmd_msg.z <= -1){
    set_motor_coast(0);
    set_motor_coast(1);
    state_data += "coasting request";
    state_rightarm = COAST_STATE;
  }
  else{
    last_command_time=micros();
    state_rightarm = MOVE_STATE;
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


void CallbackSyncrightArmSlip (const std_msgs::Empty& cmd_msg){
  // reset angle of arm. TODO
  update_states();
  if (sync_slip_drive()){//check and run sync
    state_data += "slip_synced";
  }
  else{
    state_data += "slip_not_synced";  
  }
  
}


void CallbackrightArmSetPID2 (const geometry_msgs::Vector3& cmd_msg){
    set_motor_gains(1, cmd_msg.x, cmd_msg.y, cmd_msg.z);

}
void CallbackrightArmSetPID1 (const geometry_msgs::Vector3& cmd_msg){
    set_motor_gains(0, cmd_msg.x, cmd_msg.y, cmd_msg.z);

}

void CallbackSyncrightArmOverride (const std_msgs::Bool& cmd_msg){
    override_safety = cmd_msg.data;

}

void CallbackrightArmSetZero (const std_msgs::Empty& msg) {
  angleSensor1.SetZeroPosition();
  angleSensor2.SetZeroPosition();
}

void CallbackrightArmSetZeroPos (const geometry_msgs::Vector3& msg) {
  int16_t offsert1 = map(msg.x, -PI, PI, -8192, 8191);
  int16_t offsert2 = map(msg.y, -PI, PI, -8192, 8191);
  angleSensor1.SetZeroPosition(offsert1);
  angleSensor2.SetZeroPosition(offsert2);
}


ros::Subscriber<geometry_msgs::Vector3> sub_rightarmpos("/quori/arm_right/cmd_pos", CallbackrightArmPos); //subscriber 
ros::Subscriber<std_msgs::Empty> sub_rightarmsync("/quori/arm_right/sync_slip", CallbackSyncrightArmSlip); //
ros::Subscriber<geometry_msgs::Vector3> sub_rightarmPID1("/quori/arm_right/set_pid1", CallbackrightArmSetPID1); //subscriber  
ros::Subscriber<geometry_msgs::Vector3> sub_rightarmPID2("/quori/arm_right/set_pid2", CallbackrightArmSetPID2); //subscriber 
ros::Subscriber<geometry_msgs::Vector3> sub_rightarmposdir("/quori/arm_right/cmd_pos_dir", CallbackrightArmPosDir); //subscriber 
ros::Subscriber<std_msgs::Bool> sub_rightarmoverride("/quori/arm_right/limit_override", CallbackSyncrightArmOverride); //
ros::Subscriber<std_msgs::Empty> sub_setzero("/quori/arm_right/setzero", CallbackrightArmSetZero);
ros::Subscriber<geometry_msgs::Vector3> sub_setzeropos("/quori/arm_right/setzeropos", CallbackrightArmSetZeroPos);


/***************************************************/
/*                                                 */
/*                      SETUP                      */
/*                                                 */
/***************************************************/
void setup()
{
  //ROS
  nh.initNode();
  nh.advertise(pub_posright);
  nh.advertise(pub_Mright);
  nh.advertise(pub_MrightOffset);
  nh.advertise(pub_MrightGoal);
  nh.advertise(pub_State);
  nh.subscribe(sub_rightarmpos);
  nh.subscribe(sub_rightarmposdir);
  nh.subscribe(sub_rightarmsync);
  nh.subscribe(sub_rightarmPID1);
  nh.subscribe(sub_rightarmPID2);
  nh.subscribe(sub_rightarmoverride);
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

  angleSensor1.SetZeroPosition(map(-0.342, -PI, PI, -8192, 8191));
  angleSensor2.SetZeroPosition(map(2.496, -PI, PI, -8192, 8191));

  if (sync_slip_drive()){
    Serial.println("System initialized !");  
    state_data += "slip_synced";
  }
  else{
    Serial.println("System not fully initalized. slip drive not synced !");
    state_data += "slip_not_synced";  
  }
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
 
   // kill all motors if timeout. later it will check buffer for positions it may have run out of.
  if (shoulderAngleLimiter ()){
    //shoulderAngleLimiter will coast motors
    state_data += "angle limit";
  }
  else if (micros() - last_command_time > command_timeout){
    coast_right_arm();
    cmd_timeout = 1;
    state_data += "cmd timeout";
  }
  else if (state_rightarm == COAST_STATE){
    coast_right_arm();
  }
  else if (state_rightarm == MOVE_STATE){
    cmd_timeout = 0;
    //Update arm
    move_arms();
    state_data += "moving arms";
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
  pub_posright.publish(&lp3_msg);
  
  lm3_msg.x=motor_1_pos;
  lm3_msg.y=motor_2_pos;
  lm3_msg.z=0;
  pub_Mright.publish(&lm3_msg);

  lmo3_msg.x=motor_1_pos_offset;
  lmo3_msg.y=motor_2_pos_offset;
  lmo3_msg.z=0;
  pub_MrightOffset.publish(&lmo3_msg);

  lmg3_msg.x=motor_1_pos_cmd;//see set_motor command
  lmg3_msg.y=motor_2_pos_cmd;
  lmg3_msg.z=0;
  pub_MrightGoal.publish(&lmg3_msg);
  
  char charBuf[50];
  state_data.toCharArray(charBuf, 50); 
  state_msg.data = charBuf;
  pub_State.publish(&state_msg);
  state_data = " ";

  }

// Read and adjust the position of the arm joints.
void update_states(){
//  joint_1_pos_meas =(int)angleSensor1.getRotation();
//  joint_2_pos_meas =(int)angleSensor2.getRotation();
 // joint_1_pos_meas =(int)angleSensor1.getRawRotation();// for calibration
  //joint_2_pos_meas =(int)angleSensor2.getRawRotation();//^
  angleSensor1.SendGET3();
  angleSensor2.SendGET3();
  joint_1_pos_meas =(int)angleSensor1.ReadAngle();
  joint_2_pos_meas =(int)angleSensor2.ReadAngle();
  
  //TODO

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
  // catch and correct for slip.
  //sync_slip_drive();
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
    if (safe2moveright()){
      set_motor_speed(0,arm2motor_1(joint_1_vel_cmd,joint_2_vel_cmd));
      set_motor_speed(1,arm2motor_2(joint_1_vel_cmd,joint_2_vel_cmd));
    }
  }
  
}

//function place holder. TODO: write function that limits or saves arm from being cmded into a collision
bool safe2moveright(){
  /*float j_est = joint_1_pos_meas*6.283185+joint_1_vel_cmd*DT;// in radians
  if (j_est>right_UPPER_LIMIT||j_est<right_LOWER_LIMIT){
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
    coast_right_arm();
    return 1;
  }
  else{
    return 0;
  }
}

float joint_angle_trans(float angle) {
  if (angle > PI) return joint_angle_trans(angle - 2 * PI);
  else if (angle <= -PI) return joint_angle_trans(angle + 2 * PI);
  else return angle;
}

// Function used to convert from arm joint goal, alpha is J1, beta is felxion(windmill) is J2 abduction(flapping) to motor goal.
float arm2motor_1(float alpha, float beta){
  //converts the arm axis to the motors axis with gear ratios. can be used for position or velocity
       alpha = joint_angle_trans(alpha);
       return G_RATIO1*(alpha  + beta);
       //return alpha * G_ratio - beta * G_ratio;
}

// Function used to convert from arm joint goal to motor goal.
float arm2motor_2(float alpha, float beta){
  //converts the arm axis to the motors axis with gear ratios.  can be used for position or velocity
       alpha = joint_angle_trans(alpha);
       //return alpha * G_RATIO + beta * G_RATIO;
       return G_RATIO2*(alpha - beta);
}

// Function gets the current position of the arm and updates the offset for the relationship between the motors and the arm
// Make sure the update state cmd is run before this otherwise the data will be old
bool sync_slip_drive(){
  //gets the current position of the arm and updates the offset for the relationship between the motors and the arm
  bool sync_state = 0;
  sync_state = get_motor_pos(0);
  sync_state = sync_state & get_motor_pos(1);
  joint_1_pos_sync_0 = joint_1_pos_meas;
  joint_2_pos_sync_0 = joint_2_pos_meas;
  motor_1_pos_offset = motor_1_pos; 
  motor_2_pos_offset = motor_2_pos;
  return sync_state;
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

// Wrapper function to coast both right arm motors
void coast_right_arm(){
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
      if (debug_cmd){
        //delay(READ_DELAY);
        angle_ctrl_client[id].trajectory_angular_displacement_.get(com[id]);// trajectory cmd
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
        if(angle_ctrl_client[id].trajectory_angular_displacement_.IsFresh()) {
          lmg3_msg.x = angle_ctrl_client[id].trajectory_angular_displacement_.get_reply();
        }        
      }
      break;
    case 1:
      Serial3.write((uint8_t*)write_communication_buffer,sizeof(uint8_t[static_cast<int>(write_communication_length)]));
       if (debug_cmd){
        //delay(READ_DELAY);
        angle_ctrl_client[id].trajectory_angular_displacement_.get(com[id]);// trajectory cmd
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
        if(angle_ctrl_client[id].trajectory_angular_displacement_.IsFresh()) {
          lmg3_msg.y = angle_ctrl_client[id].trajectory_angular_displacement_.get_reply();
        }        

      }
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
      if (debug_cmd){
        delay(READ_DELAY);
        //angle_ctrl_client[id].trajectory_angular_displacement_.get(com[id]);// trajectory cmd
        angle_ctrl_client[id].ctrl_angle_.get(com[id]);// trajectory cmd
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
        //if(angle_ctrl_client[id].trajectory_angular_displacement_.IsFresh()) {
        //  lmg3_msg.x = angle_ctrl_client[id].trajectory_angular_displacement_.get_reply();
        //}        
        if(angle_ctrl_client[id].ctrl_angle_.IsFresh()) {
          lmg3_msg.x = angle_ctrl_client[id].ctrl_angle_.get_reply();
        }
      }
      break;
    case 1:
      Serial3.write((uint8_t*)write_communication_buffer,sizeof(uint8_t[static_cast<int>(write_communication_length)]));
       if (debug_cmd){
        delay(READ_DELAY);
        //angle_ctrl_client[id].trajectory_angular_displacement_.get(com[id]);// trajectory cmd
        angle_ctrl_client[id].ctrl_angle_.get(com[id]);// trajectory cmd
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
        //if(angle_ctrl_client[id].trajectory_angular_displacement_.IsFresh()) {
        //  lmg3_msg.x = angle_ctrl_client[id].trajectory_angular_displacement_.get_reply();
        //}        
        if(angle_ctrl_client[id].ctrl_angle_.IsFresh()) {
          lmg3_msg.x = angle_ctrl_client[id].ctrl_angle_.get_reply();
        }
      }
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
          state_data += "m1_got";
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
          state_data += "m2_got";
          return 1; //successful get
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
