
/*
 * This code controls the position of a motor to move the waist of the robot. The code will prevent the motor from moving if the waist is too close to colliding with the structure.
 */
#define USE_USBCON
#include <Arduino.h>                              // required before wiring_private.h
#include <wiring_private.h>
#include "src/MLX90363/MLX90363.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

#include "pid.h"
#include "motor.h"
#include "constants.h"
#include "additional_serial.h"
#include "LpfButter1.h"
#include "pid_linear.hpp"


/**********************/
/*       Sensor       */
/**********************/
MLX90363 angle_sensor_waist(9); // waist shaft
MLX90363 angle_sensor_MT(10); //motor shaft
int decoder_count_MT = 0;

/******************/
/* Desired Values */
/******************/

// motor globals
float desired_MT_pos      = 0; //radians
float prev_desired_MT_pos = 0; //radians

float desired_MT_vel = 0; //radians/sec


/******************/
/* Set Speed Vars */
/******************/
float MT_volts_cmd = 0;

int serial_MT = 0;

// motor tags
int turret_motor_tag = 1;


/**********************/
/* Control Parameters */
/**********************/
// Low-level PID

PidLinear pos_MT_pid;

//Butterworth filter for each motor. This value should be tuned.
LpfButter1 mt_filter_position(10,LOOPHZ);

/************************/
/*        Timing        */
/************************/
unsigned long last_command_time     = 0;
unsigned long last_recorded_time    = 0;
unsigned long last_update_msg_time  = 0;

/************************/
/*    Timing            */
/************************/
float time_elapsed_micros; // microseconds
float time_elapsed_millis; // time_elapsed converted
float time_elapsed_secs;   //time elapsed in seconds
float time_elapsed_secs_inv;   //time elapsed in seconds
int loop_time_duration; // Timing loop time- for performance testing


/***************************/
/*    State Variables      */
/***************************/

float waist_pos   = 0; // radians
int   waist_count = 0; //encoder raw value

float motorKp = 0;
float motorKd = 0;
float motorKi = 0;

float motor_pos = 0; //radians

char control_mode = 1;// 0 is position 1 is speed
bool override_safety = 0;

String state_data = "booting";
char state_waist = COAST_STATE;


/***********************/
/*     ROS             */
/***********************/
ros::NodeHandle nh;

geometry_msgs::Vector3 lp3_msg;
ros::Publisher pub_posLeft("/quori/waist/pos_status", &lp3_msg);

geometry_msgs::Vector3 lm3_msg;
ros::Publisher pub_MLeft("/quori/waist/motor_status", &lm3_msg);

std_msgs::String state_msg;
ros::Publisher pub_State("/quori/waist/state", &state_msg);

/******************************/
/* ROS callback functions     */
/******************************/

void CallbackLeftArmPos (const geometry_msgs::Vector3& cmd_msg){
  // directly send value to motors
  
  if (cmd_msg.z <= -1){
    state_data += "coasting request";
    state_waist = COAST_STATE;
    last_command_time=micros();
    
  }
  else{
    last_command_time=micros();
    state_waist = MOVE_STATE;
    //if (((cmd_msg.x > MOTOR_UPP_LIMIT) || (cmd_msg.x< MOTOR_LOW_LIMIT))){
    //  state_data += "position out of limits";
    //  state_waist = COAST_STATE;
   // }
   // else{
       state_waist = MOVE_STATE;
       state_data += "cmd_recieved";
       update_motor_goal(cmd_msg.x, cmd_msg.y);

   // } 
  }
}


void CallbackLeftArmSetPID1 (const geometry_msgs::Vector3& cmd_msg){
    pos_MT_pid.set_Kp(cmd_msg.x);
    pos_MT_pid.set_Ki(cmd_msg.y);
    pos_MT_pid.set_Kd(cmd_msg.z);

}

ros::Subscriber<geometry_msgs::Vector3> sub_leftarmPID1("/quori/waist/set_pid1", CallbackLeftArmSetPID1); //subscriber  
ros::Subscriber<geometry_msgs::Vector3> sub_leftarmposdir("/quori/waist/cmd_pos_dir", CallbackLeftArmPos); //subscriber 


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
  nh.subscribe(sub_leftarmPID1);

  // Start SPI (MOSI=11, MISO=12, SCK=13)
  MLX90363::InitializeSPI(11,12,13);

  // Initialize UART serial ports
  Serial.begin(115200);   // for communication with PC
  serial_setup();
  
  delay(500);
  //update sensor position Zero
  update_states();
  angle_sensor_waist.SetZeroPosition(map(1.55, -PI, PI, -8192, 8191));//TODO: Set this for each robot
  angle_sensor_MT.SetZeroPosition(map(0.767, -PI, PI, -8192, 8191));//TODO: Set this for each robot ...-1.91384649277

  pos_MT_pid.set_Kp(1.0);
  pos_MT_pid.set_Ki(0.05);
  pos_MT_pid.set_Kd(0.03);
  pos_MT_pid.set_saturation(1.0);// percent of motor voltage
  pos_MT_pid.set_deadband(0.0);// percent of motor voltage
  pos_MT_pid.set_feed_forward(0.0);
  Serial.flush();
}

/***************************************************/
/*                                                 */
/*                      MAIN                       */
/*                                                 */
/***************************************************/
void loop(){
  loop_time_duration = micros() - last_recorded_time;
  if (loop_time_duration >= LOOPTIME) { // ensures stable loop time
    checkLoopTimeError();
    time_elapsed_secs = ((float)loop_time_duration) * 0.000001;
    time_elapsed_secs_inv = 1.0/time_elapsed_secs;// if loop is stable enough and readings are not too sensitive then comment this out and replace with contants DT and DT_INV

    last_recorded_time = micros();
    //
    update_states();

    // send ROS Messages
    if ((micros()-last_update_msg_time)>UPDATE_MSG_TIME or 1){ // or 1 makes the telemetry excute every loop
      last_update_msg_time = micros();//
      ros_telemetry();
    }
    nh.spinOnce();
    
  if (safe2move()){   
    if ((state_waist == MOVE_STATE) & !cmdTimedout()){
      move_motor();
      state_data += "moving waist";
    }
    else{
    set_motor_coast();
    }
  }
  else{
    set_motor_coast();
  }
 }
}


/***************************/
/*          Support        */
/***************************/

void checkLoopTimeError(){
  if (loop_time_duration > LOOPTIME+100) {
    state_data += "loop time error";
    state_data += String(loop_time_duration,DEC);
  }
}

float count2rad(int count){
  return (float)count*3.1415/8191.0;
}

// Prepares and sends ROS rotopic messages
void ros_telemetry(){

  lp3_msg.x= waist_pos;//joint_1_pos_meas;
  lp3_msg.y=0 ;
  lp3_msg.z=0;
  pub_posLeft.publish(&lp3_msg);
  
  lm3_msg.x=motor_pos;
  lm3_msg.y=MT_volts_cmd;// for debugging
  lm3_msg.z=0;
  pub_MLeft.publish(&lm3_msg);
  
  char charBuf[50];
  state_data.toCharArray(charBuf, 50); 
  state_msg.data = charBuf;
  pub_State.publish(&state_msg);
  state_data = " ";
  

  }


// Read and adjust the position of the arm joints.
void update_states(){
  //
  angle_sensor_waist.SendGET3();
  waist_count =-(int)angle_sensor_waist.ReadAngle();
  waist_pos = count2rad(waist_count);

  // 
   angle_sensor_MT.SendGET3();
   decoder_count_MT = -(int)angle_sensor_MT.ReadAngle();
   motor_pos = mt_filter_position.sample(count2rad(decoder_count_MT));

   //
}

//commands arms to move. Currently position and velocity modes are possible, but we only plan to use position.
void move_motor(){
    set_motor_control(); // uncomment this and replace delete other code
    
}

//
bool safe2move(){
 if (override_safety) {
  return 1; 
 }
 else if (((waist_pos > JOINT_UPP_LIMIT) || (waist_pos< JOINT_LOW_LIMIT))){
    state_data += "joint limit reached"+String(waist_pos);
    return 0;
 }
  else{
    return 1;
  }
}

bool cmdTimedout(){
 if (micros()-last_command_time>CMD_TIMEOUT){
  state_data += "cmd_timedout";
  return 1;
 }
 else{
  return 0;
 }
  
}

// use this to prevent the velocity goal esitmate from not being handled properly
void update_motor_goal(float goal_pos, float goal_vel){
  prev_desired_MT_pos = desired_MT_pos;
  desired_MT_pos = goal_pos;
  desired_MT_vel = goal_vel;
}

/***************************/
/*   IQ-Motor Functions    */
/***************************/

// Creates and sends message to motors requesting a coast cmd.
void set_motor_coast()
{
 MT_volts_cmd = 0;
 pos_MT_pid.Reset();
 set_volts(&MT_volts_cmd, &serial_MT);
}

void set_motor_control() {
    pos_MT_pid.set_reference(desired_MT_pos);
    pos_MT_pid.set_reference_dot(desired_MT_vel);
    //pos_MT_pid.set_reference_dot((desired_MT_pos-prev_desired_MT_pos)*time_elapsed_secs_inv);//approximation
    //float MT_feed_fwd = desired_MT_pos * 0.004022 + 0.0175;
    //pos_MT_pid.set_feed_forward(MT_feed_fwd);
    MT_volts_cmd = pos_MT_pid.PidCompute(motor_pos, time_elapsed_secs, time_elapsed_secs_inv);
 
    set_volts(&MT_volts_cmd, &serial_MT);
}
