/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

// Use the following line if you have a Leonardo or MKR1000
//#define USE_USBCON

#include <ros.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle nh;

geometry_msgs::Vector3 str_msg;
ros::Publisher User_Right_Arm_Goal("/quori/arm_right/joint_goal", &str_msg);

// Define read pin
int arm_read_pin = A0;
int read_val_raw = 0;
float out_angle = 0;
float min_out_angle = -1.3;
float max_out_angle = 1.3;

void setup() {
  nh.initNode();
  nh.advertise(User_Right_Arm_Goal);
  Serial.begin(115200);
}

void loop() {

  // Read puppet angle
  read_val_raw = analogRead(arm_read_pin);
  out_angle = map(read_val_raw, 0, 1023, -5*PI/3, 5*PI/3);
  
  if(out_angle <= min_out_angle) {
    out_angle = min_out_angle;
  } else if(out_angle >= max_out_angle) {
    out_angle = max_out_angle;
  }
  
  str_msg.x = out_angle;
  str_msg.y = 0;
  str_msg.z = 0;
  
  User_Right_Arm_Goal.publish( &str_msg );
  nh.spinOnce();
  delay(50);
}
