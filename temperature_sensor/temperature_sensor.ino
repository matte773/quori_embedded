#include <ros.h>
#include <geometry_msgs/Vector3.h>

const int temperaturePin1 = 0;
const int temperaturePin2 = 1;

ros::NodeHandle nh;
geometry_msgs::Vector3 temp_msg;
ros::Publisher pub_motor_temp("/quori/motor_temp", &temp_msg);

void setup()
{
  Serial.begin(9600);
  pinMode(temperaturePin1, INPUT);
  pinMode(temperaturePin2, INPUT);
  nh.advertise(pub_motor_temp);
}


void loop()
{
  // sensor 1
  float voltage1, degreesC1, degreesF1;

  voltage1 = getVoltage(temperaturePin1);

  degreesC1 = (voltage1 - 0.5) * 100.0;

  degreesF1 = degreesC1 * (9.0/5.0) + 32.0;

  Serial.print("Sensor 1: voltage: ");
  Serial.print(voltage1);
  Serial.print("  deg C: ");
  Serial.print(degreesC1);
  Serial.print("  deg F: ");
  Serial.println(degreesF1);

  // sensor 2

  float voltage2, degreesC2, degreesF2;

  voltage2 = getVoltage(temperaturePin2);

  degreesC2 = (voltage2 - 0.5) * 100.0;

  degreesF2 = degreesC2 * (9.0/5.0) + 32.0;

  Serial.print("Sensor 2: voltage: ");
  Serial.print(voltage2);
  Serial.print("  deg C: ");
  Serial.print(degreesC2);
  Serial.print("  deg F: ");
  Serial.println(degreesF2);


  // send message

  temp_msg.x=degreesF1;

  temp_msg.y=degreesF2;

  temp_msg.z=0;
  
  pub_motor_temp.publish(&temp_msg);

  nh.spinOnce();

  delay(100); // repeat once per 0.1 second
}


float getVoltage(int pin)
{
  return (analogRead(pin) * 0.004882814);
}

