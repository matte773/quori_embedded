#include "src/iq_module_communicaiton.hpp"
IqSerial ser1(Serial1);
IqSerial ser3(Serial3);
SerialInterfaceClient sic(0);
PowerMonitorClient pwr(0);
uint32_t baud_counter = 9400;
#define GOAL_RATE 38400

void setup() {
ser1.begin(115200);
ser1.set(sic.baud_rate_,(uint32_t)GOAL_RATE);
ser3.begin(115200);
ser3.set(sic.baud_rate_,(uint32_t)GOAL_RATE);
Serial.begin(115200);

}
void loop() {
  Serial.print("attempted rate: ");
  Serial.println(baud_counter);
  ser1.begin(baud_counter);
    delay (5);
    uint32_t val = 0;
  if(ser1.get(sic.baud_rate_,val)){
    Serial.print(val);
    Serial.print(" was read (1) succesfully at :");
    Serial.println(baud_counter);
    delay(1);
    Serial.println("attempting change serial1...");
    ser1.set(sic.baud_rate_,(uint32_t)GOAL_RATE);
    delay(1000);
  }
  else{
//    Serial.print(baud_counter);
//    Serial.println("failed get");
  }
  float voltage = 0;
if(ser1.get(pwr.volts_,voltage)){
Serial.print("voltage reading1: ");
Serial.println(voltage);
delay(10);
}

ser3.begin(baud_counter);
    delay (5);

  if(ser3.get(sic.baud_rate_,val)){
    Serial.print(val);
    Serial.print(" was read (3) succesfully at :");
    Serial.println(baud_counter);
    delay(1);
    Serial.println("attempting change serial3...");
    ser3.set(sic.baud_rate_,(uint32_t)GOAL_RATE);
    delay(1000);
  }
  else{
    //Serial.print(baud_counter);
    //Serial.println("failed get");
  }

  if(ser3.get(pwr.volts_,voltage)){
  Serial.print("voltage reading1: ");
  Serial.println(voltage);
  delay(10);
  }
  baud_counter = baud_counter+200;
  if (baud_counter >120000){
    baud_counter = 8000;
  }
  
}
