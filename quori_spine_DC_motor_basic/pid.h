#ifndef Pid_h
#define Pid_h

#include "Arduino.h"
#include "constants.h"

typedef struct PID_Vars {
  float Kp;
  float Ki;
  float Kd;
  float error_acc;
  float error_prev; 
  float prevErrorDif;
  PID_Vars(float Kp_in, float Ki_in, float Kd_in) {
    Kp = Kp_in;
    Ki = Ki_in;
    Kd = Kd_in;
    error_acc = 0.0;
    error_prev = 0.0;
  }

  float update_pid(float targetValue, float currentValue, float time_elapsed) {

      float error_acc_limit = 5.0;//TODO Check this
      float pidTerm = 0.0;
      float error = targetValue - currentValue; //Defines Proportional Term

      if (targetValue == 0) {
        
        error = 0;      
        error_acc = 0;  //guarantees no integral windup when system is at rest
        pidTerm = 0;    //guarantees no jitter when system is at rest      
      } else {   

         float errorDif = error - error_prev;  //defines Derivative Term

         // error_acc += error * (time_elapsed*0.001);   //defines Integral Term
           error_acc += error * (DT*0.001);   //defines Integral Term TODO get rid of float mult
          error_acc = constrain((error_acc), -1*error_acc_limit, error_acc_limit); // Anti integrator windup using clamping

          // Low Pass Filter
          // float beta = 0.5;
          // float filteredDifference = beta * errorDif + (1 - beta) * prevErrorDif;

          //pidTerm = (Kp * error) + (Kd * (filteredDifference) / (time_elapsed/1000.0)) + (Ki * (error_acc));      
//          Ki = 0;
         // Kd = 0;pw
          //pidTerm = (Kp * error) + (Kd * (errorDif) / (time_elapsed)*0.001) + (Ki * (error_acc));  //The PID Controller
          pidTerm = (Kp * error) + (Kd * (errorDif) *DT_INV*0.001) + (Ki * (error_acc));  //The PID Controller TODO Check this
      
      }
  
    /*
      Serial.print("P: ");
      Serial.print((Kp * error),3);
      Serial.print(", D: ");
      Serial.print((Kd * (error - error_prev) / (time_elapsed/1000.0)),3);
      Serial.print(", I: ");
      Serial.print((Ki * (error_acc)),3);
      Serial.print("\n");
    */
      error_prev = error; // stores old error for the next loop      
      return pidTerm;
    }
};


#endif




