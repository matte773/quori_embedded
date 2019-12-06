#include "pid_linear.hpp"
#include "float.h"
#include "math_helper.h"

// Review header file for usage notes.

PidLinear::PidLinear():
Kp(0),                // proportional gain
Ki(0),                // integral gain
Kd(0),                // derivative gain
saturation(FLT_MAX),  // saturation value for control action, units of control u
deadband(0),          // deadband value for control action, units of control u
feed_forward(0),      // feed forward, units of control u
r(0),                 // set point, units of output y
r_dot(0),             // set point derivative, units of output y
prev_y(0),            // previous value of set point
integral_action(0)    // integral action, units of control u
{
}

// Work Functions

float PidLinear::PidCompute(float y, float dt,float dt_inv)
{
  float y_dot = (y - prev_y)*dt_inv;
  return PidCompute(y, y_dot, dt,dt_inv);
}

float PidLinear::PidCompute(float y, float y_dot, float dt, float dt_inv)
{
  float e     = y     - r;      // error
  float e_dot = y_dot - r_dot;  // error derivative
  float u     = -Kp*e -Kd*e_dot + integral_action + feed_forward;
  
  // Add deadband
  if ( u > 0){ 
    u= u+deadband;
    }
  if (u < 0){ 
    u= u-deadband;
    }

  // Accumulate integral action unless control u is already saturated in the
  // direction correcting the error.  This is "Integrator Clamping" anti-windup.
  if( !((u > saturation && e < 0) || (u < -saturation && e > 0))) {
    integral_action = integral_action - Ki*e*dt;
  }
  u = SatFloat(u, -saturation, saturation);

  prev_y = y;
  return u;
}

void PidLinear::Reset()
{
  feed_forward = 0;
  r = 0;
  r_dot = 0;
  prev_y = 0;
  integral_action = 0;
}

// Setters

void PidLinear::set_reference(float in)     {r = in;}

void PidLinear::set_reference_dot(float in) {r_dot = in;}

void PidLinear::set_Kp(float in)            {Kp = in;}

void PidLinear::set_Ki(float in)            {Ki = in;}

void PidLinear::set_Kd(float in)            {Kd = in;}

void PidLinear::set_saturation(float in)    {saturation = in;}

void PidLinear::set_deadband(float in)    {deadband = in;}

void PidLinear::set_feed_forward(float in)  {feed_forward = in;}

// Getters

float PidLinear::get_reference()            {return r;}

float PidLinear::get_reference_dot()        {return r_dot;}

float PidLinear::get_Kp()                   {return Kp;}

float PidLinear::get_Ki()                   {return Ki;}

float PidLinear::get_Kd()                   {return Kd;}

float PidLinear::get_saturation()           {return saturation;}

float PidLinear::get_deadband()           {return deadband;}


float PidLinear::get_feed_forward() {return feed_forward;}
