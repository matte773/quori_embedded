// Implements a PID controller C(s) to drive the process P(s) output y to the 
// reference r using control u.  The PID control is error feedback based on the 
// error signal e = y-r, but a feedforward value can optionally be added to u.
//
//             +------+       +------+         
//   r     e   |      |   u   |      |      y  
// +---> +---> | C(s) +-----> | P(s) +---+---->
//    + ^      |      |       |      |   |     
//     -|      +------+       +------+   |     
//      |                                |     
//      +--------------------------------+     
//
// A saturation value for u can be specified, in which case the output will be 
// restricted to -sat < u < sat and integrator clamping anti-windup will engage.

#ifndef PID_LINEAR_HPP
#define PID_LINEAR_HPP

#include <stdint.h>

class PidLinear {

protected:

  // parameters
  float Kp;           // proportional gain
  float Ki;           // integral gain
  float Kd;           // derivative gain
  float saturation;   // saturation value for control action, units of control u
  float deadband;     // deadband value for control action, units of control u
  float feed_forward; // feed forward, units of control u

  // variables
  float r;                // set point, units of output y
  float r_dot;            // set point derivative, units of output y / time
  float prev_y;           // previous value of set point
  float integral_action;  // integral action, units of control u

public:

  PidLinear();

  // Takes in output measurement y; returns control u.
  // The output derivative is implicity computed as y_dot = y - y_prev.
  // Before use, Kp, Ki, Kd, feed_forward, r, and r_dot should be set.
  float PidCompute(float y, float dt, float dt_inv);

  // Takes in output measurement y and its derivative y_dot; returns control y.
  // Before use, Kp, Ki, Kd, feed_forward, r, and r_dot should be set.
  float PidCompute(float y, float y_dot, float dt, float dt_inv);
  
  // Reset the feed forward, set point, previous measurement, and 
  // integrator all to zero.
  void Reset();

  // Setters

  void set_Kp(float);
  void set_Ki(float);
  void set_Kd(float);
  void set_saturation(float);
  void set_deadband(float);
  void set_feed_forward(float);
  void set_reference(float);
  void set_reference_dot(float);
  
  // Getters

  float get_Kp();
  float get_Ki();
  float get_Kd();
  float get_saturation();
  float get_deadband();
  float get_feed_forward();
  float get_reference();
  float get_reference_dot();
};

#endif // PID_LINEAR_HPP
