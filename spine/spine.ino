// Per-robot configuration:

const static float QUORI_CONFIG_ZERO_POSITION_WAIST = 1.4965f;
const static float QUORI_CONFIG_ZERO_POSITION_MOTOR = 2.7852f;


#define USE_USBCON

#include <Arduino.h>
#include <wiring_private.h>
#include "common/MLX90363/MLX90363.h"
#include "common/MLX90363/MLX90363.cpp"
#include "common/quori/Filter.hpp"
#include "common/quori/message.hpp"
#include "common/quori/message.cpp"

#include "pid.h"
#include "motor.h"
#include "constants.h"
#include "additional_serial.h"
#include "LpfButter1.h"
#include "pid_linear.hpp"

const unsigned long DELAY_MS = 10;// 10 is 100Hz goal, 20 is 50Hz goal

template<typename T>
inline T clamp(T value, T min, T max)
{
  if (value != value) return min;
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

MLX90363 angle_sensor_waist(9);
MLX90363 angle_sensor_mt(10);

PidLinear pos_mt_pid;

LpfButter1 mt_filter_position(10, 200.0);

unsigned long last_command_time = 0;
unsigned long last_pid_update = 0;

const unsigned long COMMAND_TIMEOUT = 1000;

float ticksToAngle(const int ticks)
{
  return static_cast<float>(ticks) * 3.1415 / 8191.0;
}

struct GlobalState
{
  float positions[1];
  float measured[1];
  float waist[1];
  unsigned long measurement_time;
};

void coast()
{
  float cmd = 0;
  pos_mt_pid.Reset();
  last_pid_update = millis();
  int serial_mt = 0;
  set_volts(&cmd, &serial_mt);
}

Filter<float> mt_position_filter(0.175f);

bool inited = false;
bool got_states = false;
size_t iter = 0;


size_t processMessage(const std::uint8_t *const message, const size_t max_length, struct GlobalState *const state)
{
  if (max_length < 1) return 0;

  using namespace quori::message;

  switch (static_cast<Type>(message[0]))
  {
    case Type::SetPositions:
    {
      if (max_length < sizeof (SetPositions)) return 0;
      const SetPositions *const set_positions = reinterpret_cast<const SetPositions *>(message);

      const float p = -set_positions->positions[0] * G_RATIO;
      state->positions[0] = clamp(p, MOTOR_LOW_LIMIT, MOTOR_UPP_LIMIT);

      SetPositionsRes set_positions_res;
      set_positions_res.values[0] = state->measured[0];
      set_positions_res.values[1] = state->waist[0];
      set_positions_res.values[2] = 0;
      set_positions_res.values[3] = 0;

      Serial.write(reinterpret_cast<const uint8_t *>(&set_positions_res), sizeof(set_positions_res));
      return sizeof(SetPositions);
    }  
    case Type::Coast:
    {
      if (max_length < sizeof (Coast)) return 0;
      coast();
      return sizeof(Coast);
    }
    case Type::GetStates:
    {
      if (max_length < sizeof (GetStates)) return 0;

      States states;
      states.positions[0] = -state->waist[0];

      states.measured[0] = -state->waist[0];//state->measured[0];
      states.positions[1] = max_length;

      Serial.write(reinterpret_cast<const uint8_t *>(&states), sizeof(states));
      got_states = true;
      return sizeof(GetStates);
    }
    case Type::Initialize:
    {
      if (max_length < sizeof (Initialize)) return 0;

      const Initialize *initialize = reinterpret_cast<const Initialize *>(message);

      Initialized initialized;
      strncpy(initialized._0, "waist_pitch", sizeof(initialized._0));
      Serial.write(reinterpret_cast<const uint8_t *>(&initialized), sizeof(Initialized));
      inited = true;

      return sizeof(Initialize);
    }
    default:
    {
      SetPositionsRes set_positions_res;
      set_positions_res.values[0] = 1;
      set_positions_res.values[1] = 0;
      set_positions_res.values[2] = 0;
      set_positions_res.values[3] = 0;

      Serial.write(reinterpret_cast<const uint8_t *>(&set_positions_res), sizeof(set_positions_res));
      return sizeof (SetPositions);
    }
  }

  return 0;
}

struct GlobalState state;

void setup()
{
  // Start SPI (MOSI=11, MISO=12, SCK=13)
  MLX90363::InitializeSPI(11,12,13);

  Serial.begin(115200);
  Serial1.begin(115200);

  angle_sensor_waist.SetZeroPosition(map(QUORI_CONFIG_ZERO_POSITION_WAIST, -PI, PI, -8192, 8191));
  angle_sensor_mt.SetZeroPosition(map(QUORI_CONFIG_ZERO_POSITION_MOTOR, -PI, PI, -8192, 8191));
  
  state.measured[0] = 0;
  state.positions[0] = 0;
  state.waist[0] = 0;
  state.measurement_time = millis();

  delay(500);

  coast();
  last_command_time = 0;

  pos_mt_pid.set_Kp(1.0);
  pos_mt_pid.set_Ki(0.05);
  pos_mt_pid.set_Kd(0.03);
  pos_mt_pid.set_saturation(1.0);
  pos_mt_pid.set_deadband(0.0);
  pos_mt_pid.set_feed_forward(0.0);

  Serial.flush();

  const float clamped = clamp(0.0, JOINT_LOW_LIMIT, JOINT_UPP_LIMIT);
  pos_mt_pid.set_reference(clamped);
  pos_mt_pid.set_reference_dot(0.01);

  // Prime the filter
  for (unsigned i = 0; i < 100; ++i)
  {
    angle_sensor_mt.SendGET3();
    state.measured[0] = mt_filter_position.sample(ticksToAngle(-angle_sensor_mt.ReadAngle()));
    delay(2);
  }
  state.measurement_time = millis();
}

uint8_t incoming_buffer[256];
size_t incoming_buffer_length = 0;

size_t i = 0;
void loop()
{
  const unsigned long now = millis();
  const unsigned long LpStrt = now;
  if (inited) ++i;

  angle_sensor_waist.SendGET3();
  state.waist[0] = ticksToAngle(-angle_sensor_waist.ReadAngle());

  angle_sensor_mt.SendGET3();
  state.measured[0] = mt_filter_position.sample(ticksToAngle(-angle_sensor_mt.ReadAngle()));
  state.measurement_time = millis();
  
  bool mtrErr = isMotorErr();
  unsigned int vinmV = getVIN();

  if (now - last_command_time > COMMAND_TIMEOUT)
  {
    iter = 0;
    coast();
  }
  else
  {
//    if (state.measured[0] >= MOTOR_UPP_LIMIT || state.measured[0] <= MOTOR_LOW_LIMIT || state.waist[0] >= JOINT_UPP_LIMIT || state.waist[0] <= JOINT_LOW_LIMIT)
    if ((state.waist[0] >= JOINT_UPP_LIMIT && (state.positions[0]/G_RATIO)>= JOINT_UPP_LIMIT) || (state.waist[0] <= JOINT_LOW_LIMIT && (state.positions[0]/G_RATIO)<= JOINT_LOW_LIMIT))
    {
      iter = 0;
      coast();
    }
    else if(mtrErr){
      iter = 0;
      coast();
    }
    else
    {


      const float clamped = clamp(state.positions[0], MOTOR_LOW_LIMIT, MOTOR_UPP_LIMIT);
      pos_mt_pid.set_reference(clamped);
      pos_mt_pid.set_reference_dot(1.0);

      const unsigned long now = millis();
      const unsigned long delta_ms = now - last_pid_update;
      const float delta_secs = static_cast<float>(delta_ms) / 1000.0f;
      last_pid_update = now;
      float cmd = pos_mt_pid.PidCompute(state.measured[0], delta_secs, 1.0 / delta_secs);

      int serial_mt = 0;

      if (iter < 400)
      {
        const float factor = static_cast<float>(iter) / 400.0f;
        cmd *= factor;
        set_volts(&cmd, &serial_mt);
        iter++;
      }
      else
      {
        set_volts(&cmd, &serial_mt);  
      }

    }
  }

  while (Serial.available() && incoming_buffer_length < sizeof(incoming_buffer))
  {
    incoming_buffer[incoming_buffer_length++] = Serial.read();
  }

  while (incoming_buffer_length > 0)
  {
    const size_t read_count = processMessage(incoming_buffer, incoming_buffer_length, &state);
    if (read_count == 0) break;
    
    last_command_time = now;
    incoming_buffer_length -= read_count;
    if (incoming_buffer_length > 0) memmove(incoming_buffer, incoming_buffer + read_count, incoming_buffer_length);
  }

  const unsigned long end = millis();
  const unsigned long duration = end - LpStrt;
  
  if (duration < DELAY_MS){
    delay(DELAY_MS - duration);
  }
}
