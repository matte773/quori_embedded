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

      const float p = set_positions->positions[0] * G_RATIO;
      state->positions[0] = clamp(p, MOTOR_LOW_LIMIT, MOTOR_UPP_LIMIT);

      SetPositionsRes set_positions_res;
      set_positions_res.values[0] = 0;
      set_positions_res.values[1] = 0;
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
      states.positions[0] = state->waist[0];

      states.measured[0] = state->measured[0];
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
      strncpy(initialized._0, "waist_hinge", sizeof(initialized._0));
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

  // 0.258475
  angle_sensor_waist.SetZeroPosition(map(0.18, -PI, PI, -8192, 8191));
  
  // 0.1988125
  angle_sensor_mt.SetZeroPosition(map(0.9, -PI, PI, -8192, 8191));
  
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

  

  /*for (;;)
  {
    angle_sensor_waist.SendGET3();
    const float angle = ticksToAngle(-angle_sensor_waist.ReadAngle());

    angle_sensor_mt.SendGET3();
    state.measured[0] = mt_filter_position.sample(ticksToAngle(-angle_sensor_mt.ReadAngle()));
    const unsigned long now = millis();

    state.measurement_time = now;

    Serial.printf("measured %d %d %d\n",
      (int)(angle * 100),
      (int)(MOTOR_UPP_LIMIT * 100),
      (int)(MOTOR_LOW_LIMIT * 100)
    );

    int serial_mt = 0;


    if (state.measured[0] >= MOTOR_UPP_LIMIT || state.measured[0] <= MOTOR_LOW_LIMIT)
    {
      if (state.measured[0] >= MOTOR_UPP_LIMIT)
      {
        float cmd = -0.1;
        set_volts(&cmd, &serial_mt);
      }
      else
      {
        float cmd = 0.1;
        set_volts(&cmd, &serial_mt);
      }

      delay(10);
      coast();
      continue;
    }

    if (angle >= JOINT_UPP_LIMIT || angle <= JOINT_LOW_LIMIT)
    {
      // Serial.println("Joint limit");
      coast();
      delay(10);
      continue;
    }



    const unsigned long delta_ms = now - last_pid_update;
    const float delta_secs = static_cast<float>(now - last_pid_update) / 1000.0f;
    last_pid_update = now;

    

    float cmd = pos_mt_pid.PidCompute(state.measured[0], delta_secs, 1.0 / delta_secs);


    set_volts(&cmd, &serial_mt);
    delay(10);
  }*/
}

uint8_t incoming_buffer[256];
size_t incoming_buffer_length = 0;

size_t i = 0;
void loop()
{
  const unsigned long now = millis();
  if (inited) ++i;

  angle_sensor_waist.SendGET3();
  state.waist[0] = ticksToAngle(-angle_sensor_waist.ReadAngle());

  angle_sensor_mt.SendGET3();
  state.measured[0] = mt_filter_position.sample(ticksToAngle(-angle_sensor_mt.ReadAngle()));
  state.measurement_time = millis();


  if (now - last_command_time > COMMAND_TIMEOUT)
  {
    coast();
    // if (i % 100 == 1) Log::create("Coasting... %u", 1).write(&Serial);
  }
  else
  {
    if (state.measured[0] >= MOTOR_UPP_LIMIT || state.measured[0] <= MOTOR_LOW_LIMIT || state.waist[0] >= JOINT_UPP_LIMIT || state.waist[0] <= JOINT_LOW_LIMIT)
    {
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
      // if (i % 100 == 1) Log::create("%d %d %d", (int)(cmd * 10), (int)(state.positions[0] * 10), (int)(clamped * 10)).write(&Serial);

      int serial_mt = 0;
      set_volts(&cmd, &serial_mt);

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

  delay(5);
}
