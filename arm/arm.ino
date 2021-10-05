 #include "calibration.hpp"

#define USE_USBCON
#include <Arduino.h>
#include <wiring_private.h>
#include "common/MLX90363/MLX90363.h"
#include <SPI.h>

#include "common/quori/Filter.hpp"
#include "common/quori/message.hpp"

#include "common/libiqinetics/inc/bipbuffer.h"
#include "common/libiqinetics/inc/communication_interface.h"
#include "common/libiqinetics/inc/byte_queue.h"
#include "common/libiqinetics/inc/packet_finder.h"
#include "common/libiqinetics/inc/crc_helper.h"
#include "common/libiqinetics/inc/generic_interface.hpp"
#include "common/libiqinetics/inc/multi_turn_angle_control_client.hpp"

#include "common/MLX90363/MLX90363.cpp"

#include "common/libiqinetics/src/byte_queue.c"
#include "common/libiqinetics/src/crc_helper.c"
#include "common/libiqinetics/src/generic_interface.cpp"
#include "common/libiqinetics/src/packet_finder.c"
#include "common/quori/message.cpp"
#include "common/quori/Result.hpp"
#include "common/quori/MultiResult.hpp"

#include <cmath>


const static float TICKS2RADIANS = (2.0 * PI / 16383.0);
const static float TICKS_PER_REV = 16383.0;
const static size_t MOTOR_BAUD_RATE = 38400;
const static unsigned long READ_TIMEOUT = 5;
const unsigned long DELAY_MS = 10;// 10 is 100Hz goal, 20 is 50Hz goal

template<typename T>
inline T clamp(T value, T min, T max)
{
  if (value != value) return min;
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

bool inited = false;
size_t i = 0;

class Actuator
{
public:
  enum class GetMeasuredState : uint8_t
  {
    Ok,
    ChecksumError,
    LimitViolation,
  };

  typedef quori::MultiResult<GetMeasuredState, float> GetMeasuredResult;

  enum class State : uint8_t
  {
    Ok,
    ChksumError,
    LimitViolation,
    NotFresh
  } state_;

  Actuator(HardwareSerial *const serial, const uint8_t angle_sensor_pin, const float limit_min, const float limit_max)
    : state_(State::Ok)
    , serial_(serial)
    , angle_control_client_(0)
    , angle_sensor_pin_(angle_sensor_pin)
    , angle_sensor_(nullptr)
    , limit_min_(limit_min)
    , limit_max_(limit_max)
    , safe_(false)
    , prev_position_(0.0f)
    , prev_measured_(0.0f)
    , measured_filter_p2_(0.08f)
    , disconnected_(false)
  {
  }

  ~Actuator()
  {
    delete angle_sensor_;
  }

  void initialize(const float zero_position)
  {
    angle_sensor_ = new MLX90363(angle_sensor_pin_);
    serial_->begin(MOTOR_BAUD_RATE); 
    angle_sensor_->SetZeroPosition(map(zero_position, -PI, PI, -8192, 8191));
  }

  bool setPosition(float position)
  {
    if (safe_)
    {
      
    }

    angle_control_client_.ctrl_angle_.set(comm_, position);
    write_();
    return true;
  }

  quori::Result<float> getPosition()
  {
    angle_control_client_.obs_angular_displacement_.get(comm_);
    write_();
    
    uint8_t read_buffer[256];

    size_t start = millis();
    while (!angle_control_client_.obs_angular_displacement_.IsFresh() && (millis() - start) < READ_TIMEOUT)
    {
      const size_t read_length = read_(read_buffer, sizeof(read_buffer));
      appendPacketData_(read_buffer, read_length);
      delay(1);
    }

    if (!angle_control_client_.obs_angular_displacement_.IsFresh())
    {
      disconnected_ = true;
      state_ = State::NotFresh;
      return quori::Result<float>();
    }

    state_ = State::Ok;

    disconnected_ = false;
    return prev_position_ = angle_control_client_.obs_angular_displacement_.get_reply();
  }

  float getPrevPosition() const
  {
    return prev_position_;
  }

  GetMeasuredResult getMeasured()
  {
    // This function returns true on ERROR. Sigh.
    if (angle_sensor_->SendGET3())
    {
      Log log;
      // state_ = State::ChksumError;
      
//      snprintf(log.message, sizeof (log.message), "chksum error");
//      Serial.write(reinterpret_cast<const uint8_t *>(&log), sizeof(log));
//      snprintf(log.message, sizeof (log.message), angle_sensor_->WholeMessage());
//      Serial.write(reinterpret_cast<const uint64_t *>(&log), sizeof(log));
//      Log::create("My message %d", angle_sensor_->WholeMessage()).write(&Serial); 
      return GetMeasuredResult(GetMeasuredState::ChecksumError, 0.0f);
    }

    prev_ticks_ = angle_sensor_->ReadAngle();

    const float next = prev_ticks_ / TICKS_PER_REV * 2 * M_PI;

    if (next < limit_min_ || next > limit_max_)
    {
      // state_ = State::LimitViolation;
      return GetMeasuredResult(GetMeasuredState::LimitViolation, next);
    }

    prev_measured_ = measured_filter_p1_.update(next);
    //prev_measured_ = measured_filter_p2_.update(prev_measured_);

    

    return GetMeasuredResult(GetMeasuredState::Ok, prev_measured_);
  }

  float getPrevMeasured() const
  {
    return prev_measured_;
  }

  void coast()
  {
    angle_control_client_.ctrl_coast_.set(comm_);
    write_();
  }

  bool isDisconnected()
  {
    return disconnected_;
  }

  float getLimitMin()
  {
    return limit_min_;
  }

  float getLimitMax()
  {
    return limit_max_;
  }

  void setSafe(bool safe)
  {
    safe_ = safe;
  }

  int64_t prev_ticks_;
  


private:
  void write_()
  {
    uint8_t buffer[256];
    uint8_t length = sizeof(buffer);
    comm_.GetTxBytes(buffer, length);
    serial_->write(buffer, length);
  }

  size_t read_(uint8_t *const buffer, size_t length)
  {
    size_t ret = 0;
    while (serial_->available()) buffer[ret++] = serial_->read();
    return ret;
  }

  void appendPacketData_(uint8_t *const buffer, size_t length)
  {
    uint8_t *rx_data;
    uint8_t rx_length;

    comm_.SetRxBytes(buffer, length);
    while(comm_.PeekPacket(&rx_data, &rx_length))
    {
      angle_control_client_.ReadMsg(comm_, rx_data, rx_length);
      comm_.DropPacket();
    }
  }

  HardwareSerial *serial_;
  GenericInterface comm_;
  MultiTurnAngleControlClient angle_control_client_;
  uint8_t angle_sensor_pin_;
  MLX90363 *angle_sensor_;

  float limit_min_;
  float limit_max_;
  bool safe_;

  float prev_position_;
  float prev_measured_;

  quori::MedianFilter<float, 3> measured_filter_p1_;
  quori::Filter<float> measured_filter_p2_;

  bool disconnected_;

  
};

static Actuator actuators[2] = {
  // inner
  Actuator(&Serial1, 9, -2.4, 2.4),
  // outer
  Actuator(&Serial3, 10, -1.3, 1.3)
};

int asd = 0;

void coast()
{
  for (int i = 0; i < 2; ++i) actuators[i].coast();
}

unsigned long communication_time_last = 0;
unsigned long last_command_time = 0;
const unsigned long COMMAND_TIMEOUT = 1000;
const static float G_RATIO = 12.18f;



Filter<float> position_filters[2] = {
  Filter<float>(0.175f),
  Filter<float>(0.175f)
};

struct GlobalState
{
  float positions[2];

  float measured[2];
  unsigned long measurement_time;
};

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
      
#ifdef QUORI_CONFIG_ARM_LEFT
      state->positions[0] = clamp(-set_positions->positions[0], -2.4f, 2.4f);
#else
      state->positions[0] = clamp(set_positions->positions[0], -2.4f, 2.4f);
#endif
      state->positions[1] = clamp(set_positions->positions[1], -1.3f, 1.3f);

      SetPositionsRes set_positions_res;

      set_positions_res.values[0] = static_cast<uint8_t>(actuators[1].state_);
      set_positions_res.values[1] = actuators[0].prev_ticks_;
      set_positions_res.values[2] = actuators[0].getLimitMin();
      set_positions_res.values[3] = asd;
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

      
      const unsigned long start = millis();
      States states;

      for (int i = 0; i < 2; ++i)
      {
        float position = 0.0f;
        float measured = 0.0f;
#ifdef QUORI_CONFIG_ARM_LEFT
        if (i == 0)
        {
          position = -actuators[i].getPrevPosition();
          measured = -actuators[i].getPrevMeasured();
        }
        else
        {
          position = actuators[i].getPrevPosition();
          measured = actuators[i].getPrevMeasured();
        }
#else
        position = actuators[i].getPrevPosition();
        measured = actuators[i].getPrevMeasured();
#endif

        states.positions[i] = position;
        states.measured[i] = measured;
      }      

      const unsigned long end = millis();

      Serial.write(reinterpret_cast<const uint8_t *>(&states), sizeof(states));

      return sizeof(GetStates);
    }
    case Type::Initialize:
    {
      if (max_length < sizeof (Initialize)) return 0;

      const Initialize *initialize = reinterpret_cast<const Initialize *>(message);

      Initialized initialized;
#ifdef QUORI_CONFIG_ARM_LEFT
      strncpy(initialized._0, "l_shoulder_pitch", sizeof(initialized._0));
      strncpy(initialized._1, "l_shoulder_roll", sizeof(initialized._1));
#else
      strncpy(initialized._0, "r_shoulder_pitch", sizeof(initialized._0));
      strncpy(initialized._1, "r_shoulder_roll", sizeof(initialized._1));
#endif
      Serial.write(reinterpret_cast<const uint8_t *>(&initialized), sizeof(Initialized));
      
      inited = true;

      return sizeof(Initialize);
    }
  }

  return 0;
}

struct GlobalState state;

void setup()
{
  // Start SPI (MOSI=11, MISO=12, SCK=13)
  MLX90363::InitializeSPI(11, 12, 13);

  // Initialize the actuators

#ifdef QUORI_CONFIG_ARM_LEFT
  actuators[0].initialize(QUORI_LEFT_ARM_ZERO_POSITION_X);
  actuators[1].initialize(QUORI_LEFT_ARM_ZERO_POSITION_Y);
#else
  actuators[0].initialize(QUORI_RIGHT_ARM_ZERO_POSITION_X);
  actuators[1].initialize(QUORI_RIGHT_ARM_ZERO_POSITION_Y);
#endif

  state.positions[0] = 0.0f;
  state.positions[1] = 0.0f;
  state.measured[0] = 0.0f;
  state.measured[1] = 0.0f;
  state.measurement_time = millis();

  Serial.begin(115200);
  delay(500);

  coast();
  last_command_time = 0;
  
  Serial.flush();
}

uint8_t incoming_buffer[256];
size_t incoming_buffer_length = 0;

size_t iter = 0;
size_t disconnected_count = 0;


void loop()
{
  const unsigned long now = millis() + COMMAND_TIMEOUT;

  if (now - last_command_time > COMMAND_TIMEOUT)
  {
    iter = 0;
    coast();
  }
  else
  {
    // Angle sensor readings in radians

    using namespace quori;

    Actuator::GetMeasuredResult measured0 = actuators[0].getMeasured();
    Actuator::GetMeasuredResult measured1 = actuators[1].getMeasured();


    if (measured0.getState() == Actuator::GetMeasuredState::Ok) state.measured[0] = measured0.value();
    if (measured1.getState() == Actuator::GetMeasuredState::Ok) state.measured[1] = measured1.value();

    Result<float> position0 = actuators[0].getPosition();
    Result<float> position1 = actuators[1].getPosition();

    const bool success = (
      measured0.getState() == Actuator::GetMeasuredState::Ok &&
      measured1.getState() == Actuator::GetMeasuredState::Ok &&
      position0.isOk() &&
      position1.isOk()
    );

    if (actuators[0].isDisconnected() || actuators[1].isDisconnected())
    {
      ++disconnected_count;
    }
    else
    {
      disconnected_count = 0;
    }

    if (disconnected_count > 100)
    {
      iter = 0;
    }

    if (success)
    {
      state.measurement_time = now;

      // state.positions are user commands in radians
      const float diff_x = state.positions[0] - state.measured[0];
      const float diff_y = state.positions[1] - state.measured[1];

      const float raw_x = G_RATIO * (diff_x + diff_y) + position0.ok();
      
      // low pass filter
      const float filtered_x = position_filters[0].update(raw_x);

      const float raw_y = G_RATIO * (diff_x - diff_y) + position1.ok();

      // low pass filter
      const float filtered_y = position_filters[1].update(raw_y);
    

      if (iter > 200)
      {
        asd = 0;
        actuators[0].setPosition(filtered_x);
        actuators[1].setPosition(filtered_y);
      }
      else
      {
        asd = 1;
        const float factor = static_cast<float>(iter) / 200.0f;
        actuators[0].setPosition(filtered_x * factor + position0.ok() * (1.0f - factor));
        actuators[1].setPosition(filtered_y * factor + position1.ok() * (1.0f - factor));
        iter++;
      }
    }
    else
    {
      asd = 3;
      if (measured0.getState() == Actuator::GetMeasuredState::LimitViolation)
      {
        actuators[0].coast();
        asd = 4;
      }
      if (measured1.getState() == Actuator::GetMeasuredState::LimitViolation)
      {
        actuators[1].coast();
        asd = 5;
      }
      if (!position0.isOk())
      {
        asd = 6;
      }
      if (position1.isErr())
      {
        asd = 7;
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
    memmove(incoming_buffer, incoming_buffer + read_count, incoming_buffer_length);
  }

  const unsigned long end = millis() + COMMAND_TIMEOUT;

  const unsigned long duration = end - now;

  
  if (duration < DELAY_MS){
    delay(DELAY_MS - duration);
  }
}
