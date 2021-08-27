#include "message.hpp"

using namespace quori;
using namespace quori::message;

Initialize::Initialize()
  : marker(static_cast<uint8_t>(Type::Initialize))
{
}

SetPositions::SetPositions()
  : marker(static_cast<uint8_t>(Type::SetPositions))
{
}

Set::Set()
  : marker(static_cast<uint8_t>(Type::Set))
{
}

SetPositionsRes::SetPositionsRes()
  : marker(static_cast<uint8_t>(Type::SetPositionsRes))
{
}

GetStates::GetStates()
  : marker(static_cast<uint8_t>(Type::GetStates))
{
}

SetVelocities::SetVelocities()
  : marker(static_cast<uint8_t>(Type::SetVelocities))
{
}

Coast::Coast()
  : marker(static_cast<uint8_t>(Type::Coast))
{
}

SetLimit::SetLimit()
  : marker(static_cast<uint8_t>(Type::SetLimit))
{
}

Log::Log()
  : marker(static_cast<uint8_t>(Type::Log))
{
  memset(message, 0, sizeof(message));
}

Log Log::create(const char *const message)
{
  Log log;
  log.marker = static_cast<uint8_t>(Type::Log);
  strncpy(log.message, message, sizeof (log.message));
  return log;
}

Log Log::create(const char *const format, ...)
{
  Log log;
  log.marker = static_cast<uint8_t>(Type::Log);
  va_list args;
  va_start(args, format);
  vsnprintf(log.message, sizeof (log.message), format, args);
  va_end(args);
  return log;
}

void Log::write(usb_serial_class *const serial)
{
  serial->write(reinterpret_cast<const uint8_t *>(this), sizeof (Log));
}

States::States()
  : marker(static_cast<uint8_t>(Type::States))
{
}

Initialized::Initialized()
  : marker(static_cast<uint8_t>(Type::Initialized))
{
  _0[0] = 0;
  _1[0] = 0;
  _2[0] = 0;
  modes[0] = 0;
  modes[1] = 0;
  modes[2] = 0;
}

size_t message::string_length(const std::string &t)
{
  return t.size();
}

size_t message::string_length(const char *const t)
{
  return strlen(t);
}

const char *message::c_string(const std::string &t)
{
  return t.c_str();
}

const char *message::c_string(const char *const t)
{
  return t;
}