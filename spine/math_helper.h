#ifndef MATH_HELPER_H
#define MATH_HELPER_H

// Misc. math helper functions targeted for ARM Cortex-M with floating point.
// Use more specific #includes such as "angle_math.hpp" where possible.

#include <stdint.h>
#include <string.h> // for memcpy

#ifndef PI
  #define PI 3.14159265358979f
#endif

#ifdef __cplusplus
  // In C++, use of kPi in a constexpr guarantees computation at compile time.
  #include "angle_math.hpp"
  constexpr float k2Pi    = 2*kPi; // deprecated
  constexpr float kPiBy2  = kPi/2; // deprecated
  constexpr float kPiBy3  = kPi/3; // deprecated
#else
  // In C, no guarentee of when eg. kPi/2 will be computed, but in practice the
  // compiler will usually optimize this out.
  static const float kPi     = 3.141592653589793238462643383279502884f;
  static const float k2Pi    = 6.283185307179586476925286766559005768f;
  static const float kPiBy2  = 1.570796326794896619231321691639751442f;
  static const float kPiBy3  = 1.047197551196597746154214461093167628f;
#endif

////////////////////////////////////////////////////////////////////////////////
/// Input Checking

/// Wrap a float value in radians to between -PI and PI inclusive.
/// 9 assembly instructions, 3 registers
static inline float WrapRad(float rad) {
  while(rad > kPi) {
    rad = rad - k2Pi;
  };
  while(rad < -kPi) {
    rad = rad + k2Pi;
  };
  return rad;
}

/// Wrap a float value, in, between max and min inclusive.  Max must be greater than min.
static inline float WrapFloat(float in, float min, float max) {
  float step = max - min;
  while(in > max){
    in = in-step;
  };
  while(in < min){
    in = in + step;
  };
  return in;
}

/// Saturate a float value 'in' to between 'lb' and 'ub', inclusive.
static inline float SatFloat(float in, float lb, float ub) {
  if(in < lb)
    return lb;
  if(in > ub)
    return ub;
  return in;
}

/// Undo a deadband with a range of +- db.  Only shifts, does not scale.
static inline float LivebandFloat(float in, float db) {
  if(in < 0.0f)
    return in - db;
  if(in > 0.0f)
    return in+db;
  return 0.0f;
}

////////////////////////////////////////////////////////////////////////////////
/// Type Punning
/// Only available in C++ (no )

#ifdef __cplusplus

// example:
//   float my_float = 3.14f;
//   uint32_t x = pun<uint32_t>(my_float);
//   int32_t my_int = 42;
//   float y = pun<float>(my_int);
template <typename T, typename U>
static inline T Pun(const U &in)
{
  static_assert(sizeof(T) == sizeof(U), "Function Pun can not work on types with different sizes.");
  T out;
  memcpy(&out, &in, sizeof(T));
  return out;
}

/// Dereference pointer to uint32_t as a floating point value
static inline float PunToFloat(const uint32_t* in) {
  float out;
  memcpy(&out, in, 4);
  return out;
}

/// Dereference pointer to 4 uint8_t as a floating point value
static inline float PunToFloat(const uint8_t* in) {
  float out;
  memcpy(&out, in, 4);
  return out;
}

/// Dereference pointer to float as a uint32_t value
static inline uint32_t PunToUint32(const float* in) {
  uint32_t out;
  memcpy(&out, in, 4);
  return out;
}

/// Pun uint32_t as a float value
static inline float PunToFloat(const uint32_t in) {
  float out;
  memcpy(&out, &in, 4);
  return out;
}

/// Pun float as a uint32_t value
static inline uint32_t PunToUint32(const float in) {
  uint32_t out;
  memcpy(&out, &in, 4);
  return out;
}

/// Pun int32 as a uint32_t value
static inline uint32_t PunToUint32(int32_t in) {
  uint32_t out;
  memcpy(&out, &in, 4);
  return out;
}

/// Pun int16 as a uint32_t value
static inline uint32_t PunToUint32(int16_t in) {
  int32_t val = in;
  uint32_t out;
  memcpy(&out, &val, 4);
  return out;
}

/// Dereference pointer to 4 uint8_t as a uint32_t value
static inline uint32_t PunToUint32(const uint8_t* in) {
  uint32_t out;
  memcpy(&out, in, 4);
  return out;
}

/// Dereference pointer to 4 uint8_t as a int32_t value
static inline int32_t PunToInt32(const uint8_t* in) {
  int32_t out;
  memcpy(&out, in, 4);
  return out;
}

/// Dereference pointer to 2 uint8_t as a uint16_t value
static inline uint16_t PunToUint16(const uint8_t* in) {
  uint16_t out;
  memcpy(&out, in, 2);
  return out;
}

/// Dereference pointer to 2 uint8_t as a int16_t value
static inline int16_t PunToInt16(const uint8_t* in) {
  int16_t out;
  memcpy(&out, in, 2);
  return out;
}

#endif // __cplusplus

#endif
