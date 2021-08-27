#ifndef ANGLE_MATH_HPP
#define ANGLE_MATH_HPP

#include "fixed_point_math.hpp"

constexpr float kPi = 3.141592653589793238462643383279502884f;

// Wrap radians into interval [-pi, pi).
// Mimics Matlab function of the same name.
// 9 assembly instructions, 2 registers
static inline float WrapToPi(float x) {
  // Scale and shift.
  x = x * (1/(2*kPi)) + 0.5f;
  // Subtract by floor() without calling floor().  Yields [0,1).
  x = x - static_cast<int>(x);
  if(x < 0.0f)
    x = x + 1.0f;
  // Shift back and rescale. Yields [-pi,pi).
  x = x - 0.5f;
  return x * (2*kPi);
}

// Wrap radians into interval [0, 2*pi).
// Mimics Matlab function of the same name.
// 9 assembly instructions, 2 registers
static inline float WrapTo2Pi(float x) {
  // Scale.
  x = x * (1/(2*kPi));
  // Subtract by floor() without calling floor(). Yields [0,1).
  x = x - static_cast<int>(x);
  if(x < 0.0f)
    x = x + 1.0f;
  // Rescale. Yields [0,2*pi).
  return x * (2*kPi);
}

// Fixed point type for representing rotations. Rev32 objects add and subtract
// with natural wrapping.  The minimum and maximum values are understood to
// represent the interval [-pi, +pi).  The representation precision on this
// domain is greater than using floating point numbers.  Stored as Q32 format.
class Rev32 {
public:

  // Default Zero Constructor

  Rev32():value_(0){};

  // Named Constructors

  // Create from floating point type representing radians.
  static Rev32 FromRad(float val) {
    // Scale to revolutions, extract fractional part, multiply by 2^32.
    // Must ensure float to integer conversion does not overflow.
    constexpr float pre_coef = 1.0f/(2.0f*kPi);
    val = pre_coef * val;
    val = val - static_cast<int>(val);
    if(val < 0) {
      val = val + 1;
    }
    constexpr float post_coef = pow2<float>(32);
    val = val * post_coef;
    return Rev32::FromEncoder<32>(static_cast<uint32_t>(val));
  };

  // Create from index value out of a N-bit encoder.
  template <unsigned N>
  static Rev32 FromEncoder(uint32_t val) {
    constexpr unsigned shift = 32-N;
    return Rev32(val << shift);
  };

  // Mathematical Operators

  Rev32& operator+=(const Rev32& rhs) {
    value_ += rhs.value_;
    return *this;
  };

  Rev32& operator-=(const Rev32& rhs) {
    value_ -= rhs.value_;
    return *this;
  };

  Rev32 operator-() {
    return Rev32(-value_);
  };

  friend Rev32 operator+(Rev32 lhs, const Rev32& rhs) {
    lhs += rhs;
    return lhs;
  };

  friend Rev32 operator-(Rev32 lhs, const Rev32& rhs) {
    lhs -= rhs;
    return lhs;
  };

  // Output Conversions

  float ToRad2Pi() {
    constexpr float coef = kPi / (pow2<std::uint64_t>(31));
    return static_cast<float>(value_) * coef;
  };

  float ToRadPi() {
    constexpr float coef = kPi / (pow2<std::uint64_t>(31));
    int32_t val = static_cast<int32_t>(value_); // reinterpret value_ as signed integer
    return static_cast<float>(val) * coef;
  };

  // Approximate as index value out of a N-bit encoder.
  template<unsigned N>
  uint32_t ToEncoder() {
    constexpr unsigned shift = 32-N;
    uint32_t val = value_ >> shift;
    return val;
  };

private:

  Rev32(uint32_t val):value_(val) {};

  uint32_t value_;
};

// Fixed point type for representing multiple rotations. RevI15Q16 objects
// represent the interval (-32769, 32768) rotations or
// (205893.69933, 205887.41615) radians with a fixed precision of 1/65536
// rotation or 9.58738e-5 radians.
// There is no wrapping protection, so don't do it.
class RevI15Q16 {
public:
  // Default Zero Constructor

    RevI15Q16():value_(0){};

  // Named Constructors

  // Create from floating point type representing radians.
  static RevI15Q16 FromRad(float val) {
    // Scale to revolutions,  multiply by 2^16.
    constexpr float coef = pow2<float>(16)/(2.0f*kPi);
    val = coef * val; // rad to rotations
    return RevI15Q16(static_cast<int32_t>(val));
  };

  static RevI15Q16 FromRev2Pi(Rev32 val) {
    return RevI15Q16(static_cast<int32_t>(val.ToEncoder<16>()));
  };

  static RevI15Q16 FromRevPi(Rev32 val) {
    return RevI15Q16(static_cast<int32_t>(static_cast<int16_t>(val.ToEncoder<16>())));
  };

  // Mathematical Operators

  RevI15Q16& operator+=(const RevI15Q16& rhs) {
    value_ += rhs.value_;
    return *this;
  };

  RevI15Q16& operator-=(const RevI15Q16& rhs) {
    value_ -= rhs.value_;
    return *this;
  };

  RevI15Q16 operator-() {
    return RevI15Q16(-value_);
  };

  friend RevI15Q16 operator+(RevI15Q16 lhs, const RevI15Q16& rhs) {
    lhs += rhs;
    return lhs;
  };

  friend RevI15Q16 operator-(RevI15Q16 lhs, const RevI15Q16& rhs) {
    lhs -= rhs;
    return lhs;
  };

  // Output Conversions

  float ToRad() {
    constexpr float coef = kPi / (pow2<std::uint32_t>(15)); // 2*pi/2^16
    return static_cast<float>(value_) * coef;
  };

private:

  RevI15Q16(int32_t val):value_(val) {};

  int32_t value_;
};
#endif // ANGLE_MATH_HPP
