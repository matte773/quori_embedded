// Reminders about C++ standard integer operations on uint32_t and int32_t.
//
// uint32_t u;
// int32_t s;
//
// Signed to Unsigned Conversion: "Wraps (Safe)"
//   static_cast<uint32_t> s is source value + value of 2^32
// Unsigned to Signed Conversion "Wraps (Unsafe)"
//   static_vast<int32_t> u is implementation defined if u is too large
//   however, gcc specifies that in this case the value is reduced by 2^32.
// Unsigned + Unsigned Overflow: "OK"
//   performed modulo 2^32
// Signed + Signed overflow: "BAD"
//   UNDEFINED


#ifndef FIXED_POINT_MATH_HPP
#define FIXED_POINT_MATH_HPP

#include <cstdint>

// Compile time powers.  Template argument is storage/return type.
template <typename T>
constexpr T ipow(T base, unsigned exp, T result = 1) {
  return exp < 1 ? result : ipow(base*base, exp/2, (exp % 2) ? result*base : result);
}

// Compile time powers of 2.  Template argument is storage/return type.
template <typename T>
constexpr T pow2(unsigned exp) {
  return ipow(T(2), exp);
}

// // Fixed point types add and subtract with natural circular wrapping.
// template <unsigned N>
// class QN {
// public:

//   // Default Zero Constructor

//   QN<N>():value_(0){};

//   // Named Constructors

//   static QN FromRepr(uint32_t val) {
//     return QN<N>(val);
//   };

//   static QN FromNumerator(int32_t val) {
//     return QN<N>(static_cast<uint32_t>(val));
//   };

//   template <unsigned M>
//   static QN FromQN(QN<M> q) {
//     // bools computed at compile time
//     constexpr bool shift_left  = N > M;
//     constexpr bool shift_right = M > N;
//     // executed branch is determined at compile time by gcc optimizer
//     if(shift_left) {
//       constexpr unsigned n = pow2<unsigned>(N - M);
//       return FromRepr(q.ToRepr() * n);
//     }
//     if(shift_right) {
//       constexpr unsigned n = pow2<unsigned>(M - N);
//       int32_t temp = q.ToRepr();
//       return FromRepr(temp / n); // gcc optimizer does arithmetic shift
//     }
//     return FromRepr(q.ToRepr());
//   };

//   static QN FromFloat(float val) {
//     constexpr uint32_t coef = pow2<unsigned>(N);
//     return FromRepr(static_cast<uint32_t>(val * coef));
//   };

//   // Mathematical Operators

//   QN& operator+=(const QN& rhs) {
//     value_ += rhs.value_;
//     return *this;
//   };

//   QN& operator-=(const QN& rhs) {
//     value_ -= rhs.value_;
//     return *this;
//   };

//   QN operator-() {
//     return QN<N>(-value_);
//   };

//   friend QN operator+(QN lhs, const QN& rhs) {
//     lhs += rhs;
//     return lhs;
//   };

//   friend QN operator-(QN lhs, const QN& rhs) {
//     lhs -= rhs;
//     return lhs;
//   };

//   // Output Conversions

//   float ToFloat() {
//     // casting uint to int is implementation defined
//     constexpr float coef = 1.0f/pow2<unsigned>(N);
//     return static_cast<int32_t>(value_) * coef;
//   };

//   uint32_t ToRepr() {
//     return value_;
//   };

//   int32_t ToNumerator() {
//     return static_cast<int32_t>(value_);
//   };

// private:

//   QN<N>(uint32_t repr):value_(repr) {};

//   std::uint32_t value_;
// };

#endif // FIXED_POINT_MATH_HPP
