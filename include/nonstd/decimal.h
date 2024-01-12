/*
 * copyright svante karlsson at csi dot se 2021
 */
#include "stdint128.h"
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>
#pragma once

// maximum precision of int16 -> 4
// maximum precision of int32 -> 10
// maximum precision of int64 -> 18
// maximum precision of int128 -> 38

// MAXINT16 32,767
// MAXDEC40 9999
// MAXDEC50 32767
// MAXDEC52 327.67
// MAXDEC94 99999.9999

// MAXDEC18 999999999999999999
// MAXDEC188 9999999999.99999999

namespace nonstd {
template <typename T>
inline constexpr bool Is_ValidINT =
    std::is_same_v<T, int16_t> || std::is_same_v<T, int32_t> ||
    std::is_same_v<T, int64_t> || std::is_same_v<T, int128_t>;

template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE,
          typename = std::enable_if_t<Is_ValidINT<INTEGER_TYPE>>>
class decimal {
private:
  static constexpr inline INTEGER_TYPE make_nan() {
    /* Selecting value outside numeric range for decimal */
    INTEGER_TYPE nan_value = std::numeric_limits<INTEGER_TYPE>::min();
    return nan_value;
  }

  static constexpr inline INTEGER_TYPE make_epsilon() {
    INTEGER_TYPE epsilon_value = 1;
    return epsilon_value;
  }

  static constexpr inline INTEGER_TYPE make_min_int() {
    INTEGER_TYPE min_representable =
        -pow10_128(PRECISION) <
                (int64_t)std::numeric_limits<INTEGER_TYPE>::min()
            ? std::numeric_limits<INTEGER_TYPE>::min() + 1
            : -pow10_128(PRECISION);
    return min_representable;
  }

  constexpr static inline const INTEGER_TYPE make_max_int() {
    return pow10_128(PRECISION) > std::numeric_limits<INTEGER_TYPE>::max()
               ? std::numeric_limits<INTEGER_TYPE>::max()
               : pow10_128(PRECISION);
  }

private:
  /** create from rep */
  constexpr inline explicit decimal(INTEGER_TYPE raw_value)
      : value_(raw_value) {}

public:
  constexpr explicit inline decimal() : value_(0) {}

  //constexpr explicit inline decimal(double v)
  //    : value_(0.5 + v * pow10_128(SCALE)) {}

  constexpr inline decimal(double v)
      : value_(0.5 + v * pow10_128(SCALE)) {}


  ~decimal() = default;

  // allow explicit cast from everything
  template <typename INTEGER_TYPE2, unsigned int PRECISION2,
            unsigned int SCALE2>
  constexpr explicit decimal(decimal<INTEGER_TYPE2, PRECISION2, SCALE2> b)
      : value_(b.get_rep() * (pow10_128(SCALE) / pow10_128(SCALE2))) {}

  [[nodiscard]] constexpr static inline decimal make_from_rep(INTEGER_TYPE t) {
    return decimal(t);
  }

  constexpr static const unsigned int precision = PRECISION;
  constexpr static const unsigned int scale = SCALE;
  constexpr static const INTEGER_TYPE MAXINT = make_max_int();
  constexpr static const INTEGER_TYPE NaN_VALUE = make_nan();
  constexpr static const INTEGER_TYPE MININT = make_min_int();
  constexpr static const INTEGER_TYPE EPSILON_VALUE = make_epsilon();

  constexpr inline explicit operator double() const {
    return value_ == NaN_VALUE ? std::numeric_limits<double>::quiet_NaN()
                               : ((double)value_ / (double)pow10_128(SCALE));
  }

  constexpr decimal operator-() const { return make_from_rep(-value_); }

  inline decimal &operator+=(const decimal &b) {
    value_ += b.value_;
    return *this;
  }

  // please do not use..
  inline INTEGER_TYPE get_rep() const { return value_; }

private:
  INTEGER_TYPE value_;
};

template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
inline decimal<INTEGER_TYPE, PRECISION, SCALE>
operator-(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1,
          const decimal<INTEGER_TYPE, PRECISION, SCALE> &c2) {
  return decimal<INTEGER_TYPE, PRECISION, SCALE>::make_from_rep(c1.get_rep() -
                                                                c2.get_rep());
}

template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
inline decimal<INTEGER_TYPE, PRECISION, SCALE>
operator+(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1,
          const decimal<INTEGER_TYPE, PRECISION, SCALE> &c2) {
  return decimal<INTEGER_TYPE, PRECISION, SCALE>::make_from_rep(c1.get_rep() +
                                                                c2.get_rep());
}

template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
inline decimal<INTEGER_TYPE, PRECISION, SCALE>
operator*(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1,
          const decimal<INTEGER_TYPE, PRECISION, SCALE> &c2) {
  return decimal<INTEGER_TYPE, PRECISION, SCALE>::make_from_rep(
      ((int128_t)c1.get_rep()) * ((int128_t)c2.get_rep()) / (pow10_128(SCALE)));
}

template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
inline decimal<INTEGER_TYPE, PRECISION, SCALE>
operator/(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1,
          const decimal<INTEGER_TYPE, PRECISION, SCALE> &c2) {
  return decimal<INTEGER_TYPE, PRECISION, SCALE>::make_from_rep(
      ((pow10_128(SCALE)) * (int128_t)c1.get_rep()) / ((int128_t)c2.get_rep()));
}

template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
inline bool operator==(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1,
                       const decimal<INTEGER_TYPE, PRECISION, SCALE> &c2) {
  return (c1.get_rep() == c2.get_rep());
}

template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
inline bool operator!=(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1,
                       const decimal<INTEGER_TYPE, PRECISION, SCALE> &c2) {
  return (c1.get_rep() != c2.get_rep());
}

template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
inline bool operator>(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1,
                      const decimal<INTEGER_TYPE, PRECISION, SCALE> &c2) {
  return (c1.get_rep() > c2.get_rep());
}

template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
inline bool operator<(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1,
                      const decimal<INTEGER_TYPE, PRECISION, SCALE> &c2) {
  return (c1.get_rep() < c2.get_rep());
}

template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
constexpr bool operator>=(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1,
                          const decimal<INTEGER_TYPE, PRECISION, SCALE> &c2) {
  return (c1.get_rep() >= c2.get_rep());
}

template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
constexpr bool operator<=(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1,
                          const decimal<INTEGER_TYPE, PRECISION, SCALE> &c2) {
  return (c1.get_rep() <= c2.get_rep());
}

// const operators against double
template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
inline bool operator==(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1,
                       double c2) {
  return (c1.get_rep() ==
          decimal<INTEGER_TYPE, PRECISION, SCALE>(c2).get_rep());
}

template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
inline bool operator!=(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1,
                       double c2) {
  return (c1.get_rep() !=
          decimal<INTEGER_TYPE, PRECISION, SCALE>(c2).get_rep());
}

template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
inline bool operator<(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1,
                      double c2) {
  return (c1.get_rep() < decimal<INTEGER_TYPE, PRECISION, SCALE>(c2).get_rep());
}

template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
inline bool operator<=(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1,
                       double c2) {
  return (c1.get_rep() <=
          decimal<INTEGER_TYPE, PRECISION, SCALE>(c2).get_rep());
}

template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
inline bool operator>(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1,
                      double c2) {
  return (c1.get_rep() > decimal<INTEGER_TYPE, PRECISION, SCALE>(c2).get_rep());
}

template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
inline bool operator>=(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1,
                       double c2) {
  return (c1.get_rep() >=
          decimal<INTEGER_TYPE, PRECISION, SCALE>(c2).get_rep());
}

// math operators on double
template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
constexpr decimal<INTEGER_TYPE, PRECISION, SCALE>
operator*(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1, double b) {
  return decimal<INTEGER_TYPE, PRECISION, SCALE>::make_from_rep(c1.get_rep() *
                                                                b);
}

template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
constexpr decimal<INTEGER_TYPE, PRECISION, SCALE>
operator/(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1, double b) {
  return decimal<INTEGER_TYPE, PRECISION, SCALE>::make_from_rep(
      (c1.get_rep() / b));
}

// math functions
template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
inline const decimal<INTEGER_TYPE, PRECISION, SCALE>
abs(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1) {
  return c1.get_rep() > 0 ? c1 : -c1;
}

template <typename INTEGER_TYPE, unsigned int PRECISION, unsigned int SCALE>
inline bool isnan(const decimal<INTEGER_TYPE, PRECISION, SCALE> &c1) {
  return c1.get_rep() == decimal<INTEGER_TYPE, PRECISION, SCALE>::NaN_VALUE;
}
} // namespace nonstd