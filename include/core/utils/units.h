#pragma once

#include <algorithm>
#include <cevalm.hpp>
#include <cmath>
#include <concepts>
#include <numbers>
#include <ratio>
#include <type_traits>
#include <utility>

#if __cplusplus < 202300L
#error "requires >= C++23"
#endif

namespace units {

/**
 * Quantity class represents a value that has a unit. Storage is in the SI base.
 */
template <typename Mass = std::ratio<0>, typename Length = std::ratio<0>,
          typename Time = std::ratio<0>, typename Current = std::ratio<0>,
          typename Angle = std::ratio<0>, typename Temperature = std::ratio<0>,
          typename Luminosity = std::ratio<0>, typename Moles = std::ratio<0>>
class Quantity {
protected:
  double value;

public:
  using mass = Mass;
  using length = Length;
  using time = Time;
  using current = Current;
  using angle = Angle;
  using temperature = Temperature;
  using luminosity = Luminosity;
  using moles = Moles;

  using Self = Quantity<Mass, Length, Time, Current, Angle, Temperature,
                        Luminosity, Moles>;
  static constexpr bool is_dimensionless = std::is_same_v<Self, Quantity<>>;

  /**
   * Default to value of 0.
   */
  explicit constexpr Quantity() : value(0) {}

  /**
   * Construct a new Quantity object using the SI base unit
   *
   * @param value to initialize with
   */
  explicit constexpr Quantity(double value)
    requires(!is_dimensionless)
      : value(value) {}

  /**
   * Construct a new Quantity object
   *
   * @param value to initialize with
   */
  constexpr Quantity(double value)
    requires is_dimensionless
      : value(value) {}

  /**
   * Construct a quantity from a numeric value expressed in a specific unit.
   *
   * @param value numeric value in the provided unit
   * @param unit unit in which value is expressed
   */
  constexpr Quantity(double value, Self unit) : value(value * unit.value) {}

  /**
   * Double cast overload, gets internal value.
   *
   * @return internal value
   */
  constexpr operator double() const
    requires is_dimensionless
  {
    return value;
  }

  /**
   * Constructor copying from a quantity with the same dimensions
   *
   * @param other the quantity to copy
   */
  constexpr Quantity(const Self &other) = default;

  /**
   * Gets the internal raw value
   *
   * @return constexpr internal value
   */
  constexpr double internal() const { return value; }

  /**
   * Gets this quantity's numeric value expressed in a specific unit.
   * e.g. double v_mps = v.to(mps);
   */
  constexpr double to(Self unit) const { return value / unit.value; }

  /**
   * Adds another quantity's value to this.
   */
  constexpr Self &operator+=(Self other) {
    value += other.value;
    return *this;
  }

  /**
   * Adds a double to this internal value
   * Only allowed if this quantity is dimensionless
   */
  constexpr Self &operator+=(double other)
    requires is_dimensionless
  {
    value += other;
    return *this;
  }

  /**
   * Subtracts another quantity's value from this.
   */
  constexpr Self &operator-=(Self other) {
    value -= other.value;
    return *this;
  }

  /**
   * Subtracts a double from this internal value
   * Only allowed if this quantity is dimensionless
   */
  constexpr Self &operator-=(double other)
    requires is_dimensionless
  {
    value -= other;
    return *this;
  }

  /**
   * Multiplies this quantity by a scalar
   */
  constexpr Self &operator*=(double scalar) {
    value *= scalar;
    return *this;
  }

  /**
   * Divides this quantity by a scalar
   */
  constexpr Self &operator/=(double divisor) {
    value /= divisor;
    return *this;
  }

  /**
   * Sets this quantity's value to another value
   * Only allowed if the quantity is dimensionless
   */
  constexpr Self &operator=(const double &rhs)
    requires is_dimensionless
  {
    value = rhs;
    return *this;
  }
};

/**
 * Number is just dimensionless
 */
using Number = Quantity<>;

/**
 * quantity checker. Used by the is_quantity concept
 */
template <typename Mass = std::ratio<0>, typename Length = std::ratio<0>,
          typename Time = std::ratio<0>, typename Current = std::ratio<0>,
          typename Angle = std::ratio<0>, typename Temperature = std::ratio<0>,
          typename Luminosity = std::ratio<0>, typename Moles = std::ratio<0>>
void quantity_checker(Quantity<Mass, Length, Time, Current, Angle, Temperature,
                               Luminosity, Moles>) {}

/**
 * Used to require a Quantity be passed in a template (instead of typename Q)
 */
template <typename Q>
concept IsQuantity = requires(Q q) { quantity_checker(q); };

/**
 * Creates a quantity from a numeric value expressed in a specific unit.
 * e.g. Velocity v = from(5, inps);
 */
template <IsQuantity Q> constexpr Q from(double value, Q unit) {
  return Q(value * unit.internal());
}

/**
 * Isomorphic concept checks whether dimensions are the same between quantities
 */
template <typename Q, typename... Quantities>
concept Isomorphic = ((std::convertible_to<Q, Quantities> &&
                       std::convertible_to<Quantities, Q>) &&
                      ...);

/**
 * Multiplying quantities adds their dimensions
 */
template <IsQuantity Q1, IsQuantity Q2>
using Multiplied =
    Quantity<std::ratio_add<typename Q1::mass, typename Q2::mass>,
             std::ratio_add<typename Q1::length, typename Q2::length>,
             std::ratio_add<typename Q1::time, typename Q2::time>,
             std::ratio_add<typename Q1::current, typename Q2::current>,
             std::ratio_add<typename Q1::angle, typename Q2::angle>,
             std::ratio_add<typename Q1::temperature, typename Q2::temperature>,
             std::ratio_add<typename Q1::luminosity, typename Q2::luminosity>,
             std::ratio_add<typename Q1::moles, typename Q2::moles>>;

/**
 * Dividing quantities subtracts their dimensions
 */
template <IsQuantity Q1, IsQuantity Q2>
using Divided = Quantity<
    std::ratio_subtract<typename Q1::mass, typename Q2::mass>,
    std::ratio_subtract<typename Q1::length, typename Q2::length>,
    std::ratio_subtract<typename Q1::time, typename Q2::time>,
    std::ratio_subtract<typename Q1::current, typename Q2::current>,
    std::ratio_subtract<typename Q1::angle, typename Q2::angle>,
    std::ratio_subtract<typename Q1::temperature, typename Q2::temperature>,
    std::ratio_subtract<typename Q1::luminosity, typename Q2::luminosity>,
    std::ratio_subtract<typename Q1::moles, typename Q2::moles>>;

/**
 * Exponentiating a quantity multiplies its dimensions by the power
 */
template <IsQuantity Q, typename factor>
using Exponentiated =
    Quantity<std::ratio_multiply<typename Q::mass, factor>,
             std::ratio_multiply<typename Q::length, factor>,
             std::ratio_multiply<typename Q::time, factor>,
             std::ratio_multiply<typename Q::current, factor>,
             std::ratio_multiply<typename Q::angle, factor>,
             std::ratio_multiply<typename Q::temperature, factor>,
             std::ratio_multiply<typename Q::luminosity, factor>,
             std::ratio_multiply<typename Q::moles, factor>>;

/**
 * Rooting a quantity divides its dimensions by the root
 */
template <IsQuantity Q, typename quotient>
using Rooted = Quantity<std::ratio_divide<typename Q::mass, quotient>,
                        std::ratio_divide<typename Q::length, quotient>,
                        std::ratio_divide<typename Q::time, quotient>,
                        std::ratio_divide<typename Q::current, quotient>,
                        std::ratio_divide<typename Q::angle, quotient>,
                        std::ratio_divide<typename Q::temperature, quotient>,
                        std::ratio_divide<typename Q::luminosity, quotient>,
                        std::ratio_divide<typename Q::moles, quotient>>;

template <IsQuantity Q> constexpr Q operator+(Q rhs) { return rhs; }

/**
 * Add two isomorphic quantities
 */
template <IsQuantity Q, IsQuantity R>
constexpr Q operator+(Q lhs, R rhs)
  requires Isomorphic<Q, R>
{
  return Q(lhs.internal() + rhs.internal());
}

/**
 * Negate a quantity
 */
template <IsQuantity Q> constexpr Q operator-(Q rhs) {
  return Q(-rhs.internal());
}

/**
 * Subtract two isomorphic quantities
 */
template <IsQuantity Q, IsQuantity R>
constexpr Q operator-(Q lhs, R rhs)
  requires Isomorphic<Q, R>
{
  return Q(lhs.internal() - rhs.internal());
}

/**
 * Multiply a Quantity by a Number
 */
template <IsQuantity Q>
  requires(!std::is_same_v<Q, Number>)
constexpr Q operator*(Q quantity, Number multiple) {
  return Q(quantity.internal() * multiple.internal());
}

/**
 * Multiply a Quantity by a Number
 */
template <IsQuantity Q>
  requires(!std::is_same_v<Q, Number>)
constexpr Q operator*(Number multiple, Q quantity) {
  return Q(quantity.internal() * multiple.internal());
}

/**
 * Divide a Quantity by a Number
 */
template <IsQuantity Q>
  requires(!std::is_same_v<Q, Number>)
constexpr Q operator/(Q quantity, Number divisor) {
  return Q(quantity.internal() / divisor.internal());
}

/**
 * Divide a Number by a Quantity
 */
template <IsQuantity Q>
  requires(!std::is_same_v<Q, Number>)
constexpr auto operator/(Number enumerator, Q divisor) {
  return Divided<Number, Q>(enumerator.internal() / divisor.internal());
}

/**
 * Multiply two Quantities
 */
template <IsQuantity Q1, IsQuantity Q2>
constexpr auto operator*(Q1 lhs, Q2 rhs) {
  using Result = Multiplied<Q1, Q2>;
  if constexpr (std::is_same_v<Result, Number>)
    return lhs.internal() * rhs.internal();
  else
    return Result(lhs.internal() * rhs.internal());
}

/**
 * Divide two Quantities
 */
template <IsQuantity Q1, IsQuantity Q2>
constexpr auto operator/(Q1 lhs, Q2 rhs) {
  using Result = Divided<Q1, Q2>;
  if constexpr (std::is_same_v<Result, Number>)
    return lhs.internal() / rhs.internal();
  else
    return Result(lhs.internal() / rhs.internal());
}

/**
 * Check whether two Quantities are equal
 */
template <IsQuantity Q, IsQuantity R>
constexpr bool operator==(const Q &lhs, const R &rhs)
  requires Isomorphic<Q, R>
{
  return (lhs.internal() == rhs.internal());
}

/**
 * Check whether two Quantities are not equal
 */
template <IsQuantity Q, IsQuantity R>
constexpr bool operator!=(const Q &lhs, const R &rhs)
  requires Isomorphic<Q, R>
{
  return (lhs.internal() != rhs.internal());
}

/**
 * Check whether a Quantity is less than or equal to another
 */
template <IsQuantity Q, IsQuantity R>
constexpr bool operator<=(const Q &lhs, const R &rhs)
  requires Isomorphic<Q, R>
{
  return (lhs.internal() <= rhs.internal());
}

/**
 * Check whether a Quantity is greater than or equal to another
 */
template <IsQuantity Q, IsQuantity R>
constexpr bool operator>=(const Q &lhs, const R &rhs)
  requires Isomorphic<Q, R>
{
  return (lhs.internal() >= rhs.internal());
}

/**
 * Check whether a Quantity is less than another
 */
template <IsQuantity Q, IsQuantity R>
constexpr bool operator<(const Q &lhs, const R &rhs)
  requires Isomorphic<Q, R>
{
  return (lhs.internal() < rhs.internal());
}

/**
 * Check whether a Quantity is greater than another
 */
template <IsQuantity Q, IsQuantity R>
constexpr bool operator>(const Q &lhs, const R &rhs)
  requires Isomorphic<Q, R>
{
  return (lhs.internal() > rhs.internal());
}

/**
 * Macro that defines a new unit as a specific Quantity type
 *
 * @param Name name for the unit (e.g. Length)
 * @param long_suffix long suffix for the unit (e.g. meters)
 * @param suffix alternate short suffix for the unit (e.g. m)
 * @param m mass dim
 * @param l length dim
 * @param t time dim
 * @param i current dim
 * @param a angle dim
 * @param o temperature dim
 * @param j luminosity dim
 * @param n moles dim
 */
#define NEW_UNIT(Name, long_suffix, suffix, m, l, t, i, a, o, j, n)            \
  using Name =                                                                 \
      Quantity<std::ratio<m>, std::ratio<l>, std::ratio<t>, std::ratio<i>,     \
               std::ratio<a>, std::ratio<o>, std::ratio<j>, std::ratio<n>>;    \
  [[maybe_unused]]                                                             \
  constexpr Name long_suffix = Name(1.0);                                      \
  constexpr Name suffix = long_suffix;                                         \
  namespace literals {                                                         \
  constexpr Name operator""_##long_suffix(long double value) {                 \
    return Name(                                                               \
        Quantity<std::ratio<m>, std::ratio<l>, std::ratio<t>, std::ratio<i>,   \
                 std::ratio<a>, std::ratio<o>, std::ratio<j>, std::ratio<n>>(  \
            static_cast<double>(value)));                                      \
  }                                                                            \
  constexpr Name operator""_##long_suffix(unsigned long long value) {          \
    return Name(                                                               \
        Quantity<std::ratio<m>, std::ratio<l>, std::ratio<t>, std::ratio<i>,   \
                 std::ratio<a>, std::ratio<o>, std::ratio<j>, std::ratio<n>>(  \
            static_cast<double>(value)));                                      \
  }                                                                            \
  constexpr Name operator""_##suffix(long double value) {                      \
    return Name(                                                               \
        Quantity<std::ratio<m>, std::ratio<l>, std::ratio<t>, std::ratio<i>,   \
                 std::ratio<a>, std::ratio<o>, std::ratio<j>, std::ratio<n>>(  \
            static_cast<double>(value)));                                      \
  }                                                                            \
  constexpr Name operator""_##suffix(unsigned long long value) {               \
    return Name(                                                               \
        Quantity<std::ratio<m>, std::ratio<l>, std::ratio<t>, std::ratio<i>,   \
                 std::ratio<a>, std::ratio<o>, std::ratio<j>, std::ratio<n>>(  \
            static_cast<double>(value)));                                      \
  }                                                                            \
  }

#define NEW_UNIT_LITERAL(Name, long_suffix, suffix, multiple)                  \
  [[maybe_unused]]                                                             \
  constexpr Name suffix = multiple;                                            \
  constexpr Name long_suffix = multiple;                                       \
  namespace literals {                                                         \
  constexpr Name operator""_##long_suffix(long double value) {                 \
    return static_cast<double>(value) * multiple;                              \
  }                                                                            \
  constexpr Name operator""_##suffix(long double value) {                      \
    return static_cast<double>(value) * multiple;                              \
  }                                                                            \
  constexpr Name operator""_##long_suffix(unsigned long long value) {          \
    return static_cast<double>(value) * multiple;                              \
  }                                                                            \
  constexpr Name operator""_##suffix(unsigned long long value) {               \
    return static_cast<double>(value) * multiple;                              \
  }                                                                            \
  }

#define NEW_METRIC_PREFIXES(Name, long_base, base)                             \
  NEW_UNIT_LITERAL(Name, tera##long_base, T##base, base * 1E12)                \
  NEW_UNIT_LITERAL(Name, giga##long_base, G##base, base * 1E9)                 \
  NEW_UNIT_LITERAL(Name, mega##long_base, M##base, base * 1E6)                 \
  NEW_UNIT_LITERAL(Name, kilo##long_base, k##base, base * 1E3)                 \
  NEW_UNIT_LITERAL(Name, centi##long_base, c##base, base / 1E2)                \
  NEW_UNIT_LITERAL(Name, milli##long_base, m##base, base / 1E3)                \
  NEW_UNIT_LITERAL(Name, micro##long_base, u##base, base / 1E6)                \
  NEW_UNIT_LITERAL(Name, nano##long_base, n##base, base / 1E9)                 \
  NEW_UNIT_LITERAL(Name, pico##long_base, p##base, base / 1E12)

NEW_UNIT(Mass, kilograms, kg, 1, 0, 0, 0, 0, 0, 0, 0)
NEW_UNIT_LITERAL(Mass, grams, g, kg / 1000)
NEW_UNIT_LITERAL(Mass, ounces, oz, g * 28.349523125)
NEW_UNIT_LITERAL(Mass, pounds_mass, lbm, oz * 16)
NEW_UNIT_LITERAL(Mass, tons, ton, lbm * 2000)

NEW_UNIT(Time, seconds, s, 0, 0, 1, 0, 0, 0, 0, 0)
NEW_METRIC_PREFIXES(Time, seconds, s)
NEW_UNIT_LITERAL(Time, minutes, mins, s * 60)
NEW_UNIT_LITERAL(Time, hours, hr, mins * 60)
NEW_UNIT_LITERAL(Time, days, day, hr * 24)

NEW_UNIT(Length, meters, m, 0, 1, 0, 0, 0, 0, 0, 0)
NEW_METRIC_PREFIXES(Length, meters, m)
NEW_UNIT_LITERAL(Length, inches, in, cm * 2.54)
NEW_UNIT_LITERAL(Length, feet, ft, in * 12)
NEW_UNIT_LITERAL(Length, yards, yd, ft * 3)
NEW_UNIT_LITERAL(Length, miles, mi, ft * 5280)
/// Tile is the measured length of a vex field tile in real life
NEW_UNIT_LITERAL(Length, tiles, tile, in * 23.75)

NEW_UNIT(Area, square_meters, m2, 0, 2, 0, 0, 0, 0, 0, 0)
NEW_UNIT_LITERAL(Area, square_yards, yd2, yd * yd)
NEW_UNIT_LITERAL(Area, square_feet, ft2, ft * ft)
NEW_UNIT_LITERAL(Area, square_inches, in2, in * in)
NEW_UNIT_LITERAL(Area, acres, acre, yd2 * 4840)

NEW_UNIT(Volume, cubic_meters, m3, 0, 3, 0, 0, 0, 0, 0, 0)
NEW_UNIT_LITERAL(Volume, cubic_feet, ft3, ft * ft * ft)
NEW_UNIT_LITERAL(Volume, cubic_inches, in3, in * in * in)
NEW_UNIT_LITERAL(Volume, liters, liter, m3 * 0.001)
NEW_UNIT_LITERAL(Volume, gallons, gal, in3 * 231)

NEW_UNIT(Velocity, meters_per_second, mps, 0, 1, -1, 0, 0, 0, 0, 0)
NEW_METRIC_PREFIXES(Velocity, mps, mps)
NEW_UNIT_LITERAL(Velocity, inches_per_second, inps, in / s)
NEW_UNIT_LITERAL(Velocity, feet_per_second, ftps, ft / s)
NEW_UNIT_LITERAL(Velocity, miles_per_hour, miph, mi / hr)

NEW_UNIT(Acceleration, meters_per_second_squared, mps2, 0, 1, -2, 0, 0, 0, 0, 0)
NEW_METRIC_PREFIXES(Acceleration, mps2, mps2)
NEW_UNIT_LITERAL(Acceleration, inches_per_second_squared, inps2, in / (s * s))

NEW_UNIT(Jerk, meters_per_second_cubed, mps3, 0, 1, -3, 0, 0, 0, 0, 0)
NEW_METRIC_PREFIXES(Jerk, mps3, mps3)
NEW_UNIT_LITERAL(Jerk, inches_per_second_cubed, inps3, in / (s * s * s))

NEW_UNIT(Absement, meter_seconds, m_s, 0, 1, 1, 0, 0, 0, 0, 0)
NEW_UNIT_LITERAL(Absement, inch_seconds, in_s, in * s)

NEW_UNIT(Angle, radians, rad, 0, 0, 0, 0, 1, 0, 0, 0)
NEW_UNIT_LITERAL(Angle, degrees, deg, rad *std::numbers::pi / 180)
NEW_UNIT_LITERAL(Angle, revolutions, rev, rad *std::numbers::pi * 2)
NEW_UNIT_LITERAL(Angle, gradians, grad, deg * 0.9)

NEW_UNIT(AngularVelocity, radians_per_second, radps, 0, 0, -1, 0, 1, 0, 0, 0)
NEW_UNIT_LITERAL(AngularVelocity, degrees_per_second, dps, deg / s)
NEW_UNIT_LITERAL(AngularVelocity, revolutions_per_second, revps, rev / s)
NEW_UNIT_LITERAL(AngularVelocity, revolutions_per_minute, rpm, rev / mins)

NEW_UNIT(AngularAcceleration, radians_per_second_squared, radps2, 0, 0, -2, 0,
         1, 0, 0, 0)
NEW_UNIT_LITERAL(AngularAcceleration, degrees_per_second_squared, dps2,
                 deg / (s * s))
NEW_UNIT_LITERAL(AngularAcceleration, revolutions_per_second_squared, revps2,
                 rev / (s * s))

NEW_UNIT(AngularJerk, radians_per_second_cubed, radps3, 0, 0, -3, 0, 1, 0, 0, 0)
NEW_UNIT_LITERAL(AngularJerk, degrees_per_second_cubed, dps3, deg / (s * s * s))
NEW_UNIT_LITERAL(AngularJerk, revolutions_per_second_cubed, revps3,
                 rev / (s * s * s))

NEW_UNIT(AngularAbsement, radian_seconds, rad_s, 0, 0, 1, 0, 1, 0, 0, 0)
NEW_UNIT_LITERAL(AngularAbsement, degree_seconds, d_s, deg * s)
NEW_UNIT_LITERAL(AngularAbsement, revolution_seconds, rev_s, rev * s)

NEW_UNIT(Curvature, radians_per_meter, radpm, 0, -1, 0, 0, 1, 0, 0, 0)
NEW_UNIT_LITERAL(Curvature, degrees_per_meter, degpm, deg / m)
NEW_UNIT_LITERAL(Curvature, radians_per_inch, radpin, rad / in)
NEW_UNIT_LITERAL(Curvature, degrees_per_inch, degpin, deg / in)

NEW_UNIT(Frequency, hertz, Hz, 0, 0, -1, 0, 0, 0, 0, 0)
NEW_METRIC_PREFIXES(Frequency, hertz, Hz)

NEW_UNIT(Charge, coulombs, coulomb, 0, 0, 1, 1, 0, 0, 0, 0)

NEW_UNIT(Voltage, volts, V, 1, 2, -3, -1, 0, 0, 0, 0)
NEW_METRIC_PREFIXES(Voltage, volts, V)

NEW_UNIT(Resistance, ohms, ohm, 1, 2, -3, -2, 0, 0, 0, 0)
NEW_METRIC_PREFIXES(Resistance, ohms, ohm)

NEW_UNIT(Conductance, siemens, S, -1, -2, 3, 2, 0, 0, 0, 0)
NEW_METRIC_PREFIXES(Conductance, siemens, S)

NEW_UNIT(Capacitance, farads, F, -1, -2, 4, 2, 0, 0, 0, 0)
NEW_METRIC_PREFIXES(Capacitance, farads, F)

NEW_UNIT(Inductance, henries, H, 1, 2, -2, -2, 0, 0, 0, 0)
NEW_METRIC_PREFIXES(Inductance, henries, H)

NEW_UNIT(Force, newtons, N, 1, 1, -2, 0, 0, 0, 0, 0)
NEW_METRIC_PREFIXES(Force, newtons, N)
NEW_UNIT_LITERAL(Force, pounds_force, lbf, lbm * 9.80665 * m / (s * s))

NEW_UNIT(Power, watts, W, 1, 2, -3, 0, 0, 0, 0, 0)
NEW_METRIC_PREFIXES(Power, watts, W)

NEW_UNIT(Momentum, kilogram_meters_per_second, kgmps, 1, 1, -1, 0, 0, 0, 0, 0)
NEW_UNIT_LITERAL(Momentum, newton_seconds, Ns, N * s)

/**
 * Inertia as in moment of inertia is divided by radians^2
 * normally kg*m^2, but also J*s^2 / rad^2... rad is normally dimensionless, not
 * here
 */
NEW_UNIT(Inertia, kilogram_meters_squared, kgm2, 1, 2, 0, 0, -2, 0, 0, 0)

NEW_UNIT(Energy, joules, J, 1, 2, -2, 0, 0, 0, 0, 0)
NEW_METRIC_PREFIXES(Energy, joules, J)

/**
 * Torque and Energy are NOT the same here, torque is divided by radians...
 * as in, energy per radian. J and Nm are normally the same but here they're not
 */
NEW_UNIT(Torque, newton_meters, Nm, 1, 2, -2, 0, -1, 0, 0, 0)
NEW_UNIT_LITERAL(Torque, pound_feet, lbft, Nm * 1.3558179483)

NEW_UNIT(Current, amps, A, 0, 0, 0, 1, 0, 0, 0, 0)
NEW_METRIC_PREFIXES(Current, amps, A)

NEW_UNIT(LuminousIntensity, candelas, candela, 0, 0, 0, 0, 0, 0, 1, 0)

NEW_UNIT(Amount, moles, mol, 0, 0, 0, 0, 0, 0, 0, 1)

NEW_UNIT(LinearVelocityFeedforward, volts_per_meter_per_second, VpMps, 1, 1, -2,
         -1, 0, 0, 0, 0)
NEW_UNIT_LITERAL(LinearVelocityFeedforward, volts_per_inch_per_second, VpInps,
                 V / inps)

NEW_UNIT(LinearProportionalGain, volts_per_meter, VpM, 1, 1, -3, -1, 0, 0, 0, 0)
NEW_UNIT_LITERAL(LinearProportionalGain, volts_per_inch, VpIn, V / in)

NEW_UNIT(LinearIntegralGain, volts_per_meter_second, VpMS, 1, 1, -4, -1, 0, 0,
         0, 0)
NEW_UNIT_LITERAL(LinearIntegralGain, volts_per_inch_second, VpInS, V / (in * s))

NEW_UNIT(LinearAccelerationFeedforward, volts_per_meter_per_second_squared,
         VpMps2, 1, 1, -1, -1, 0, 0, 0, 0)
NEW_UNIT_LITERAL(LinearAccelerationFeedforward,
                 volts_per_inch_per_second_squared, VpInps2, V / inps2)

NEW_UNIT(LinearDerivativeGain, volt_seconds_per_meter, VspM, 1, 1, -2, -1, 0, 0,
         0, 0)
NEW_UNIT_LITERAL(LinearDerivativeGain, volt_seconds_per_inch, VspIn, V / inps)

/**
 * These are the same dimension, and we don't have special names, so "using" is
 * cleanest
 */
using LinearVelocityProportionalGain = LinearDerivativeGain;

NEW_UNIT(LinearVelocityDerivativeGain, volt_seconds_squared_per_meter, Vs2pM, 1,
         1, -1, -1, 0, 0, 0, 0)
NEW_UNIT_LITERAL(LinearVelocityDerivativeGain, volt_seconds_squared_per_inch,
                 Vs2pIn, V / inps2)

using LinearVelocityIntegralGain = LinearProportionalGain;

NEW_UNIT(AngularVelocityFeedforward, volts_per_radian_per_second, VpRadPs, 1, 2,
         -2, -1, -1, 0, 0, 0)
NEW_UNIT_LITERAL(AngularVelocityFeedforward, volts_per_degree_per_second,
                 VpDegPs, V / dps)

NEW_UNIT(AngularProportionalGain, volts_per_radian, VpRad, 1, 2, -3, -1, -1, 0,
         0, 0)
NEW_UNIT_LITERAL(AngularProportionalGain, volts_per_degree, VpDeg, V / deg)

NEW_UNIT(AngularIntegralGain, volts_per_radian_second, VpRadS, 1, 2, -4, -1, -1,
         0, 0, 0)
NEW_UNIT_LITERAL(AngularIntegralGain, volts_per_degree_second, VpDegS,
                 V / (deg * s))

NEW_UNIT(AngularAccelerationFeedforward, volts_per_radian_per_second_squared,
         VpRadPs2, 1, 2, -1, -1, -1, 0, 0, 0)
NEW_UNIT_LITERAL(AngularAccelerationFeedforward,
                 volts_per_degree_per_second_squared, VpDegPs2, V / dps2)

NEW_UNIT(AngularDerivativeGain, volt_seconds_per_radian, VspRad, 1, 2, -2, -1,
         -1, 0, 0, 0)
NEW_UNIT_LITERAL(AngularDerivativeGain, volt_seconds_per_degree, VspDeg,
                 V / dps)

using AngularVelocityProportionalGain = AngularDerivativeGain;

NEW_UNIT(AngularVelocityDerivativeGain, volt_seconds_squared_per_radian,
         Vs2pRad, 1, 2, -1, -1, -1, 0, 0, 0)
NEW_UNIT_LITERAL(AngularVelocityDerivativeGain, volt_seconds_squared_per_degree,
                 Vs2pDeg, V / dps2)

using AngularVelocityIntegralGain = AngularProportionalGain;

#undef NEW_METRIC_PREFIXES
#undef NEW_UNIT_LITERAL
#undef NEW_UNIT

/**
 * Temperature gets special treatment since its conversions are affine.
 * Addition subtraction and negation are all just not allowed. Do it manually.
 */
using TemperatureQuantity =
    Quantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>,
             std::ratio<0>, std::ratio<1>, std::ratio<0>, std::ratio<0>>;

enum class TemperatureUnit { Kelvin, Celsius, Fahrenheit };

class Temperature : public TemperatureQuantity {
public:
  constexpr Temperature() = default;
  explicit constexpr Temperature(double kelvins)
      : TemperatureQuantity(kelvins) {}

  constexpr double to(TemperatureUnit unit) const;

  constexpr Temperature &operator+=(Temperature) = delete;
  constexpr Temperature &operator-=(Temperature) = delete;
};

constexpr TemperatureUnit kelvin = TemperatureUnit::Kelvin;
constexpr TemperatureUnit K = kelvin;
constexpr TemperatureUnit celsius = TemperatureUnit::Celsius;
constexpr TemperatureUnit fahrenheit = TemperatureUnit::Fahrenheit;

constexpr Temperature from(double value, TemperatureUnit unit) {
  switch (unit) {
  case TemperatureUnit::Kelvin:
    return Temperature(value);
  case TemperatureUnit::Celsius:
    return Temperature(value + 273.15);
  case TemperatureUnit::Fahrenheit:
    return Temperature((value - 32.0) * (5.0 / 9.0) + 273.15);
  }
  return Temperature(value);
}

constexpr double Temperature::to(TemperatureUnit unit) const {
  switch (unit) {
  case TemperatureUnit::Kelvin:
    return internal();
  case TemperatureUnit::Celsius:
    return internal() - 273.15;
  case TemperatureUnit::Fahrenheit:
    return (internal() - 273.15) * (9.0 / 5.0) + 32.0;
  }
  return internal();
}

constexpr Temperature operator+(Temperature) = delete;
constexpr Temperature operator-(Temperature) = delete;
constexpr Temperature operator+(Temperature, Temperature) = delete;
constexpr Temperature operator-(Temperature, Temperature) = delete;

namespace literals {

constexpr Temperature operator""_kelvin(long double value) {
  return from(static_cast<double>(value), kelvin);
}

constexpr Temperature operator""_kelvin(unsigned long long value) {
  return from(static_cast<double>(value), kelvin);
}

constexpr Temperature operator""_K(long double value) {
  return from(static_cast<double>(value), kelvin);
}

constexpr Temperature operator""_K(unsigned long long value) {
  return from(static_cast<double>(value), kelvin);
}

constexpr Temperature operator""_celsius(long double value) {
  return from(static_cast<double>(value), celsius);
}

constexpr Temperature operator""_celsius(unsigned long long value) {
  return from(static_cast<double>(value), celsius);
}

constexpr Temperature operator""_fahrenheit(long double value) {
  return from(static_cast<double>(value), fahrenheit);
}

constexpr Temperature operator""_fahrenheit(unsigned long long value) {
  return from(static_cast<double>(value), fahrenheit);
}

} // namespace literals

/**
 * Helper that converts arithmetic types to Number so they can be used in the
 * following
 */
template <typename T>
constexpr auto to_quantity(T value)
    -> std::conditional_t<std::is_arithmetic_v<T>, Number, T> {
  if constexpr (std::is_arithmetic_v<T>)
    return Number(value);
  else
    return value;
}

template <typename T>
using quantity_type = decltype(to_quantity(std::declval<T>()));

template <typename T, typename... U>
concept IsomorphicValues = Isomorphic<quantity_type<T>, quantity_type<U>...>;

/**
 * Using this first function as an example, the "if consteval" checks whether
 * it is being executed at compile time or runtime.
 * std::abs is not marked constexpr, gcem::abs is. We prefer std::abs at runtime
 * since it uses libm (or compiler intrinsics) for this specific cpu rather than
 * gcem's generic abs implementation. This mostly only matters for things like
 * sin or pow where performance might matter.
 */
template <typename T> constexpr T abs(const T &lhs) {
  auto q = to_quantity(lhs);
  using Q = decltype(q);
  if consteval {
    return Q(cevalm::abs(q.internal()));
  } else {
    return Q(std::abs(q.internal()));
  }
}

template <typename T, typename U>
  requires IsomorphicValues<T, U>
constexpr auto max(const T &lhs, const U &rhs) {
  auto qlhs = to_quantity(lhs);
  auto qrhs = to_quantity(rhs);
  return (qlhs > qrhs ? qlhs : qrhs);
}

template <typename T, typename U>
  requires IsomorphicValues<T, U>
constexpr auto min(const T &lhs, const U &rhs) {
  auto qlhs = to_quantity(lhs);
  auto qrhs = to_quantity(rhs);
  return (qlhs < qrhs ? qlhs : qrhs);
}

template <typename T> constexpr auto sgn(const T &lhs) {
  auto q = to_quantity(lhs);
  if (q.internal() > 0)
    return Number(1);
  if (q.internal() < 0)
    return Number(-1);
  return Number(0);
}

template <int R, typename T> constexpr auto pow(const T &lhs) {
  auto q = to_quantity(lhs);
  using Q = decltype(q);
  using S = Exponentiated<Q, std::ratio<R>>;
  if consteval {
    return S(cevalm::pow(q.internal(), R));
  } else {
    return S(std::pow(q.internal(), R));
  }
}

template <typename T> constexpr auto square(const T &lhs) { return lhs * lhs; }

template <typename T> constexpr auto cube(const T &lhs) {
  return lhs * lhs * lhs;
}

template <int R, typename T>
constexpr auto root(const T &lhs)
  requires(R > 0)
{
  auto q = to_quantity(lhs);
  using Q = decltype(q);
  using S = Rooted<Q, std::ratio<R>>;

  const double val = q.internal();
  const bool odd_and_negative = (R % 2 != 0) && (val < 0);
  const double base = odd_and_negative ? -val : val;

  double res{};

  if (R == 2) {
    if consteval {
      res = cevalm::sqrt(base);
    } else {
      res = std::sqrt(base);
    }
  } else if (R == 3) {
    if consteval {
      res = cevalm::cbrt(base);
    } else {
      res = std::cbrt(base);
    }
  } else {
    if consteval {
      res = cevalm::pow(base, 1.0 / R);
    } else {
      res = std::pow(base, 1.0 / R);
    }
  }

  return S(odd_and_negative ? -res : res);
}

template <typename T> constexpr auto sqrt(const T &lhs) { return root<2>(lhs); }

template <typename T> constexpr auto cbrt(const T &lhs) { return root<3>(lhs); }

template <typename T, typename U>
  requires IsomorphicValues<T, U>
constexpr auto hypot(const T &lhs, const U &rhs) {
  auto qlhs = to_quantity(lhs);
  auto qrhs = to_quantity(rhs);
  if consteval {
    return decltype(qlhs)(cevalm::hypot(qlhs.internal(), qrhs.internal()));
  } else {
    return decltype(qlhs)(std::hypot(qlhs.internal(), qrhs.internal()));
  }
}

template <typename T, typename U>
  requires IsomorphicValues<T, U>
constexpr auto mod(const T &lhs, const U &rhs) {
  auto qlhs = to_quantity(lhs);
  auto qrhs = to_quantity(rhs);
  if consteval {
    return decltype(qlhs)(cevalm::fmod(qlhs.internal(), qrhs.internal()));
  } else {
    return decltype(qlhs)(std::fmod(qlhs.internal(), qrhs.internal()));
  }
}

template <typename T, typename U>
constexpr auto copysign(const T &lhs, const U &rhs) {
  auto qlhs = to_quantity(lhs);
  auto qrhs = to_quantity(rhs);
  if consteval {
    return decltype(qlhs)(cevalm::copysign(qlhs.internal(), qrhs.internal()));
  } else {
    return decltype(qlhs)(std::copysign(qlhs.internal(), qrhs.internal()));
  }
}

template <typename T> constexpr auto signbit(const T &lhs) {
  auto q = to_quantity(lhs);
  return std::signbit(q.internal());
}

template <typename T, typename U, typename V>
  requires IsomorphicValues<T, U, V>
constexpr auto clamp(const T &lhs, const U &lo, const V &hi) {
  auto qlhs = to_quantity(lhs);
  auto qlo = to_quantity(lo);
  auto qhi = to_quantity(hi);
  return decltype(qlhs)(
      std::clamp(qlhs.internal(), qlo.internal(), qhi.internal()));
}

template <typename T, typename U>
  requires IsomorphicValues<T, U>
constexpr auto ceil(const T &lhs, const U &rhs) {
  auto qlhs = to_quantity(lhs);
  auto qrhs = to_quantity(rhs);
  if consteval {
    return decltype(qlhs)(cevalm::ceil(qlhs.internal() / qrhs.internal()) *
                          qrhs.internal());
  } else {
    return decltype(qlhs)(std::ceil(qlhs.internal() / qrhs.internal()) *
                          qrhs.internal());
  }
}

template <typename T, typename U>
  requires IsomorphicValues<T, U>
constexpr auto floor(const T &lhs, const U &rhs) {
  auto qlhs = to_quantity(lhs);
  auto qrhs = to_quantity(rhs);
  if consteval {
    return decltype(qlhs)(cevalm::floor(qlhs.internal() / qrhs.internal()) *
                          qrhs.internal());
  } else {
    return decltype(qlhs)(std::floor(qlhs.internal() / qrhs.internal()) *
                          qrhs.internal());
  }
}

template <typename T, typename U>
  requires IsomorphicValues<T, U>
constexpr auto trunc(const T &lhs, const U &rhs) {
  auto qlhs = to_quantity(lhs);
  auto qrhs = to_quantity(rhs);
  if consteval {
    return decltype(qlhs)(cevalm::trunc(qlhs.internal() / qrhs.internal()) *
                          qrhs.internal());
  } else {
    return decltype(qlhs)(std::trunc(qlhs.internal() / qrhs.internal()) *
                          qrhs.internal());
  }
}

template <typename T, typename U>
  requires IsomorphicValues<T, U>
constexpr auto round(const T &lhs, const U &rhs) {
  auto qlhs = to_quantity(lhs);
  auto qrhs = to_quantity(rhs);
  if consteval {
    return decltype(qlhs)(cevalm::round(qlhs.internal() / qrhs.internal()) *
                          qrhs.internal());
  } else {
    return decltype(qlhs)(std::round(qlhs.internal() / qrhs.internal()) *
                          qrhs.internal());
  }
}

/**
 * Only allows nonzero length, time, angle for the next two functions
 */
template <typename Q>
concept KinematicQuantity =
    IsQuantity<Q> && std::ratio_equal_v<typename Q::mass, std::ratio<0>> &&
    std::ratio_equal_v<typename Q::current, std::ratio<0>> &&
    std::ratio_equal_v<typename Q::temperature, std::ratio<0>> &&
    std::ratio_equal_v<typename Q::luminosity, std::ratio<0>> &&
    std::ratio_equal_v<typename Q::moles, std::ratio<0>>;

/**
 * Helper to go from angular to linear, like from encoder rotation to wheel
 * distance travel
 */
template <KinematicQuantity Q>
  requires std::ratio_equal_v<typename Q::angle, std::ratio<1>> &&
           std::ratio_equal_v<typename Q::length, std::ratio<0>>
constexpr auto to_linear(Q angular_distance, Length diameter) {
  return angular_distance * (diameter / 2.0) / rad;
}

/**
 * Helper to go from linear to angular, like from wheel distance travel to wheel
 * rotation
 */
template <KinematicQuantity Q>
  requires std::ratio_equal_v<typename Q::length, std::ratio<1>> &&
           std::ratio_equal_v<typename Q::angle, std::ratio<0>>
constexpr auto to_angular(Q linear_distance, Length diameter) {
  return linear_distance / (diameter / 2.0) * rad;
}

/**
 * Trig stuff
 */
constexpr Number sin(Angle angle) {
  if consteval {
    return Number(cevalm::sin(angle.internal()));
  } else {
    return Number(std::sin(angle.internal()));
  }
}

constexpr Number cos(Angle angle) {
  if consteval {
    return Number(cevalm::cos(angle.internal()));
  } else {
    return Number(std::cos(angle.internal()));
  }
}

constexpr Number tan(Angle angle) {
  if consteval {
    return Number(cevalm::tan(angle.internal()));
  } else {
    return Number(std::tan(angle.internal()));
  }
}

constexpr Angle asin(Number value) {
  if consteval {
    return Angle(cevalm::asin(value.internal()));
  } else {
    return Angle(std::asin(value.internal()));
  }
}

constexpr Angle acos(Number value) {
  if consteval {
    return Angle(cevalm::acos(value.internal()));
  } else {
    return Angle(std::acos(value.internal()));
  }
}

constexpr Angle atan(Number value) {
  if consteval {
    return Angle(cevalm::atan(value.internal()));
  } else {
    return Angle(std::atan(value.internal()));
  }
}

template <typename T, typename U>
  requires IsomorphicValues<T, U>
constexpr Angle atan2(const T &y, const U &x) {
  auto qy = to_quantity(y);
  auto qx = to_quantity(x);
  if consteval {
    return Angle(cevalm::atan2(qy.internal(), qx.internal()));
  } else {
    return Angle(std::atan2(qy.internal(), qx.internal()));
  }
}

/**
 * Angle wrapping
 */
constexpr Angle wrap_positive(Angle angle) {
  Angle wrapped = mod(angle, rev);
  return wrapped < Angle(0) ? wrapped + rev : wrapped;
}

constexpr Angle wrap_signed(Angle angle) {
  return wrap_positive(angle + rev / 2.0) - rev / 2.0;
}

constexpr Angle shortest_difference(Angle start, Angle end) {
  return wrap_signed(end - start);
}

} // namespace units
