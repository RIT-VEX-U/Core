#pragma once

#include <ostream>
#include <sstream>
#include <string>
#include <type_traits>

#include "gcem.hpp"

namespace units {

// Terminology
// Quantity is a type like Length or Time
// Unit is a representation of a Quantity like inches or seconds
// Quantities are defined by their Dimensions, the exponents of the base SI
// units here we allow algebra on those dimensions so arithmetic can derive
// result quantities

template <int M, int L, int Ti, int I, int Th, int N, int J, int A>
struct dim {
  static constexpr int mass = M;
  static constexpr int length = L;
  static constexpr int time = Ti;
  static constexpr int current = I;
  static constexpr int temperature = Th;
  static constexpr int amount = N;
  static constexpr int luminous_intensity = J;
  static constexpr int angle = A;
};

template <class LeftDim, class RightDim>
struct dim_add {
  typedef dim<LeftDim::mass + RightDim::mass, LeftDim::length + RightDim::length, LeftDim::time + RightDim::time,
              LeftDim::current + RightDim::current, LeftDim::temperature + RightDim::temperature,
              LeftDim::amount + RightDim::amount, LeftDim::luminous_intensity + RightDim::luminous_intensity,
              LeftDim::angle + RightDim::angle>
      type;
};

template <class LeftDim, class RightDim>
struct dim_sub {
  typedef dim<LeftDim::mass - RightDim::mass, LeftDim::length - RightDim::length, LeftDim::time - RightDim::time,
              LeftDim::current - RightDim::current, LeftDim::temperature - RightDim::temperature,
              LeftDim::amount - RightDim::amount, LeftDim::luminous_intensity - RightDim::luminous_intensity,
              LeftDim::angle - RightDim::angle>
      type;
};

typedef dim<0, 0, 0, 0, 0, 0, 0, 0> dim_none;

// Unit metadata
// adapts unit tags into conversion factors, symbols, names, and defaults

template <class UnitTag>
struct unit_traits {
  typedef typename UnitTag::dimension dimension;
  static constexpr double scale_num = UnitTag::scale_num;
  static constexpr double scale_den = UnitTag::scale_den;
  static constexpr bool is_affine = UnitTag::is_affine;
};

template <class UnitTag>
struct unit_text {
  static const char* symbol() { return UnitTag::symbol(); }
  static const char* name() { return UnitTag::name(); }
};

template <class Quantity>
struct default_unit_for_quantity;

template <class Quantity>
struct dimension_for_quantity;

// Quantity lookup
// maps dim to the named quantity, or anonymous quantity if one does not exist

template <class Dim>
class anonymous_quantity;

template <class Dim>
anonymous_quantity<Dim> quantity_for_dimension_probe(Dim*, ...);

template <class Dim>
struct quantity_for_dimension {
  typedef decltype(quantity_for_dimension_probe(static_cast<Dim*>(0), 0)) type;
};

template <class Quantity>
struct is_quantity : std::false_type {};

// Result constraints for quantity operators and math functions
// determines what functions are enabled... i.e. length + time doesn't exist but
// length + length does

template <class Left, class Right, bool Enabled = is_quantity<Left>::value && is_quantity<Right>::value>
struct same_dimension : std::false_type {};

template <class Left, class Right>
struct same_dimension<Left, Right, true> : std::is_same<typename Left::dimension, typename Right::dimension> {};

template <bool Condition, class Type>
using enable_if_t = typename std::enable_if<Condition, Type>::type;

template <class Quantity>
using quantity_result_t = enable_if_t<is_quantity<Quantity>::value, Quantity>;

template <class Left, class Right>
using same_dimension_result_t =
    enable_if_t<same_dimension<Left, Right>::value, typename quantity_for_dimension<typename Left::dimension>::type>;

template <class Left, class Right>
using dim_add_result_t = enable_if_t<
    is_quantity<Left>::value && is_quantity<Right>::value,
    typename quantity_for_dimension<typename dim_add<typename Left::dimension, typename Right::dimension>::type>::type>;

template <class Left, class Right>
using dim_sub_result_t = enable_if_t<
    is_quantity<Left>::value && is_quantity<Right>::value,
    typename quantity_for_dimension<typename dim_sub<typename Left::dimension, typename Right::dimension>::type>::type>;

template <class Quantity>
using inverse_quantity_result_t =
    enable_if_t<is_quantity<Quantity>::value,
                typename quantity_for_dimension<typename dim_sub<dim_none, typename Quantity::dimension>::type>::type>;

template <class Left, class Right>
using comparison_result_t = enable_if_t<same_dimension<Left, Right>::value, bool>;

template <int Factor, class Dim>
struct dim_scale {
  typedef dim<Dim::mass * Factor, Dim::length * Factor, Dim::time * Factor, Dim::current * Factor,
              Dim::temperature * Factor, Dim::amount * Factor, Dim::luminous_intensity * Factor, Dim::angle * Factor>
      type;
};

template <class Dim, int Divisor>
struct dim_divisible_by
    : std::integral_constant<bool, Dim::mass % Divisor == 0 && Dim::length % Divisor == 0 && Dim::time % Divisor == 0 &&
                                       Dim::current % Divisor == 0 && Dim::temperature % Divisor == 0 &&
                                       Dim::amount % Divisor == 0 && Dim::luminous_intensity % Divisor == 0 &&
                                       Dim::angle % Divisor == 0> {};

template <class Dim, int Divisor>
struct dim_divide {
  typedef dim<Dim::mass / Divisor, Dim::length / Divisor, Dim::time / Divisor, Dim::current / Divisor,
              Dim::temperature / Divisor, Dim::amount / Divisor, Dim::luminous_intensity / Divisor,
              Dim::angle / Divisor>
      type;
};

template <class Quantity, int Factor>
using scaled_dimension_t = typename dim_scale<Factor, typename Quantity::dimension>::type;
template <class Quantity, int Divisor>
using divided_dimension_t = typename dim_divide<typename Quantity::dimension, Divisor>::type;
template <class Quantity, int Factor>
using scaled_quantity_t = typename quantity_for_dimension<scaled_dimension_t<Quantity, Factor> >::type;
template <class Quantity, int Divisor>
using divided_quantity_t = typename quantity_for_dimension<divided_dimension_t<Quantity, Divisor> >::type;

template <class Quantity, int Factor, bool = is_quantity<Quantity>::value>
struct can_scale_quantity_impl : std::false_type {};
template <class Quantity, int Factor>
struct can_scale_quantity_impl<Quantity, Factor, true> : std::true_type {};
template <class Quantity, int Divisor, bool = is_quantity<Quantity>::value>
struct can_root_quantity_impl : std::false_type {};
template <class Quantity, int Divisor>
struct can_root_quantity_impl<Quantity, Divisor, true> : dim_divisible_by<typename Quantity::dimension, Divisor> {};
template <class Quantity, int Factor>
struct can_scale_quantity : can_scale_quantity_impl<Quantity, Factor> {};
template <class Quantity, int Divisor>
struct can_root_quantity : can_root_quantity_impl<Quantity, Divisor> {};
template <class Quantity>
using quantity_bool_result_t = enable_if_t<is_quantity<Quantity>::value, bool>;
template <class Quantity, int Power>
using power_result_t = enable_if_t<can_scale_quantity<Quantity, Power>::value, scaled_quantity_t<Quantity, Power> >;
template <class Quantity, int Divisor>
using root_result_t = enable_if_t<can_root_quantity<Quantity, Divisor>::value, divided_quantity_t<Quantity, Divisor> >;

// Quantity storage
// stores canonical values and provides conversion and compound operators

template <class Derived, class Dim>
class quantity_base {
 public:
  typedef Dim dimension;

  constexpr explicit quantity_base(double canonical_value = 0.0) : canonical_value_(canonical_value) {}

  template <class OtherQuantity>
  constexpr quantity_base(const OtherQuantity& other) : canonical_value_(other.canonical_value()) {}

  constexpr double canonical_value() const { return canonical_value_; }

  template <class UnitTag>
  constexpr double as() const {
    static_assert(std::is_same<typename unit_traits<UnitTag>::dimension, Dim>::value, "unit dimension mismatch");
    return canonical_value_ * unit_traits<UnitTag>::scale_den / unit_traits<UnitTag>::scale_num;
  }

  static constexpr Derived from_canonical(double value) { return Derived(value); }

  template <class UnitTag>
  static constexpr Derived from(double value) {
    return Derived(to_canonical<UnitTag>(value));
  }

 protected:
  template <class UnitTag>
  static constexpr double to_canonical(double value) {
    static_assert(std::is_same<typename unit_traits<UnitTag>::dimension, Dim>::value, "unit dimension mismatch");
    return value * unit_traits<UnitTag>::scale_num / unit_traits<UnitTag>::scale_den;
  }

  void set_canonical_value(double canonical_value) { canonical_value_ = canonical_value; }

 public:
  template <class OtherQuantity>
  Derived& operator+=(const OtherQuantity& other) {
    set_canonical_value(canonical_value() + other.canonical_value());
    return static_cast<Derived&>(*this);
  }

  template <class OtherQuantity>
  Derived& operator-=(const OtherQuantity& other) {
    set_canonical_value(canonical_value() - other.canonical_value());
    return static_cast<Derived&>(*this);
  }

  Derived& operator*=(double scalar) {
    set_canonical_value(canonical_value() * scalar);
    return static_cast<Derived&>(*this);
  }

  Derived& operator/=(double scalar) {
    set_canonical_value(canonical_value() / scalar);
    return static_cast<Derived&>(*this);
  }

  Derived& operator++() {
    set_canonical_value(canonical_value() + 1.0);
    return static_cast<Derived&>(*this);
  }

  Derived operator++(int) {
    Derived copy = static_cast<Derived&>(*this);
    ++(*this);
    return copy;
  }

  Derived& operator--() {
    set_canonical_value(canonical_value() - 1.0);
    return static_cast<Derived&>(*this);
  }

  Derived operator--(int) {
    Derived copy = static_cast<Derived&>(*this);
    --(*this);
    return copy;
  }

 private:
  double canonical_value_;
};

template <class Derived, class Dim>
using semantic_quantity_base = quantity_base<Derived, Dim>;

template <class Dim>
class anonymous_quantity : public quantity_base<anonymous_quantity<Dim>, Dim> {
 public:
  typedef quantity_base<anonymous_quantity<Dim>, Dim> base_type;
  typedef Dim dimension;

  constexpr explicit anonymous_quantity(double canonical_value = 0.0) : base_type(canonical_value) {}

  template <class OtherQuantity>
  constexpr anonymous_quantity(const OtherQuantity& other,
                               enable_if_t<same_dimension<anonymous_quantity<Dim>, OtherQuantity>::value, int> = 0)
      : base_type(other.canonical_value()) {}

  static constexpr anonymous_quantity from_canonical(double value) { return anonymous_quantity(value); }
};

template <class Dim>
struct is_quantity<anonymous_quantity<Dim> > : std::true_type {};

// Arithmetic operators
// combines as canonical values, and makes sure to return the right dims

template <class Left, class Right>
constexpr same_dimension_result_t<Left, Right> operator+(const Left& left, const Right& right) {
  return same_dimension_result_t<Left, Right>::from_canonical(left.canonical_value() + right.canonical_value());
}

template <class Left, class Right>
constexpr same_dimension_result_t<Left, Right> operator-(const Left& left, const Right& right) {
  return same_dimension_result_t<Left, Right>::from_canonical(left.canonical_value() - right.canonical_value());
}

template <class Left, class Right>
constexpr dim_add_result_t<Left, Right> operator*(const Left& left, const Right& right) {
  return dim_add_result_t<Left, Right>::from_canonical(left.canonical_value() * right.canonical_value());
}

template <class Left, class Right>
constexpr dim_sub_result_t<Left, Right> operator/(const Left& left, const Right& right) {
  return dim_sub_result_t<Left, Right>::from_canonical(left.canonical_value() / right.canonical_value());
}

template <class Quantity>
constexpr quantity_result_t<Quantity> operator*(const Quantity& quantity, double scalar) {
  return Quantity::from_canonical(quantity.canonical_value() * scalar);
}

template <class Quantity>
constexpr quantity_result_t<Quantity> operator*(double scalar, const Quantity& quantity) {
  return Quantity::from_canonical(quantity.canonical_value() * scalar);
}

template <class Quantity>
constexpr quantity_result_t<Quantity> operator/(const Quantity& quantity, double scalar) {
  return Quantity::from_canonical(quantity.canonical_value() / scalar);
}

template <class Quantity>
constexpr inverse_quantity_result_t<Quantity> operator/(double scalar, const Quantity& quantity) {
  return inverse_quantity_result_t<Quantity>::from_canonical(scalar / quantity.canonical_value());
}

template <class Quantity>
constexpr quantity_result_t<Quantity> operator-(const Quantity& quantity) {
  return Quantity::from_canonical(-quantity.canonical_value());
}

template <class Quantity>
constexpr quantity_result_t<Quantity> operator+(const Quantity& quantity) {
  return quantity;
}

template <class Left, class Right>
constexpr comparison_result_t<Left, Right> operator==(const Left& left, const Right& right) {
  return left.canonical_value() == right.canonical_value();
}

template <class Left, class Right>
constexpr comparison_result_t<Left, Right> operator!=(const Left& left, const Right& right) {
  return !(left == right);
}

template <class Left, class Right>
constexpr comparison_result_t<Left, Right> operator<(const Left& left, const Right& right) {
  return left.canonical_value() < right.canonical_value();
}

template <class Left, class Right>
constexpr comparison_result_t<Left, Right> operator<=(const Left& left, const Right& right) {
  return left.canonical_value() <= right.canonical_value();
}

template <class Left, class Right>
constexpr comparison_result_t<Left, Right> operator>(const Left& left, const Right& right) {
  return left.canonical_value() > right.canonical_value();
}

template <class Left, class Right>
constexpr comparison_result_t<Left, Right> operator>=(const Left& left, const Right& right) {
  return left.canonical_value() >= right.canonical_value();
}

// Catalog defines quantities... Length, Time, etc, and gives them their units
// number angle and temperature are special cases... not sure what to do with
// angle/rotation2d yet, we will have both for now these files are not really
// complete, they are included halfway through this file because they depend on
// everything above I could not come up with a better way to do this, if you
// open them your lsp will weep... but it works

#include "core/units/catalog.h"
#include "core/units/number.h"
#include "core/units/angle.h"
#include "core/units/temperature.h"

// Math functions that respect quantities
// These wrap gcem to be constexpr, when we upgrade to c++23 we can remove that
// dependency

template <class Quantity>
constexpr quantity_result_t<Quantity> abs(const Quantity& quantity) {
  return Quantity::from_canonical(gcem::abs(quantity.canonical_value()));
}
template <class Quantity>
constexpr quantity_result_t<Quantity> min(const Quantity& left, const Quantity& right) {
  return left < right ? left : right;
}
template <class Quantity>
constexpr quantity_result_t<Quantity> max(const Quantity& left, const Quantity& right) {
  return left < right ? right : left;
}
inline constexpr Number abs(const Number& value) { return Number(gcem::abs(value.value())); }
inline constexpr Number min(const Number& left, const Number& right) {
  return left.value() < right.value() ? left : right;
}
inline constexpr Number max(const Number& left, const Number& right) {
  return left.value() < right.value() ? right : left;
}
template <class Quantity>
inline constexpr Number sgn(const Quantity& quantity) {
  return Number((quantity.canonical_value() > 0.0) - (quantity.canonical_value() < 0.0));
}
inline constexpr Number sgn(const Number& value) { return Number((value.value() > 0.0) - (value.value() < 0.0)); }
template <int Power, class Quantity>
constexpr power_result_t<Quantity, Power> pow(const Quantity& quantity) {
  return scaled_quantity_t<Quantity, Power>::from_canonical(gcem::pow(quantity.canonical_value(), Power));
}
template <int Power>
inline constexpr Number pow(const Number& value) {
  return Number(gcem::pow(value.value(), Power));
}
template <typename Root, class Quantity>
constexpr root_result_t<Quantity, Root::value> root(const Quantity& quantity) {
  return divided_quantity_t<Quantity, Root::value>::from_canonical(
      gcem::pow(quantity.canonical_value(), 1.0 / Root::value));
}
template <typename Root>
inline constexpr Number root(const Number& value) {
  return Number(gcem::pow(value.value(), 1.0 / Root::value));
}
template <class Quantity>
constexpr root_result_t<Quantity, 2> sqrt(const Quantity& quantity) {
  return root<std::integral_constant<int, 2> >(quantity);
}
inline constexpr Number sqrt(const Number& value) { return root<std::integral_constant<int, 2> >(value); }
template <class Quantity>
constexpr root_result_t<Quantity, 3> cbrt(const Quantity& quantity) {
  return root<std::integral_constant<int, 3> >(quantity);
}
inline constexpr Number cbrt(const Number& value) { return root<std::integral_constant<int, 3> >(value); }
template <class Quantity>
constexpr quantity_result_t<Quantity> hypot(const Quantity& left, const Quantity& right) {
  return Quantity::from_canonical(gcem::hypot(left.canonical_value(), right.canonical_value()));
}
inline constexpr Number hypot(const Number& left, const Number& right) {
  return Number(gcem::hypot(left.value(), right.value()));
}
template <class Quantity>
constexpr quantity_result_t<Quantity> mod(const Quantity& left, const Quantity& right) {
  return Quantity::from_canonical(gcem::fmod(left.canonical_value(), right.canonical_value()));
}
inline constexpr Number mod(const Number& left, const Number& right) {
  return Number(gcem::fmod(left.value(), right.value()));
}
template <class Quantity>
constexpr quantity_result_t<Quantity> remainder(const Quantity& left, const Quantity& right) {
  return Quantity::from_canonical(
      left.canonical_value() - gcem::round(left.canonical_value() / right.canonical_value()) * right.canonical_value());
}
inline constexpr Number remainder(const Number& left, const Number& right) {
  return Number(left.value() - gcem::round(left.value() / right.value()) * right.value());
}
template <class Quantity>
constexpr quantity_result_t<Quantity> copysign(const Quantity& left, const Quantity& right) {
  return Quantity::from_canonical(gcem::copysign(left.canonical_value(), right.canonical_value()));
}
inline constexpr Number copysign(const Number& left, const Number& right) {
  return Number(gcem::copysign(left.value(), right.value()));
}
template <class Quantity>
constexpr quantity_bool_result_t<Quantity> signbit(const Quantity& quantity) {
  return gcem::signbit(quantity.canonical_value());
}
inline constexpr bool signbit(const Number& value) { return gcem::signbit(value.value()); }
template <class Quantity>
constexpr quantity_result_t<Quantity> clamp(const Quantity& value, const Quantity& low, const Quantity& high) {
  return max(low, min(value, high));
}
inline constexpr Number clamp(const Number& value, const Number& low, const Number& high) {
  return max(low, min(value, high));
}
template <class Quantity>
constexpr quantity_result_t<Quantity> ceil(const Quantity& value, const Quantity& step) {
  return Quantity::from_canonical(gcem::ceil(value.canonical_value() / step.canonical_value()) *
                                  step.canonical_value());
}
template <class Quantity>
constexpr quantity_result_t<Quantity> floor(const Quantity& value, const Quantity& step) {
  return Quantity::from_canonical(gcem::floor(value.canonical_value() / step.canonical_value()) *
                                  step.canonical_value());
}
template <class Quantity>
constexpr quantity_result_t<Quantity> trunc(const Quantity& value, const Quantity& step) {
  return Quantity::from_canonical(gcem::trunc(value.canonical_value() / step.canonical_value()) *
                                  step.canonical_value());
}
template <class Quantity>
constexpr quantity_result_t<Quantity> round(const Quantity& value, const Quantity& step) {
  return Quantity::from_canonical(gcem::round(value.canonical_value() / step.canonical_value()) *
                                  step.canonical_value());
}
inline constexpr Number ceil(const Number& value, const Number& step) {
  return Number(gcem::ceil(value.value() / step.value()) * step.value());
}
inline constexpr Number floor(const Number& value, const Number& step) {
  return Number(gcem::floor(value.value() / step.value()) * step.value());
}
inline constexpr Number trunc(const Number& value, const Number& step) {
  return Number(gcem::trunc(value.value() / step.value()) * step.value());
}
inline constexpr Number round(const Number& value, const Number& step) {
  return Number(gcem::round(value.value() / step.value()) * step.value());
}
template <class Quantity>
constexpr quantity_result_t<Quantity> fma(const Quantity& x, double y, const Quantity& z) {
  return Quantity::from_canonical(x.canonical_value() * y + z.canonical_value());
}
template <class Quantity>
constexpr quantity_result_t<Quantity> fma(double x, const Quantity& y, const Quantity& z) {
  return Quantity::from_canonical(x * y.canonical_value() + z.canonical_value());
}
inline constexpr Number fma(const Number& x, const Number& y, const Number& z) {
  return Number(x.value() * y.value() + z.value());
}
template <class Quantity>
inline constexpr power_result_t<Quantity, 2> square(const Quantity& quantity) {
  return pow<2>(quantity);
}
inline constexpr Number square(const Number& value) { return pow<2>(value); }
template <class Quantity>
inline constexpr power_result_t<Quantity, 3> cube(const Quantity& quantity) {
  return pow<3>(quantity);
}
inline constexpr Number cube(const Number& value) { return pow<3>(value); }

template <class UnitTag, class Quantity>
inline typename std::enable_if<is_quantity<Quantity>::value, double>::type unit_value_for_text(
    const Quantity& quantity) {
  return quantity.template as<UnitTag>();
}

// Formatting with names for strings or streams

inline std::string humanize_identifier(const char* text) {
  std::string result(text);
  for (std::string::size_type i = 0; i < result.size(); ++i) {
    if (result[i] == '_') result[i] = ' ';
  }
  return result;
}

template <class Quantity>
inline std::string format_with_default_unit(const Quantity& quantity) {
  typedef typename default_unit_for_quantity<Quantity>::type unit_t;
  std::ostringstream stream;
  stream << unit_value_for_text<unit_t>(quantity) << ' ' << unit_text<unit_t>::symbol();
  return stream.str();
}

template <class UnitTag>
inline const char* symbol() {
  return unit_text<UnitTag>::symbol();
}
template <class UnitTag>
inline std::string display_name() {
  return humanize_identifier(unit_text<UnitTag>::name());
}
template <class UnitTag, class Quantity>
inline std::string to_string_as(const Quantity& quantity) {
  std::ostringstream stream;
  stream << unit_value_for_text<UnitTag>(quantity) << ' ' << symbol<UnitTag>();
  return stream.str();
}
inline std::string to_string(const Number& number) {
  std::ostringstream stream;
  stream << number.value();
  return stream.str();
}
template <class Quantity>
inline
    typename std::enable_if<is_quantity<Quantity>::value && !std::is_same<Quantity, Number>::value, std::string>::type
    to_string(const Quantity& quantity) {
  return format_with_default_unit(quantity);
}
inline std::string to_string(const Temperature& quantity) { return format_with_default_unit(quantity); }
inline std::string to_string(const TemperatureDelta& quantity) { return format_with_default_unit(quantity); }
inline std::ostream& operator<<(std::ostream& stream, const Number& number) {
  stream << number.value();
  return stream;
}
template <class Quantity>
inline
    typename std::enable_if<is_quantity<Quantity>::value && !std::is_same<Quantity, Number>::value, std::ostream&>::type
    operator<<(std::ostream& stream, const Quantity& quantity) {
  stream << to_string(quantity);
  return stream;
}
inline std::ostream& operator<<(std::ostream& stream, const Temperature& quantity) {
  stream << to_string(quantity);
  return stream;
}
inline std::ostream& operator<<(std::ostream& stream, const TemperatureDelta& quantity) {
  stream << to_string(quantity);
  return stream;
}

// Compatibility literals not in the catalog because they don't match the name,
// the intent is that they're more obvious to think about volt per meter per
// second as opposed to volt second per meter... one obviously gives you volts
// when multiplied by m/s, the latter is less obvious

namespace literals {
constexpr LinearDerivativeGain operator""_VpMPs(long double value) {
  return LinearDerivativeGain::from<volt_second_per_meter_t>(static_cast<double>(value));
}
constexpr LinearDerivativeGain operator""_VpMPs(unsigned long long value) {
  return LinearDerivativeGain::from<volt_second_per_meter_t>(static_cast<double>(value));
}
constexpr LinearDerivativeGain operator""_VpInPs(long double value) {
  return LinearDerivativeGain::from<volt_second_per_inch_t>(static_cast<double>(value));
}
constexpr LinearDerivativeGain operator""_VpInPs(unsigned long long value) {
  return LinearDerivativeGain::from<volt_second_per_inch_t>(static_cast<double>(value));
}
constexpr LinearVelocityDerivativeGain operator""_VpMPs2(long double value) {
  return LinearVelocityDerivativeGain::from<volt_second_squared_per_meter_t>(static_cast<double>(value));
}
constexpr LinearVelocityDerivativeGain operator""_VpMPs2(unsigned long long value) {
  return LinearVelocityDerivativeGain::from<volt_second_squared_per_meter_t>(static_cast<double>(value));
}
constexpr LinearVelocityDerivativeGain operator""_VpInPs2(long double value) {
  return LinearVelocityDerivativeGain::from<volt_second_squared_per_inch_t>(static_cast<double>(value));
}
constexpr LinearVelocityDerivativeGain operator""_VpInPs2(unsigned long long value) {
  return LinearVelocityDerivativeGain::from<volt_second_squared_per_inch_t>(static_cast<double>(value));
}
}  // namespace literals

}  // namespace units

using namespace units;
using namespace units::literals;
