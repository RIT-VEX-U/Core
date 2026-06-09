#pragma once

typedef dim<0, 0, 0, 0, 1, 0, 0, 0> dim_temperature;

struct kelvin_t {
  typedef dim_temperature dimension;
  static constexpr double scale_num = static_cast<double>(1);
  static constexpr double scale_den = static_cast<double>(1);
  static constexpr bool is_affine = false;
  static const char* symbol() { return "K"; }
  static const char* name() { return "kelvin"; }
};

struct celsius_t {
  typedef dim_temperature dimension;
  static constexpr double scale_num = static_cast<double>(1);
  static constexpr double scale_den = static_cast<double>(1);
  static constexpr bool is_affine = false;
  static const char* symbol() { return "degC"; }
  static const char* name() { return "degrees_celsius"; }
};

struct fahrenheit_t {
  typedef dim_temperature dimension;
  static constexpr double scale_num = static_cast<double>(5);
  static constexpr double scale_den = static_cast<double>(9);
  static constexpr bool is_affine = false;
  static const char* symbol() { return "degF"; }
  static const char* name() { return "degrees_fahrenheit"; }
};

class TemperatureDelta : public quantity_base<TemperatureDelta, dim_temperature> {
 public:
  typedef quantity_base<TemperatureDelta, dim_temperature> base_type;
  typedef dim_temperature dimension;
  constexpr explicit TemperatureDelta(double kelvin_delta = 0.0) : base_type(kelvin_delta) {}
  static constexpr TemperatureDelta from_canonical(double value) { return TemperatureDelta(value); }
  template <class UnitTag>
  static constexpr TemperatureDelta from(double value) {
    return TemperatureDelta(base_type::template to_canonical<UnitTag>(value));
  }
  static constexpr TemperatureDelta from_K(double value) { return TemperatureDelta(value); }
  static constexpr TemperatureDelta from_degC(double value) { return TemperatureDelta(value); }
  static constexpr TemperatureDelta from_degF(double value) { return TemperatureDelta(value * 5.0 / 9.0); }
  constexpr double K() const { return canonical_value(); }
  constexpr double kelvin() const { return canonical_value(); }
  constexpr double degC() const { return canonical_value(); }
  constexpr double celsius() const { return canonical_value(); }
  constexpr double degF() const { return canonical_value() * 9.0 / 5.0; }
  constexpr double fahrenheit() const { return canonical_value() * 9.0 / 5.0; }
};

template <>
struct is_quantity<TemperatureDelta> : std::true_type {};

class Temperature {
 public:
  constexpr explicit Temperature(double kelvin = 0.0) : kelvin_(kelvin) {}
  static constexpr Temperature from_kelvin(double value) { return Temperature(value); }
  static constexpr Temperature from_celsius(double value) { return Temperature(value + 273.15); }
  static constexpr Temperature from_fahrenheit(double value) { return Temperature((value + 459.67) * 5.0 / 9.0); }
  static constexpr Temperature from_canonical(double value) { return Temperature(value); }
  constexpr double canonical_value() const { return kelvin_; }
  constexpr double K() const { return kelvin_; }
  constexpr double kelvin() const { return kelvin_; }
  constexpr double degC() const { return kelvin_ - 273.15; }
  constexpr double celsius() const { return kelvin_ - 273.15; }
  constexpr double degF() const { return kelvin_ * 9.0 / 5.0 - 459.67; }
  constexpr double fahrenheit() const { return kelvin_ * 9.0 / 5.0 - 459.67; }

 private:
  double kelvin_;
};

inline constexpr TemperatureDelta operator-(const Temperature& left, const Temperature& right) {
  return TemperatureDelta::from_canonical(left.canonical_value() - right.canonical_value());
}
inline constexpr Temperature operator+(const Temperature& temperature, const TemperatureDelta& delta) {
  return Temperature::from_canonical(temperature.canonical_value() + delta.canonical_value());
}
inline constexpr Temperature operator+(const TemperatureDelta& delta, const Temperature& temperature) {
  return temperature + delta;
}
inline constexpr Temperature operator-(const Temperature& temperature, const TemperatureDelta& delta) {
  return Temperature::from_canonical(temperature.canonical_value() - delta.canonical_value());
}
inline constexpr bool operator==(const Temperature& left, const Temperature& right) {
  return left.canonical_value() == right.canonical_value();
}
inline constexpr bool operator!=(const Temperature& left, const Temperature& right) {
  return !(left == right);
}
inline constexpr bool operator<(const Temperature& left, const Temperature& right) {
  return left.canonical_value() < right.canonical_value();
}
inline constexpr bool operator<=(const Temperature& left, const Temperature& right) {
  return left.canonical_value() <= right.canonical_value();
}
inline constexpr bool operator>(const Temperature& left, const Temperature& right) {
  return left.canonical_value() > right.canonical_value();
}
inline constexpr bool operator>=(const Temperature& left, const Temperature& right) {
  return left.canonical_value() >= right.canonical_value();
}

TemperatureDelta quantity_for_dimension_probe(dim_temperature*, int);

template <>
struct default_unit_for_quantity<Temperature> {
  typedef kelvin_t type;
};
template <>
struct default_unit_for_quantity<TemperatureDelta> {
  typedef kelvin_t type;
};

template <class UnitTag>
struct text_temperature_value;
template <>
struct text_temperature_value<kelvin_t> {
  static double get(const Temperature& temperature) { return temperature.K(); }
  static double get(const TemperatureDelta& temperature) { return temperature.K(); }
};
template <>
struct text_temperature_value<celsius_t> {
  static double get(const Temperature& temperature) { return temperature.degC(); }
  static double get(const TemperatureDelta& temperature) { return temperature.degC(); }
};
template <>
struct text_temperature_value<fahrenheit_t> {
  static double get(const Temperature& temperature) { return temperature.degF(); }
  static double get(const TemperatureDelta& temperature) { return temperature.degF(); }
};
template <class UnitTag>
inline double unit_value_for_text(const Temperature& temperature) {
  return text_temperature_value<UnitTag>::get(temperature);
}
template <class UnitTag>
inline double unit_value_for_text(const TemperatureDelta& temperature) {
  return text_temperature_value<UnitTag>::get(temperature);
}

namespace literals {
constexpr Temperature operator""_K(long double value) {
  return Temperature::from_kelvin(static_cast<double>(value));
}
constexpr Temperature operator""_K(unsigned long long value) {
  return Temperature::from_kelvin(static_cast<double>(value));
}
constexpr Temperature operator""_degC(long double value) {
  return Temperature::from_celsius(static_cast<double>(value));
}
constexpr Temperature operator""_degC(unsigned long long value) {
  return Temperature::from_celsius(static_cast<double>(value));
}
constexpr Temperature operator""_degF(long double value) {
  return Temperature::from_fahrenheit(static_cast<double>(value));
}
constexpr Temperature operator""_degF(unsigned long long value) {
  return Temperature::from_fahrenheit(static_cast<double>(value));
}
constexpr TemperatureDelta operator""_K_delta(long double value) {
  return TemperatureDelta::from_K(static_cast<double>(value));
}
constexpr TemperatureDelta operator""_K_delta(unsigned long long value) {
  return TemperatureDelta::from_K(static_cast<double>(value));
}
constexpr TemperatureDelta operator""_degC_delta(long double value) {
  return TemperatureDelta::from_degC(static_cast<double>(value));
}
constexpr TemperatureDelta operator""_degC_delta(unsigned long long value) {
  return TemperatureDelta::from_degC(static_cast<double>(value));
}
constexpr TemperatureDelta operator""_degF_delta(long double value) {
  return TemperatureDelta::from_degF(static_cast<double>(value));
}
constexpr TemperatureDelta operator""_degF_delta(unsigned long long value) {
  return TemperatureDelta::from_degF(static_cast<double>(value));
}
}  // namespace literals
