#pragma once

typedef dim<0, 0, 0, 0, 0, 0, 0, 1> dim_angle;

class Angle;
template <>
struct dimension_for_quantity<Angle> {
  typedef dim_angle type;
};

struct radian_t {
  typedef dim_angle dimension;
  static constexpr double scale_num = static_cast<double>(1);
  static constexpr double scale_den = static_cast<double>(1);
  static constexpr bool is_affine = false;
  static const char* symbol() { return "rad"; }
  static const char* name() { return "radians"; }
};

struct degree_t {
  typedef dim_angle dimension;
  static constexpr double scale_num = static_cast<double>(17453292519943295.0);
  static constexpr double scale_den = static_cast<double>(1000000000000000000.0);
  static constexpr bool is_affine = false;
  static const char* symbol() { return "deg"; }
  static const char* name() { return "degrees"; }
};

struct revolution_t {
  typedef dim_angle dimension;
  static constexpr double scale_num = static_cast<double>(6283185307179586.0);
  static constexpr double scale_den = static_cast<double>(1000000000000000.0);
  static constexpr bool is_affine = false;
  static const char* symbol() { return "rev"; }
  static const char* name() { return "revolutions"; }
};

class Angle : public quantity_base<Angle, dim_angle> {
 public:
  typedef quantity_base<Angle, dim_angle> base_type;
  typedef dim_angle dimension;
  static constexpr double pi() { return 3.141592653589793238462643383279502884; }
  static constexpr double normalization_epsilon() { return 1e-12; }

 private:
  static constexpr double two_pi() { return 2.0 * pi(); }
  static constexpr double wrap_positive(double wrapped) { return wrapped < 0.0 ? wrapped + two_pi() : wrapped; }
  static constexpr double zero_near_endpoints(double normalized) {
    return gcem::abs(normalized - two_pi()) <= normalization_epsilon() ||
                   gcem::abs(normalized) <= normalization_epsilon()
               ? 0.0
               : normalized;
  }
  static constexpr double zero_near_origin(double normalized) {
    return gcem::abs(normalized) <= normalization_epsilon() ? 0.0 : normalized;
  }
  static constexpr double normalize_0_to_2pi_value(double radians) {
    return zero_near_endpoints(wrap_positive(gcem::fmod(radians, two_pi())));
  }
  static constexpr double normalize_minus_pi_to_pi_value(double radians) {
    return zero_near_origin(wrap_positive(gcem::fmod(radians + pi(), two_pi())) - pi());
  }

 public:
  constexpr explicit Angle(double radians = 0.0) : base_type(radians) {}
  template <class OtherQuantity>
  constexpr Angle(const OtherQuantity& other) : base_type(other.canonical_value()) {}
  static constexpr Angle from_canonical(double value) { return Angle(value); }
  template <class UnitTag>
  static constexpr Angle from(double value) {
    return Angle(base_type::template to_canonical<UnitTag>(value));
  }
  constexpr Angle normalized_0_to_2pi() const { return Angle(normalize_0_to_2pi_value(as<radian_t>())); }
  constexpr Angle normalized_minus_pi_to_pi() const { return Angle(normalize_minus_pi_to_pi_value(as<radian_t>())); }
  constexpr Angle normalized_0_to_360() const { return normalized_0_to_2pi(); }
  constexpr Angle normalized_minus_180_to_180() const { return normalized_minus_pi_to_pi(); }
};

template <>
struct is_quantity<Angle> : std::true_type {};

Angle quantity_for_dimension_probe(dim_angle*, int);

template <>
struct default_unit_for_quantity<Angle> {
  typedef radian_t type;
};

namespace literals {
constexpr Angle operator""_rad(long double value) { return Angle::from<radian_t>(static_cast<double>(value)); }
constexpr Angle operator""_rad(unsigned long long value) { return Angle::from<radian_t>(static_cast<double>(value)); }
constexpr Angle operator""_deg(long double value) { return Angle::from<degree_t>(static_cast<double>(value)); }
constexpr Angle operator""_deg(unsigned long long value) { return Angle::from<degree_t>(static_cast<double>(value)); }
constexpr Angle operator""_rev(long double value) { return Angle::from<revolution_t>(static_cast<double>(value)); }
constexpr Angle operator""_rev(unsigned long long value) {
  return Angle::from<revolution_t>(static_cast<double>(value));
}
}  // namespace literals

inline constexpr Angle shortest_angular_distance(const Angle& from, const Angle& to) {
  return (to - from).normalized_minus_pi_to_pi();
}
inline constexpr Length arc_length(const Length& radius, const Angle& angle) {
  return Length::from_canonical(radius.as<meter_t>() * angle.as<radian_t>());
}
inline constexpr Length turn_radius(const Curvature& curvature) { return Angle::from<radian_t>(1.0) / curvature; }
inline constexpr Curvature curvature_from_radius(const Length& radius) { return Angle::from<radian_t>(1.0) / radius; }
inline constexpr bool is_left_turn(const Curvature& curvature) { return curvature.as<radian_per_meter_t>() > 0.0; }
inline constexpr bool is_right_turn(const Curvature& curvature) { return curvature.as<radian_per_meter_t>() < 0.0; }
inline constexpr Number sin(const Angle& angle) { return Number(gcem::sin(angle.as<radian_t>())); }
inline constexpr Number cos(const Angle& angle) { return Number(gcem::cos(angle.as<radian_t>())); }
inline constexpr Number tan(const Angle& angle) { return Number(gcem::tan(angle.as<radian_t>())); }
inline constexpr Angle asin(const Number& value) { return Angle(gcem::asin(value.value())); }
inline constexpr Angle acos(const Number& value) { return Angle(gcem::acos(value.value())); }
inline constexpr Angle atan(const Number& value) { return Angle(gcem::atan(value.value())); }
inline constexpr Angle atan2(const Number& y, const Number& x) { return Angle(gcem::atan2(y.value(), x.value())); }
inline constexpr Angle atan2(const Length& y, const Length& x) {
  return Angle(gcem::atan2(y.as<meter_t>(), x.as<meter_t>()));
}
