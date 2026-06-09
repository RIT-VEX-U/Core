#pragma once

class Number : public quantity_base<Number, dim_none> {
 public:
  typedef quantity_base<Number, dim_none> base_type;
  typedef dim_none dimension;
  constexpr explicit Number(double value = 0.0) : base_type(value) {}
  static constexpr Number from_canonical(double value) { return Number(value); }
  constexpr double value() const { return canonical_value(); }
};

template <>
struct is_quantity<Number> : std::true_type {};

Number quantity_for_dimension_probe(dim_none*, int);

template <class Scalar>
using number_arithmetic_enable_t = typename std::enable_if<std::is_arithmetic<Scalar>::value, Number>::type;

template <class Scalar>
constexpr number_arithmetic_enable_t<Scalar> operator+(const Number& left, Scalar right) {
  return Number(left.value() + static_cast<double>(right));
}
template <class Scalar>
constexpr number_arithmetic_enable_t<Scalar> operator+(Scalar left, const Number& right) {
  return Number(static_cast<double>(left) + right.value());
}
template <class Scalar>
constexpr number_arithmetic_enable_t<Scalar> operator-(const Number& left, Scalar right) {
  return Number(left.value() - static_cast<double>(right));
}
template <class Scalar>
constexpr number_arithmetic_enable_t<Scalar> operator-(Scalar left, const Number& right) {
  return Number(static_cast<double>(left) - right.value());
}

template <>
struct default_unit_for_quantity<Number> {
  typedef void type;
};
