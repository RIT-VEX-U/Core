#pragma once

#include <Eigen/Core>
#include <cmath>
#include <iostream>

/**
 * This class implements dual numbers for automatic differentiation.
 * They represent a pair of doubles, a value and a derivative, with
 * overloaded operators for arithmetic functions, as well as a variety
 * of other mathematical functions.
 */
class Dual {
 public:
  Dual() : val(0.0), der(0.0) {}
  Dual(double value) : val(value), der(0.0) {}
  Dual(double value, double derivative) : val(value), der(derivative) {}

  double value() const { return val; }
  double derivative() const { return der; }

  void set_value(double value) { val = value; }
  void set_derivative(double derivative) { der = derivative; }

  Dual operator+(const Dual& other) const {
    return Dual(val + other.val, der + other.der);
  }

  Dual operator-(const Dual& other) const {
    return Dual(val - other.val, der - other.der);
  }

  Dual operator*(const Dual& other) const {
    return Dual(val * other.val, val * other.der + der * other.val);
  }

  Dual operator/(const Dual& other) const {
    double quotient = 1.0 / other.val;
    return Dual(val * quotient, 
                (der * other.val - val * other.der) * quotient * quotient);
  }

  Dual& operator+=(const Dual& other) {
    val += other.val;
    der += other.der;
    return *this;
  }

  Dual& operator-=(const Dual& other) {
    val -= other.val;
    der -= other.der;
    return *this;
  }

  Dual& operator*=(const Dual& other) {
    der = val * other.der + der * other.val;
    val *= other.val;
    return *this;
  }

  Dual& operator/=(const Dual& other) {
    double quotient = 1.0 / other.val;
    der = (der * other.val - val * other.der) * quotient * quotient;
    val *= quotient;
    return *this;
  }

  Dual operator-() const {
    return Dual(-val, -der);
  }

  bool operator==(const Dual& other) const {
    return val == other.val && der == other.der;
  }

  bool operator!=(const Dual& other) const {
    return val != other.val;
  }

  bool operator<(const Dual& other) const {
    return val < other.val;
  }

  bool operator<=(const Dual& other) const {
    return val <= other.val;
  }

  bool operator>(const Dual& other) const {
    return val > other.val;
  }

  bool operator>=(const Dual& other) const {
    return val >= other.val;
  }

  friend Dual sin(const Dual& x);
  friend Dual cos(const Dual& x);
  friend Dual tan(const Dual& x);
  friend Dual asin(const Dual& x);
  friend Dual acos(const Dual& x);
  friend Dual atan(const Dual& x);
  friend Dual exp(const Dual& x);
  friend Dual log(const Dual& x);
  friend Dual sqrt(const Dual& x);
  friend Dual pow(const Dual& x, double n);
  friend std::ostream& operator<<(std::ostream& os, const Dual& d);

 private:
  double val;
  double der;
};

inline Dual sin(const Dual& x) {
  return Dual(std::sin(x.val), x.der * std::cos(x.val));
}

inline Dual cos(const Dual& x) {
  return Dual(std::cos(x.val), -x.der * std::sin(x.val));
}

inline Dual tan(const Dual& x) {
  double tan_val = std::tan(x.val);
  return Dual(tan_val, x.der / (std::cos(x.val) * std::cos(x.val)));
}

inline Dual asin(const Dual& x) {
  return Dual(std::asin(x.val), x.der / std::sqrt(1 - x.val * x.val));
}

inline Dual acos(const Dual& x) {
  return Dual(std::acos(x.val), -x.der / std::sqrt(1 - x.val * x.val));
}

inline Dual atan(const Dual& x) {
  return Dual(std::atan(x.val), x.der / (1 + x.val * x.val));
}

inline Dual exp(const Dual& x) {
  double exp_val = std::exp(x.val);
  return Dual(exp_val, x.der * exp_val);
}

inline Dual log(const Dual& x) {
  return Dual(std::log(x.val), x.der / x.val);
}

inline Dual sqrt(const Dual& x) {
  double sqrt_val = std::sqrt(x.val);
  return Dual(sqrt_val, x.der / (2.0 * sqrt_val));
}

inline Dual pow(const Dual& x, double n) {
  double pow_val = std::pow(x.val, n);
  return Dual(pow_val, n * x.der * std::pow(x.val, n-1));
}

inline std::ostream& operator<<(std::ostream& os, const Dual& d) {
  os << d.val << " + " << d.der << "ε";
  return os;
}

// Allows Dual to be used in Eigen matrices and vectors.
namespace Eigen {
template<> struct NumTraits<Dual> : NumTraits<double> {
  typedef Dual Real;
  typedef Dual NonInteger;
  typedef Dual Nested;
  enum {
    IsComplex = 0,
    IsInteger = 0,
    IsSigned = 1,
    RequireInitialization = 1,
    ReadCost = 2,
    AddCost = 2,
    MulCost = 4
  };
};

} // end namespace Eigen
