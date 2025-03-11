#pragma once
#include <cmath>
#include "../core/include/utils/math/geometry/transform2d.h"

/**
 *  Describes a Rectangle with a minimum and maximum point
 */

struct Rect {
  Translation2d min;
  Translation2d max;
  static Rect from_min_and_size(Translation2d min, Translation2d size) { return {min, min + size}; }
  Translation2d dimensions() const { return max - min; }
  Translation2d center() const { return (min + max) / 2; }
  double width() const { return max.x() - min.x(); }
  double height() const { return max.y() - min.y(); }
  bool contains(Translation2d p) const {
    bool xin = p.x() > min.x() && p.x() < max.x();
    bool yin = p.y() > min.y() && p.y() < max.y();
    return xin && yin;
  }
};

struct Mat2 {
  double X11, X12;
  double X21, X22;
  Translation2d operator*(const Translation2d p) const {
    double outx = p.x() * X11 + p.y() * X12;
    double outy = p.x() * X21 + p.y() * X22;
    return {outx, outy};
  }

  static Mat2 FromRotationDegrees(double degrees) {
    double rad = degrees * (M_PI / 180.0);
    double c = cos(rad);
    double s = sin(rad);
    return {c, -s, s, c};
  }
};