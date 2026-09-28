#pragma once

#include <cmath>

#include "core/utils/math/eigen_interface.h"

///  Describes a Rectangle with a minimum and maximum point
struct Rect {
    EVec<2> min;
    EVec<2> max;
    static Rect from_min_and_size(EVec<2> min, EVec<2> size) { return {min, min + size}; }
    EVec<2> dimensions() const { return max - min; }
    EVec<2> center() const { return (min + max) / 2; }
    double width() const { return max.x() - min.x(); }
    double height() const { return max.y() - min.y(); }
    bool contains(EVec<2> p) const {
        bool xin = p.x() > min.x() && p.x() < max.x();
        bool yin = p.y() > min.y() && p.y() < max.y();
        return xin && yin;
    }
};
