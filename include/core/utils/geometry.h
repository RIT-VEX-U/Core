#pragma once
#include "core/utils/math/geometry/transform2d.h"
#include <cmath>

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