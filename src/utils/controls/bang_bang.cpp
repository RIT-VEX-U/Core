#include "core/utils/controls/bang_bang.h"

#include <cmath>

BangBang::BangBang(double threshhold, double low, double high)
    : setpt(low), sensor_val(low), lower_bound(low), upper_bound(high), threshhold(threshhold) {}

void BangBang::init(double start_pt, double set_pt) {
  sensor_val = start_pt;
  setpt = set_pt;
}

void BangBang::set_limits(double lower, double upper) {
  lower_bound = lower;
  upper_bound = upper;
}
double BangBang::get() { return last_output; }

double BangBang::update(double val) {
  sensor_val = val;
  if (fabs(val - setpt) < threshhold) {
    last_output = 0;
  } else if (val > setpt) {
    last_output = lower_bound;
  } else {
    last_output = upper_bound;
  }
  return upper_bound;
}

bool BangBang::is_on_target() { return fabs(sensor_val - setpt) < threshhold; }