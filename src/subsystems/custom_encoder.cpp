#include "../core/include/subsystems/custom_encoder.h"

CustomEncoder::CustomEncoder(vex::triport::port &port, double ticks_per_rev)
: super(port)
{
  tick_scalar = 90 / ticks_per_rev;
}

void CustomEncoder::setRotation(double val, vex::rotationUnits units)
{
  super::setRotation(val * tick_scalar, units);
}

void CustomEncoder::setPosition(double val, vex::rotationUnits units)
{
  super::setPosition(val * tick_scalar, units);
}

double CustomEncoder::rotation(vex::rotationUnits units)
{
  return super::rotation(units) / tick_scalar;
}

double CustomEncoder::position(vex::rotationUnits units)
{
  return super::position(units) / tick_scalar;
}

double CustomEncoder::velocity(vex::velocityUnits units)
{
  return super::velocity(units) / tick_scalar;
}
