#pragma once
#include "vex.h"

/**
 * A wrapper class for the vex encoder that allows the use of 3rd party
 * encoders with different tick-per-revolution values.
 */
class CustomEncoder : public vex::encoder
{
  typedef vex::encoder super;

  public:
  CustomEncoder(vex::triport::port &port, double ticks_per_rev);

  void setRotation(double val, vex::rotationUnits units);

  void setPosition(double val, vex::rotationUnits units);

  double rotation(vex::rotationUnits units);

  double position(vex::rotationUnits units);

  double velocity(vex::velocityUnits units);


  private:
  double tick_scalar;
};