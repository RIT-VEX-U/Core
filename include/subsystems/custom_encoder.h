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
  /**
   * Construct an encoder with a custom number of ticks
   * @param port the triport port on the brain the encoder is plugged into
   * @param ticks_per_rev the number of ticks the encoder will report for one revolution
  */
  CustomEncoder(vex::triport::port &port, double ticks_per_rev);

  /**
   * sets the stored rotation of the encoder. Any further movements will be from this value
   * @param val the numerical value of the angle we are setting to
   * @param units the unit of val
  */
  void setRotation(double val, vex::rotationUnits units);

  /**
   * sets the stored position of the encoder. Any further movements will be from this value
   * @param val the numerical value of the position we are setting to
   * @param units the unit of val
  */
  void setPosition(double val, vex::rotationUnits units);

  /**
   * get the rotation that the encoder is at
   * @param units the unit we want the return value to be in
   * @return the rotation of the encoder in the units specified
   */
  double rotation(vex::rotationUnits units);

  /**
   * get the position that the encoder is at
   * @param units the unit we want the return value to be in
   * @return the position of the encoder in the units specified
   */
  double position(vex::rotationUnits units);

  /**
   * get the velocity that the encoder is moving at
   * @param units the unit we want the return value to be in
   * @return the velocity of the encoder in the units specified
   */
  double velocity(vex::velocityUnits units);


  private:
  double tick_scalar;
};