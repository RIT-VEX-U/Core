#pragma once
<<<<<<< HEAD
<<<<<<< HEAD

#include "core/utils/units.h"
=======
=======

#include "core/utils/units.h"
>>>>>>> f673d53 (fixed up units usage)
#include "vex/vex.h"
>>>>>>> ac3c750 (Header and Documentation Repackage (#193))

/**
 * A wrapper class for the vex encoder that allows the use of 3rd party
 * encoders with different tick-per-revolution values.
 */
class CustomEncoder : public vex::encoder {
  typedef vex::encoder super;

 public:
  /**
   * Construct an encoder with a custom number of ticks
   * @param port the triport port on the brain the encoder is plugged into
   * @param ticks_per_rev the number of ticks the encoder will report for one revolution
   */
  CustomEncoder(vex::triport::port& port, double ticks_per_rev);

  /**
   * sets the stored rotation of the encoder. Any further movements will be from this value
   * @param val the numerical value of the angle we are setting to
   * @param units the unit of val
   */
<<<<<<< HEAD
  void setRotation(double val, units::angle_unit units );
=======
  void setRotation(units::Angle val);
>>>>>>> f673d53 (fixed up units usage)

  /**
   * sets the stored position of the encoder. Any further movements will be from this value
   * @param val the numerical value of the position we are setting to
   * @param units the unit of val
   */
<<<<<<< HEAD
  void setPosition(double val, units::angle_unit units);
=======
  void setPosition(units::Angle val);
>>>>>>> f673d53 (fixed up units usage)

  /**
   * get the rotation that the encoder is at
   * @param units the unit we want the return value to be in
   * @return the rotation of the encoder in the units specified
   */
<<<<<<< HEAD
  double rotation(units::angle_unit units);
=======
  units::Angle rotation();
>>>>>>> f673d53 (fixed up units usage)

  /**
   * get the position that the encoder is at
   * @param units the unit we want the return value to be in
   * @return the position of the encoder in the units specified
   */
<<<<<<< HEAD
  double position(units::angle_unit units);
=======
  units::Angle position();
>>>>>>> f673d53 (fixed up units usage)

  /**
   * get the velocity that the encoder is moving at
   * @param units the unit we want the return value to be in
   * @return the velocity of the encoder in the units specified
   */
<<<<<<< HEAD
  double velocity(units::velocity_unit units);
=======
  units::AngularVelocity velocity();
>>>>>>> f673d53 (fixed up units usage)

 private:
  double tick_scalar;
};
