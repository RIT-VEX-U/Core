#include "core/subsystems/custom_encoder.h"

#include <vex_units.h>

#include "core/utils/units.h"

CustomEncoder::CustomEncoder(vex::triport::port& port, double ticks_per_rev) : super(port) {
  // bc it's a quadrature encoder, ticks per rev has to be multiplied by 4
  tick_scalar = 360 / (ticks_per_rev * 4);
}

void CustomEncoder::setRotation(units::Angle val) {
  super::setRotation(val.to(units::degrees) / tick_scalar, vex::rotationUnits::deg);
}

void CustomEncoder::setPosition(units::Angle val) {
  super::setRotation(val.to(units::degrees) / tick_scalar, vex::rotationUnits::deg);
}

units::Angle CustomEncoder::rotation() {
  return units::from(super::rotation(vex::rotationUnits::deg), units::degrees);
}

units::Angle CustomEncoder::position() {
  return units::from(super::position(vex::rotationUnits::deg), units::degrees);
}

units::AngularVelocity CustomEncoder::velocity() {
  return units::from(
      super::velocity(vex::velocityUnits::dps) * tick_scalar, units::degrees_per_second
  );
}
