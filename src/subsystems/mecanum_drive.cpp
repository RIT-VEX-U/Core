#include "../core/include/subsystems/mecanum_drive.h"

MecanumDrive::MecanumDrive(vex::motor &left_front, vex::motor &right_front, vex::motor &left_rear, vex::motor &right_rear)
: left_front(left_front), right_front(right_front), left_rear(left_rear), right_rear(right_rear)
{

}

/**
 * Drive the robot with a mecanum-style / arcade drive. Inputs are in percent (-100.0 -> 100.0) straight from the controller.
 * Controls are mixed, so the robot can drive forward / strafe / rotate all at the same time. 
 *
 * @param left_y left joystick, Y axis (forward / backwards)
 * @param left_x left joystick, X axis (strafe left / right)
 * @param right_x right joystick, X axis (rotation left / right)
 * @param power=2 how much of a "curve" there should be on drive controls; better for low speed maneuvers.
 *                Leave blank for a default curve of 2 (higher means more fidelity)
 */
void MecanumDrive::drive(double left_y, double left_x, double right_x, int power)
{
  // LATERAL CONTROLS - convert cartesion to a vector
  double magnitude = sqrt(pow(left_y / 100.0, 2) + pow(left_x / 100.0, 2));
  magnitude = pow(magnitude, power);

  double direction = atan2(left_x / 100.0, left_y / 100.0);

  // ROTATIONAL CONTROLS - just the right x joystick
  // Ternary makes sure that if the "power" is even, the rotation keeps it's sign
  double rotation = right_x / 100.0;
  rotation = (power%2 == 0 ? rotation < 0 ? -1.0 : 1.0 : 1.0) * pow(rotation, power);


  // ALGORITHM - "rotate" the vector by 45 degrees and apply each corner to a wheel
  // .. Oh, and mix rotation too
  double lf = (magnitude * cos(direction - (PI / 4.0))) + rotation;
  double rf = (magnitude * cos(direction + (PI / 4.0))) - rotation;
  double lr = (magnitude * cos(direction + (PI / 4.0))) + rotation;
  double rr = (magnitude * cos(direction - (PI / 4.0))) - rotation;

  // Limit the output between -1.0 and +1.0
  lf = lf > 1.0 ? 1.0 : (lf < -1.0 ? -1.0 : lf);
  rf = rf > 1.0 ? 1.0 : (rf < -1.0 ? -1.0 : rf);
  lr = lr > 1.0 ? 1.0 : (lr < -1.0 ? -1.0 : lr);
  rr = rr > 1.0 ? 1.0 : (rr < -1.0 ? -1.0 : rr);

  // Finally, spin the motors
  left_front.spin(vex::directionType::fwd, lf * 100.0, vex::velocityUnits::pct);
  right_front.spin(vex::directionType::fwd, rf * 100.0, vex::velocityUnits::pct);
  left_rear.spin(vex::directionType::fwd, lr * 100.0, vex::velocityUnits::pct);
  right_rear.spin(vex::directionType::fwd, rr * 100.0, vex::velocityUnits::pct);
}