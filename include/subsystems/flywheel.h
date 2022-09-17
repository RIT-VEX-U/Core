#include "vex.h"
#include "../core/include/robot_specs.h"
#include "../core/include/utils/pid.h"

using namespace vex;

class Flywheel{
  public:

  Flywheel(motor_group &motors, robot_specs_t &config, PID &pid);

  void spin(double speed, directionType dir=fwd);
  bool spinRPM(int rpm);
  void stop();
  void spinToDistance(double distance);

  private:

  motor_group &motors;
  robot_specs_t &config;
  PID pid;
  const int VOLTAGECONSTANT = 12;
  bool inRPMLoop = false;

};