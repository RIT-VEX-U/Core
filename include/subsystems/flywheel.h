#include "vex.h"
#include "../core/include/robot_specs.h"
#include "../core/include/utils/pid.h"

using namespace vex;

class Flywheel{
  public:

  Flywheel(motor_group &motors, PID &pid);

  void spin_raw(double speed, directionType dir=fwd);
  void spinRPM(int rpm);
  void stop();
  void stopThread();

  double getRPM();
  PID* getPID();
  motor_group* getMotors();

  private:

  motor_group &motors;
  PID pid;
  double RPM = -1.0;
  task rpmTask;
};