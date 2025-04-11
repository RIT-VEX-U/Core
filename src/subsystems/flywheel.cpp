#include "core/subsystems/flywheel.h"
#include "core/subsystems/screen.h"
#include "core/utils/controls/feedforward.h"
#include "core/utils/controls/pid.h"
#include "core/utils/graph_drawer.h"
#include "core/utils/math_util.h"
#include "vex.h"

using namespace vex;

/*********************************************************
 *         CONSTRUCTOR, GETTERS, SETTERS
 *********************************************************/

Flywheel::Flywheel(motor_group &motors, Feedback &feedback, FeedForward &helper, const double ratio, Filter &filt)
    : motors(motors), task_running(false), fb(feedback), ff(helper), ratio(ratio), avger(filt) {}

/**
 * Return the current value that the target_rpm should be set to
 */
double Flywheel::get_target() const { return target_rpm; }

/**
 * @return the motors used to run the flywheel
 */
motor_group &Flywheel::get_motors() const { return motors; }

/**
 * return the current velocity of the flywheel motors, in RPM
 * @return the measured velocity of the flywheel
 */
double Flywheel::measure_RPM() {
  double rawRPM = ratio * motors.velocity(velocityUnits::rpm);
  avger.add_entry(rawRPM);
  return avger.get_value();
}

double Flywheel::getRPM() const { return avger.get_value(); }

/**
 * Runs a thread that keeps track of updating flywheel RPM and controlling it accordingly
 */
int spinRPMTask(void *wheelPointer) {
  Flywheel &wheel = *(Flywheel *)wheelPointer;

  // get the pid from the wheel and set its target to the RPM stored in the wheel.
  while (true) {
    double rpm = wheel.measure_RPM();

    if (wheel.target_rpm != 0) {
      double output = wheel.ff.calculate(wheel.target_rpm, 0.0, 0.0);
      {
        wheel.fb_mut.lock();
        wheel.fb.update(rpm); // check the current velocity and update the PID with it.

        output += wheel.fb.get();
        wheel.fb_mut.unlock();
      }

      wheel.spin_raw(output, fwd); // set the motors to whatever feedforward tells them to do
    }
    vexDelay(5);
  }
  return 0;
}

/*********************************************************
 *         SPINNERS AND STOPPERS
 *********************************************************/

/**
 * Spin motors using voltage; defaults forward at 12 volts
 * FOR USE BY TASKS ONLY
 * @param speed - speed (between -1 and 1) to set the motor
 * @param dir - direction that the motor moves in; defaults to forward
 */
void Flywheel::spin_raw(double speed, directionType dir) { motors.spin(dir, speed * 12, voltageUnits::volt); }

/**
 * Spin motors using voltage; defaults forward at 12 volts
 * FOR USE BY OPCONTROL AND AUTONOMOUS - this only applies if the RPM thread is not running
 * @param speed - speed (between -1 and 1) to set the motor
 * @param dir - direction that the motor moves in; defaults to forward
 */
void Flywheel::spin_manual(double speed, directionType dir) {
  if (!task_running) {
    motors.spin(dir, speed * 12, voltageUnits::volt);
  }
}

/**
 * starts or sets the RPM thread at new value
 * what control scheme is dependent on control_style
 * @param input_rpm - set the current RPM
 */
void Flywheel::spin_rpm(double input_rpm) {
  // setting to 0 is equivelent to stopping
  if (input_rpm == 0.0) {
    stop();
  }
  // only run if the RPM is different or it isn't already running
  if (!task_running) {
    rpm_task = task(spinRPMTask, this);
    task_running = true;
  }
  // now that its running, set the target
  set_target(input_rpm);
}
void Flywheel::set_target(double value) {
  fb_mut.lock();
  target_rpm = (value);
  fb.init(getRPM(), value);
  fb_mut.unlock();
}

/**
 * stop the RPM thread and the wheel
 */
void Flywheel::stop() {
  if (task_running) {
    task_running = false;
    rpm_task.stop();
    target_rpm = 0.0;
    motors.stop();
  }
}

//------------------------- Screen Stuff ----------------------------
class FlywheelPage : public screen::Page {
public:
  static const size_t window_size = 40;

  FlywheelPage(const Flywheel &fw)
      : fw(fw), gd(GraphDrawer(window_size, 0.0, 0.0,
                               {vex::color(255, 0, 0), vex::color(0, 255, 0), vex::color(0, 0, 255)}, 3)),
        avg_err(window_size) {}
  /// @brief @see Page#update
  void update(bool, int, int) override {}
  /// @brief @see Page#draw
  void draw(vex::brain::lcd &screen, bool, unsigned int) override {

    double target = fw.get_target();
    double actual = fw.getRPM();
    double err = fabs(target - actual);

    avg_err.add_entry(err);
    double volts = fw.fb.get() * 12.0;
    gd.add_samples(std::vector<double>{target, actual, volts / 12.0 * 1000.0});

    gd.draw(screen, 200, 10, 220, 220);
    screen.setPenColor(vex::white);
    screen.printAt(50, 30, "set: %.2f", target);
    screen.printAt(50, 60, "act: %.2f", actual);
    screen.printAt(50, 90, "stddev: %.2f", avg_err.get_value());
    screen.printAt(50, 150, "temp: %.2fc", fw.get_motors().temperature(vex::celsius));
    screen.printAt(50, 180, "volt: %.2fv", volts);
  }

private:
  const Flywheel &fw;
  GraphDrawer gd;
  MovingAverage avg_err;
};

screen::Page *Flywheel::Page() const { return new FlywheelPage(*this); }