#include "core.h"
#include "vex.h"

#include "wallstake_mech.h"
#include "../core/include/utils/controls/pid.h"

WallStakeMech::WallStakeMech(
  const vex::motor_group &motors, const vex::pot &pot, const Rotation2d &tolerance, const Rotation2d &setpoint,
  const double &pot_offset, PID pid
)
    : motors(motors), pot(pot), tolerance(tolerance), setpoint(setpoint), pot_offset(pot_offset), wallstake_pid(pid) { handle = new vex::task(background_task, (void *)this); }

Rotation2d WallStakeMech::get_angle() { return (from_degrees(1.1 * pot.angle(vex::deg) - pot_offset)); }

void WallStakeMech::set_setpoint(const Rotation2d &new_setpoint) { setpoint = new_setpoint;  }

Rotation2d WallStakeMech::get_setpoint() { return setpoint; }

void WallStakeMech::set_state(const WallStakeState &state) { setpoint = from_degrees(state); }

bool WallStakeMech::is_at_setpoint() {
    Rotation2d current_angle = get_angle();
    return (current_angle.wrapped_degrees_180() <= (setpoint + tolerance).wrapped_degrees_180()) && (current_angle.wrapped_degrees_180() >= (setpoint - tolerance).wrapped_degrees_180());
}

bool WallStakeMech::is_at_angle(const double &angle) { 
    Rotation2d current_angle = get_angle();
    return (current_angle.wrapped_degrees_180() <= (from_degrees(angle) + tolerance).wrapped_degrees_180()) && (current_angle.wrapped_degrees_180() >= (from_degrees(angle) - tolerance).wrapped_degrees_180());
}

bool WallStakeMech::is_at_state(const WallStakeState &state) { return is_at_angle(state); }

AutoCommand *WallStakeMech::set_setpoint_command(const Rotation2d &new_setpoint) {
  return new FunctionCommand([&]() {
    set_setpoint(new_setpoint);
    return true;
  });
}

AutoCommand *WallStakeMech::set_state_command(const WallStakeState &new_state) {
  return new FunctionCommand([&]() {
    set_state(new_state);
    return true;
  });
}

void WallStakeMech::update() {
    double kg = 0.9;
    double ffout = kg * (get_angle() - from_degrees(2)).f_cos();

    // double kp = 0.05;
    // double pout = kp * (setpoint.degrees() - get_angle().degrees());
    wallstake_pid.set_target(get_setpoint().degrees());
    double pidout = wallstake_pid.update(get_angle().degrees());
    set_voltage(ffout + pidout);

    printf("%f\n", (get_angle()));
}

void WallStakeMech::set_voltage(const double &voltage) { motors.spin(vex::fwd, (voltage < 4) ? voltage : 4, vex::volt); }

/**
 * Function that runs in the background task. This function pointer is passed
 * to the vex::task constructor.
 *
 * @param ptr Pointer to OdometryBase object
 * @return Required integer return code. Unused.
 */
int WallStakeMech::background_task(void *ptr) {
  WallStakeMech &obj = *((WallStakeMech *)ptr);
  vexDelay(1000);
  while (!obj.end_task) {
    obj.update();
    vexDelay(5);
  }

  return 0;
}

/**
 * End the background task. Cannot be restarted.
 * If the user wants to end the thread but keep the data up to date,
 * they must run the update() function manually from then on.
 */
void WallStakeMech::end_async() { this->end_task = true; }




