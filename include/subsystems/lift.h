#pragma once

#include "vex.h"
#include "../core/include/utils/pid.h"
#include <iostream>
#include <map>
#include <atomic>
#include <vector>

using namespace vex;
using namespace std;

/**
 * LIFT
 * A general class for lifts (e.g. 4bar, dr4bar, linear, etc)
 * Uses a PID to hold the lift at a certain height under load, and to move the lift to different heights
 *
 * @author Ryan McGee
 */
template <typename T>
class Lift
{
  public:

  /**
   * lift_cfg_t holds the physical parameter specifications of a lify system.
   * includes:
   * - maximum speeds for the system
   * - softstops to stop the lift from hitting the hard stops too hard 
   */
  struct lift_cfg_t
  {
    double up_speed, down_speed;
    double softstop_up, softstop_down;

    PID::pid_config_t lift_pid_cfg;
  };

  /**
    * Construct the Lift object and begin the background task that controls the lift.
    *
    * Usage example:
    * /code{.cpp}
    * enum Positions {UP, MID, DOWN};
    * map<Positions, double> setpt_map { 
    *      {DOWN, 0.0}, 
    *      {MID, 0.5},
    *      {UP, 1.0}
    *  };
    * Lift<Positions> my_lift(motors, lift_cfg, setpt_map);
    * /endcode
    *
    * @param lift_motors 
    *   A set of motors, all set that positive rotation correlates with the lift going up
    * @param lift_cfg
    *   Lift characterization information; PID tunings and movement speeds
    * @param setpoint_map
    *   A map of enum type T, in which each enum entry corresponds to a different lift height
    */
  Lift(motor_group &lift_motors, lift_cfg_t &lift_cfg, map<T, double> &setpoint_map, limit *homing_switch=NULL)
  : lift_motors(lift_motors), cfg(lift_cfg), lift_pid(cfg.lift_pid_cfg), setpoint_map(setpoint_map), homing_switch(homing_switch)
  {

    is_async = true;
    setpoint = 0;
    
    // Create a background task that is constantly updating the lift PID, if requested.
    // Set once, and forget.
    task t([](void* ptr){
      Lift &lift = *((Lift*) ptr);

      while(true)
      {
        if(lift.get_async())
          lift.hold();

        vexDelay(50);
      }

      return 0;
    }, this);

  }

  /**
    *  Control the lift with an "up" button and a "down" button.
    *  Use PID to hold the lift when letting go.
    *  @param up_ctrl
    *    Button controlling the "UP" motion
    *  @param down_ctrl
    *    Button controlling the "DOWN" motion
    */
  void control_continuous(bool up_ctrl, bool down_ctrl)
  {
    static timer tmr;
    
    double cur_pos = 0;

    // Check if there's a hook for a custom sensor. If not, use the motors.
    if(get_sensor == NULL)
      cur_pos = lift_motors.position(rev);
    else
      cur_pos = get_sensor();

    if(up_ctrl && cur_pos < cfg.softstop_up)
    {
      lift_motors.spin(directionType::fwd, cfg.up_speed, volt);
      setpoint = cur_pos + .3;

      // std::cout << "DEBUG OUT: UP " << setpoint << ", " << tmr.time(sec) << ", " << cfg.down_speed << "\n";

      // Disable the PID while going UP.
      is_async = false;
    } else if(down_ctrl && cur_pos > cfg.softstop_down)
    {
      // Lower the lift slowly, at a rate defined by down_speed
      if(setpoint > cfg.softstop_down)
        setpoint = setpoint - (tmr.time(sec) * cfg.down_speed);
      // std::cout << "DEBUG OUT: DOWN " << setpoint << ", " << tmr.time(sec) << ", " << cfg.down_speed << "\n";
      is_async = true;
    } else
    {
      // Hold the lift at the last setpoint
      is_async = true;
    }

    tmr.reset();  
  }

  /**
   * Control the lift with manual controls (no holding voltage)
   * 
   * @param up_btn Raise the lift when true
   * @param down_btn Lower the lift when true
   * @param volt_up Motor voltage when raising the lift
   * @param volt_down Motor voltage when lowering the lift
   */
  void control_manual(bool up_btn, bool down_btn, int volt_up, int volt_down)
  {
    static bool down_hold = false;
    static bool init = true;

    // Allow for setting position while still calling this function
    if(init || up_btn || down_btn)
    {
      init = false;
      is_async = false;
    }

    double rev = lift_motors.position(rotationUnits::rev);

    if(rev < cfg.softstop_down && down_btn)
      down_hold = true;
    else if( !down_btn )
      down_hold = false;

    if(up_btn && rev < cfg.softstop_up)
      lift_motors.spin(directionType::fwd, volt_up, voltageUnits::volt);
    else if(down_btn && rev > cfg.softstop_down && !down_hold)
      lift_motors.spin(directionType::rev, volt_down, voltageUnits::volt);
    else
      lift_motors.spin(directionType::fwd, 0, voltageUnits::volt);
  
  }

  /**
    * Control the lift in "steps". When the "up" button is pressed, the lift will go to the next
    * position as defined by pos_list. Order matters! 
    *
    * @param up_step
    *   A button that increments the position of the lift.
    * @param down_step
    *   A button that decrements the position of the lift.
    * @param pos_list
    *   A list of positions for the lift to go through. The higher the index, the higher the lift should be (generally).
    */
  void control_setpoints(bool up_step, bool down_step, vector<T> pos_list)
  {
    // Make sure inputs are only processed on the rising edge of the button
    static bool up_last = up_step, down_last = down_step;

    bool up_rising = up_step && !up_last;
    bool down_rising = down_step && !down_last;

    up_last = up_step;
    down_last = down_step;

    static int cur_index = 0;

    // Avoid an index overflow. Shouldn't happen unless the user changes pos_list between calls.
    if(cur_index >= pos_list.size())
      cur_index = pos_list.size() - 1;

    // Increment or decrement the index of the list, bringing it up or down.
    if(up_rising && cur_index < (pos_list.size() - 1))
      cur_index++;
    else if(down_rising && cur_index > 0)
      cur_index--;

    // Set the lift to hold the position in the background with the PID loop
    set_position(pos_list[cur_index]);
    is_async = true;  

  }

  /**
    * Enable the background task, and send the lift to a position, specified by the
    * setpoint map from the constructor.
    * 
    * @param pos
    *   A lift position enum type
    * @return True if the pid has reached the setpoint
    */
  bool set_position(T pos)
  {
    this->setpoint = setpoint_map[pos];
    is_async = true;

    return (lift_pid.get_target() == this->setpoint) && lift_pid.is_on_target();
  }

  /**
    * Manually set a setpoint value for the lift PID to go to.
    * @param val
    *   Lift setpoint, in motor revolutions or sensor units defined by get_sensor. Cannot be outside the softstops.
    * @return True if the pid has reached the setpoint
    */
  bool set_setpoint(double val)
  {
    this->setpoint = val;
    return (lift_pid.get_target() == this->setpoint) && lift_pid.is_on_target();
  }
  
  /**
    * @return The current setpoint for the lift
    */
  double get_setpoint()
  {
    return this->setpoint;
  }

  /**
    * Target the class's setpoint.
    * Calculate the PID output and set the lift motors accordingly.
    */
  void hold()
  {
    lift_pid.set_target(setpoint);
    // std::cout << "DEBUG OUT: SETPOINT " << setpoint << "\n";

    if(get_sensor != NULL)
      lift_pid.update(get_sensor());
    else
      lift_pid.update(lift_motors.position(rev));

    // std::cout << "DEBUG OUT: ROTATION " << lift_motors.rotation(rev) << "\n\n";

    lift_motors.spin(fwd, lift_pid.get(), volt);
  }

  /**
   * A blocking function that automatically homes the lift based on a sensor or hard stop, 
   * and sets the position to 0. A watchdog times out after 3 seconds, to avoid damage.
   */
  void home()
  {
    static timer tmr;
    tmr.reset();
    
    while(tmr.time(sec) < 3)
    {
      lift_motors.spin(directionType::rev, 6, volt);

      if (homing_switch == NULL && lift_motors.current(currentUnits::amp) > 1.5)
        break;
      else if (homing_switch != NULL && homing_switch->pressing())
        break;
    }

    if(reset_sensor != NULL)
      reset_sensor();
    
    lift_motors.resetPosition();
    lift_motors.stop();

  }

  /**
    * @return whether or not the background thread is running the lift
    */
  bool get_async()
  {
    return is_async;
  }

  /**
    * Enables or disables the background task. Note that running the control functions, or set_position functions
    * will immediately re-enable the task for autonomous use.
    * @param val Whether or not the background thread should run the lift
    */
  void set_async(bool val)
  {
    this->is_async = val;
  }

  /**
    * Creates a custom hook for any other type of sensor to be used on the lift. Example:
    * /code{.cpp}
    * my_lift.set_sensor_function( [](){return my_sensor.position();} );
    * /endcode
    *
    * @param fn_ptr
    *   Pointer to custom sensor function
    */
  void set_sensor_function(double (*fn_ptr) (void))
  {
    this->get_sensor = fn_ptr;
  }

  /**
   *  Creates a custom hook to reset the sensor used in set_sensor_function(). Example:
   * /code{.cpp}
   * my_lift.set_sensor_reset( my_sensor.resetPosition );
   * /endcode
   */
  void set_sensor_reset(void (*fn_ptr) (void))
  {
    this->reset_sensor = fn_ptr;
  }

  private:

  motor_group &lift_motors;
  lift_cfg_t &cfg;
  PID lift_pid;
  map<T, double> &setpoint_map;
  limit *homing_switch;
  
  atomic<double> setpoint;
  atomic<bool> is_async;

  double (*get_sensor)(void) = NULL;
  void (*reset_sensor)(void) = NULL;
  

};