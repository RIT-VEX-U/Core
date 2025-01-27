#include <vex.h>

class Intake {
public:
  enum class ConveyorState {
    Stop,
    In,
    Out,
  };
  enum IntakeState {
    Stop,
    In,
    InSort,
    Out,

  };

  enum class FilterColor {
    Blue,
    Red,
  };

  Intake(
    vex::motor &conveyor, vex::motor &intake_roller, vex::optical &conveyor_sensor,
    FilterColor filter_for
  )
      : conveyor(conveyor), intake_roller(intake_roller), conveyor_sensor(conveyor_sensor),
        color_we_want(filter_for) {

    bg_task = vex::task(thread_func, (void *)this);
  }

  void intake_in();
  void intake_and_sort();
  void intake_out();

  void intake_stop();

  void conveyor_start();
  void conveyor_out();
  void conveyor_stop();

  static int thread_func(void *self);

private:
  vex::motor &conveyor;
  vex::motor &intake_roller;
  vex::optical &conveyor_sensor;

  vex::task bg_task;

  ConveyorState conv_state = ConveyorState::Stop;
  IntakeState intake_state = IntakeState::Stop;

  vex::timer timer_since_last_seen;

  FilterColor color_we_want;
};