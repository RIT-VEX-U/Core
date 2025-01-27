#include "intake.h"

void Intake::intake_and_sort() { intake_state = IntakeState::InSort; }
void Intake::intake_in() { intake_state = IntakeState::In; }
void Intake::intake_out() { intake_state = IntakeState::Out; }
void Intake::intake_stop() { intake_state = IntakeState::Stop; }

void Intake::conveyor_start() { conv_state = ConveyorState::In; }
void Intake::conveyor_stop() { conv_state = ConveyorState::Stop; }
void Intake::conveyor_out() { conv_state = ConveyorState::Out; }

bool is_red_ring(vex::color &col) {
  double hue = col.hue();
  return hue < 60;
}

bool is_blue_ring(vex::color &col) {
  double hue = col.hue();
  return hue < 250 && hue > 180;
}

int Intake::thread_func(void *v_self) {
  Intake &self = *(Intake *)v_self;
  double conv_volts = 12.0;
  double intake_volts = 12.0;

  // TUNE THESE
  uint32_t timerange_start = 94; // ms
  uint32_t timerange_end = 300;  // ms

  self.conveyor_sensor.setLightPower(100.0);
  self.conveyor_sensor.setLight(vex::ledState::on);

  // ALSO TUNE THIS
  self.conveyor_sensor.objectDetectThreshold(180);

  bool was_ring = false;

  while (true) {
    vex::color col = self.conveyor_sensor.color();
    // bool is_near = self.conveyor_sensor.isNearObject();

    bool is_red = is_red_ring(col);
    bool is_blue = is_blue_ring(col);
    // bool is_ring = is_red || is_blue;
    bool is_ring = self.conveyor_sensor.isNearObject();

    FilterColor ring_color = is_red ? FilterColor::Red : FilterColor::Blue;

    // Conveyor
    switch (self.conv_state) {
    case ConveyorState::In:
      self.conveyor.spin(vex::fwd, conv_volts, vex::volt);
      break;
    case ConveyorState::Out:
      self.conveyor.spin(vex::reverse, conv_volts, vex::volt);
      break;
    case ConveyorState::Stop:
      self.conveyor.stop();
    default:
      break;
    }

    // printf("%.2f, %.2f, %.2f\n", col.hue(), col.saturation(), col.brightness());

    if (is_ring && !was_ring) {
        // printf(" ring\n");
      self.timer_since_last_seen.reset();
    } else {
        // printf("no ring\n");
    }

    switch (self.intake_state) {
    case IntakeState::In:
      self.intake_roller.spin(vex::fwd, intake_volts, vex::volt);
      break;
    case IntakeState::InSort:
      self.intake_roller.spin(vex::fwd, intake_volts, vex::volt);
      printf("time: %d\n", self.timer_since_last_seen.time());
      if (self.timer_since_last_seen.time() > timerange_start && self.timer_since_last_seen.time() < timerange_end && ring_color == self.color_we_want) {
        printf("Flippijg\n");
        self.conveyor.stop();
      } else {
        self.conveyor.spin(vex::fwd, intake_volts, vex::volt);
      }
      break;
    case IntakeState::Out:
      self.intake_roller.spin(vex::reverse, intake_volts, vex::volt);
      break;
    case IntakeState::Stop:
      self.intake_roller.stop();
      break;

    default:
      break;
    }
    was_ring = is_ring;
    vexDelay(10);
  }
  return 0;
}