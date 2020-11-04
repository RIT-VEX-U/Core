#ifndef _MECANUMDRIVE_
#define _MECANUMDRIVE_

#include "vex.h"
#include "../core/include/utils/pid.h"

#ifndef PI
#define PI 3.141592654
#endif

class MecanumDrive
{

  public:

  struct mecanumdrive_config_t
  {
    PID::pid_config_t drive_pid_conf;
    PID::pid_config_t drive_gyro_pid_conf;
    PID::pid_config_t turn_pid_conf;

    double drive_wheel_diam;
    double lateral_wheel_diam;

  };

  MecanumDrive(vex::motor &left_front, vex::motor &right_front, vex::motor &left_rear, vex::motor &right_rear, 
               vex::rotation *lateral_wheel=NULL, vex::inertial *imu=NULL, mecanumdrive_config_t *config=NULL);

  void drive_raw(double direction_deg, double magnitude, double rotation);

  void drive(double left_y, double left_x, double right_x, int power=2);

  bool auto_drive(double inches, double direction, double speed, bool gyro_correction=true);

  private:

  vex::motor &left_front, &right_front, &left_rear, &right_rear;

  mecanumdrive_config_t *config;
  vex::rotation *lateral_wheel;
  vex::inertial *imu;

  PID *drive_pid = NULL;
  PID *drive_gyro_pid = NULL;
  PID *turn_pid = NULL;

  bool init = true;

};

#endif