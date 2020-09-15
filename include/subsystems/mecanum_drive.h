#ifndef _MECANUMDRIVE_
#define _MECANUMDRIVE_

#include "vex.h"

#ifndef PI
#define PI 3.141592654
#endif

class MecanumDrive
{

  public:

  MecanumDrive(vex::motor &left_front, vex::motor &right_front, vex::motor &left_rear, vex::motor &right_rear);

  void drive(double left_y, double left_x, double right_x, int power=2);

  private:


  vex::motor &left_front, &right_front, &left_rear, &right_rear;

};

#endif