#include "../Core/include/subsystems/rgb_controller.h"

RGBController::RGBController(rgb_t starting_color, vex::triport::port port1, vex::triport::port port2, vex::triport::port port3)
  : port1(port1), port2(port2), port3(port3)
  {
    set(starting_color);
  }

  void RGBController::set(rgb_t color)
  {
    switch(color)
    {
      case Blue:
        port1.set(1);
        port2.set(0);
        port3.set(0);
      break;
      case Red:
        port1.set(0);
        port2.set(1);
        port3.set(0);
      break;
      case Green:
        port1.set(1);
        port2.set(1);
        port3.set(0);
      break;
      case OrangeWhite:
        port1.set(0);
        port2.set(0);
        port3.set(1);
      break;
      case Rainbow:
        port1.set(1);
        port2.set(0);
        port3.set(1);
      break;

      default:
      case Off:
        port1.set(0);
        port2.set(0);
        port3.set(0);
      break;
    }
  }