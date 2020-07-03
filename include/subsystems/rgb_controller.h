#ifndef _RGB_CON_
#define _RGB_CON_

#include "vex.h"

/**
 * A simple rgb controller for controlling the arduino, which in turn controls the led strip.
 * Sends binary over 3 seperate 3 wire ports for up to 7 color modes, and off.
 */
class RGBController
{
  private:
    vex::digital_out port1, port2, port3;

  public:

  enum rgb_t
  {
    Off, Blue, Red, Green, OrangeWhite, Rainbow
  };

  RGBController(rgb_t starting_color, vex::triport::port port1, vex::triport::port port2, vex::triport::port port3);

  void set(rgb_t color);

  
};
#endif