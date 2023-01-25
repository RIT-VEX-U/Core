/**
 * File: delay_command.h
 * Desc:
 *    A DelayCommand will make the robot wait the set amount of
 *    milliseconds before continuing execution of the autonomous route
 */

#pragma once

#include "../core/include/utils/command_structure/auto_command.h"

class DelayCommand: public AutoCommand {
  public:
    /**
     * Construct a delay command
     * @param ms the number of milliseconds to delay for
    */
    DelayCommand(int ms): ms(ms) {}
    
    /**
     * Delays for the amount of milliseconds stored in the command
     * Overrides run from AutoCommand
     * @returns true when complete
     */
    bool run() override {
      vexDelay(ms);
      return true;
    }

  private:
    // amount of milliseconds to wait
    int ms;
};