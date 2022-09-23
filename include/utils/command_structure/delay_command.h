/**
 * File: delay_command.h
 * Desc:
 *    [insert meaningful desc]
 */

#pragma once

#include "../core/include/utils/command_structure/auto_command.h"

class DelayCommand: public AutoCommand {
  public:

    /**
     * Constructor
     */
    DelayCommand(int _ms) {
      ms = _ms;
    }
    
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