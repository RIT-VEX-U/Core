/**
 * File: auto_command.h
 * Desc:
 *    Interface for module-specifc commands
 */

#pragma once

#include "vex.h"

class AutoCommand {
  public:
    /**
     * Executes the command
     * Overridden by child classes
     * @returns true when the command is finished, false otherwise
     */
    virtual bool run() { return true; }
    /**
     * What to do if we timeout instead of finishing. timeout is specified by the timeout seconds in the constructor
    */
    virtual void on_timeout(){}
    AutoCommand* withTimeout(double t_seconds){
<<<<<<< HEAD
      this->timeout_seconds = t_seconds;
=======
      timeout_seconds = t_seconds;
>>>>>>> refs/subrepo/core/fetch
      return this;
    }
    /** 
     * How long to run until we cancel this command. 
     * If the command is cancelled, on_timeout() is called to allow any cleanup from the function. 
     * If the timeout_seconds <= 0, no timeout will be applied and this command will run forever
     * A timeout can come in handy for some commands that can not reach the end due to some physical limitation such as
     * - a drive command hitting a wall and not being able to reach its target
     * - a command that waits until something is up to speed that never gets up to speed because of battery voltage
     * - something else...
    */
    double timeout_seconds = 10.0;

};