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
};