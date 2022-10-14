/**
 * File: command_controller.h
 * Desc:
 *    Handles autonomous command execution
 *    Acts like a queue (because it is one)
 *    Execute and remove command at the head of the queue, 
 *    repeat until there are no more commmands in the queue
 */

#pragma once

#include <queue>
#include "../core/include/utils/command_structure/auto_command.h"

class CommandController {
  public:

  /**
   * Adds a command to the queue
   * @param cmd - AutoCommand to be added
   */
  void add(AutoCommand *cmd);

  /**
   * Adds a command that will delay progression
   * of the queue
   * @param ms - number of milliseconds to wait
   *    before continuing execution of autonomous
   */
  void add_delay(int ms);

  /**
   * Begin execution of the queue
   * Execute and remove command at the head of the queue, 
   * repeat until there are no more commmands in the queue
   */
  void run();

  private:
    std::queue<AutoCommand*> command_queue;
};