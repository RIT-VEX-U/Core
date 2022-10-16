/**
 * File: command_controller.h
 * Desc:
 *    A CommandController manages the AutoCommands that make
 *    up an autonomous route. The AutoCommands are kept in
 *    a queue and get executed and removed from the queue
 *    in FIFO order.
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
   * Execute and remove commands in FIFO order
   */
  void run();

  private:
    std::queue<AutoCommand*> command_queue;
};