/**
 * File: command_controller.h
 * Desc:
 *    A CommandController manages the AutoCommands that make
 *    up an autonomous route. The AutoCommands are kept in
 *    a queue and get executed and removed from the queue
 *    in FIFO order.
 */

#pragma once
#include <vector>
#include <queue>
#include "../core/include/utils/command_structure/auto_command.h"

class CommandController
{
public:
  /**
   * Adds a command to the queue
   * @param cmd the AutoCommand we want to add to our list
   * @param timeout_seconds the number of seconds we will let the command run for. If it exceeds this, we cancel it and run on_timeout. if it is <= 0 no time out will be applied
   */
  void add(AutoCommand *cmd, double timeout_seconds = 10.0);

  /**
   * Add multiple commands to the queue. No timeout here.
   * @param cmds the AutoCommands we want to add to our list
   */
  void add(std::vector<AutoCommand *> cmds);

  /**
   * Add multiple commands to the queue. No timeout here.
   * @param cmds the AutoCommands we want to add to our list
   * @param timeout_sec timeout in seconds to apply to all commands if they are still the default
   */
  void add(std::vector<AutoCommand *> cmds, double timeout_sec);
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
  /**
   * last_command_timed_out tells how the last command ended
   * Use this if you want to make decisions based on the end of the last command
   * @return true if the last command timed out. false if it finished regularly
   */
  bool last_command_timed_out();

private:
  std::queue<AutoCommand *> command_queue;
  bool command_timed_out = false;
};