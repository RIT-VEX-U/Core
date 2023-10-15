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
  /// @brief Create an empty CommandController. Add Command with CommandController::add()
  [[deprecated("Use list constructor instead.")]] CommandController() : command_queue({}) {}

  /// @brief Create a CommandController with commands pre added. More can be added with CommandController::add()
  /// @param cmds
  CommandController(std::initializer_list<AutoCommand *> cmds) : command_queue(cmds) {}
  /**
   * Adds a command to the queue
   * @param cmd the AutoCommand we want to add to our list
   * @param timeout_seconds the number of seconds we will let the command run for. If it exceeds this, we cancel it and run on_timeout. if it is <= 0 no time out will be applied
   */
  [[deprecated("Use list constructor instead. If you need to make a decision before adding new commands, use Branch (https://github.com/RIT-VEX-U/Core/wiki/3-%7C-Utilites#commandcontroller)")]] void add(std::vector<AutoCommand *> cmds);
  void add(AutoCommand *cmd, double timeout_seconds = 10.0);

  /**
   * Add multiple commands to the queue. No timeout here.
   * @param cmds the AutoCommands we want to add to our list
   */

  /**
   * Add multiple commands to the queue. No timeout here.
   * @param cmds the AutoCommands we want to add to our list
   * @param timeout_sec timeout in seconds to apply to all commands if they are still the default
   */
  [[deprecated("Use list constructor instead. If you need to make a decision before adding new commands, use Branch (https://github.com/RIT-VEX-U/Core/wiki/3-%7C-Utilites#commandcontroller)")]] void
  add(std::vector<AutoCommand *> cmds, double timeout_sec);
  /**
   * Adds a command that will delay progression
   * of the queue
   * @param ms - number of milliseconds to wait
   *    before continuing execution of autonomous
   */
  void add_delay(int ms);

  /// @brief add_cancel_func specifies that when this func evaluates to true, to cancel the command controller
  /// @param true_if_cancel a function that returns true when we want to cancel the command controller
  void add_cancel_func(std::function<bool(void)> true_if_cancel);

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
  std::function<bool()> should_cancel = []()
  { return false; };
};