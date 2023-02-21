/**
 * File: command_controller.cpp
 * Desc:
 *    A CommandController manages the AutoCommands that make
 *    up an autonomous route. The AutoCommands are kept in
 *    a queue and get executed and removed from the queue
 *    in FIFO order.
 */

#include "../core/include/utils/command_structure/command_controller.h"
#include "../core/include/utils/command_structure/delay_command.h"

/**
 * Adds a command to the queue
 * @param cmd the AutoCommand we want to add to our list
 * @param timeout_seconds the number of seconds we will let the command run for. If it exceeds this, we cancel it and run on_timeout
 */
void CommandController::add(AutoCommand *cmd, double timeout_seconds) {
  cmd->timeout_seconds = timeout_seconds;
  command_queue.push(cmd);
}

/**
 * Add multiple commands to the queue. No timeout here. 
 * @param cmds the AutoCommands we want to add to our list
 */
void CommandController::add(std::vector<AutoCommand *> cmds) {
  for(AutoCommand * cmd : cmds){
    command_queue.push(cmd);
  } 
}

/**
 * Adds a command that will delay progression
 * of the queue
 * @param ms - number of milliseconds to wait
 *    before continuing execution of autonomous
 */
void CommandController::add_delay(int ms) {
  AutoCommand *delay = new DelayCommand(ms);
  command_queue.push(delay);
}

/**
 * Begin execution of the queue
 * Execute and remove commands in FIFO order
 */
void CommandController::run() {
  AutoCommand *next_cmd;
  printf("Running Auto. Commands 1 to %d\n", command_queue.size());
  fflush(stdout);
  int command_count = 1;
  vex::timer tmr;
  tmr.reset();

  while(!command_queue.empty()) {
    // retrieve and remove command at the front of the queue
    next_cmd = command_queue.front();
    command_queue.pop();
    command_timed_out = false;

    printf("Beginning Command %d : timeout = %f\n", command_count, next_cmd->timeout_seconds);
    fflush(stdout);


    vex::timer timeout_timer;
    timeout_timer.reset();
    bool doTimeout = next_cmd->timeout_seconds > 0.0;

    // run the current command until it returns true or we timeout
    while(!next_cmd -> run()) {
      vexDelay(20);

      if (!doTimeout){
        continue;
      }

      // If we do want to check for timeout, check and end the command if we should
      double cmd_elapsed_sec = ((double)timeout_timer.time())/1000.0;
      if (cmd_elapsed_sec > next_cmd->timeout_seconds){
        next_cmd->on_timeout();
        command_timed_out = true;
        break;
      }
    }

    printf("Finished Command %d. Timed out: %s\n", command_count, command_timed_out ? "true" : "false" );
    fflush(stdout);
    command_count++;
  }
  printf("Finished commands in %f seconds\n", tmr.time(vex::sec));
}

bool CommandController::last_command_timed_out(){
  return command_timed_out;
}