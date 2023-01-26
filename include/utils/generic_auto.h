#pragma once

#include <queue>
#include <map>
#include "vex.h"
#include <functional>

typedef std::function<bool(void)> state_ptr;

/**
 * GenericAuto provides a pleasant interface for organizing an auto path
 * steps of the path can be added with add() and when ready, calling run() will begin executing the path
*/
class GenericAuto
{
  public:

  /**
  * The method that runs the autonomous. If 'blocking' is true, then
  * this method will run through every state until it finished.
  *
  * If blocking is false, then assuming every state is also non-blocking,
  * the method will run through the current state in the list and return
  * immediately.
  *
  * @param blocking
  *     Whether or not to block the thread until all states have run
  * @returns
  *     true after all states have finished.
  */
  bool run(bool blocking);

  /**
   * Add a new state to the autonomous via function point of type "bool (ptr*)()"
   * @param new_state the function to run
   */
  void add(state_ptr new_state);

  /**
   * Add a new state to the autonomous via function point of type "bool (ptr*)()" that will run asynchronously
   * @param async_state the function to run
   */
  void add_async(state_ptr async_state);

  /**
   * add_delay adds a period where the auto system will simply wait for the specified time
   * @param ms how long to wait in milliseconds
  */
  void add_delay(int ms);

  private:

  std::queue<state_ptr> state_list;

};
