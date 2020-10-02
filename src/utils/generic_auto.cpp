#include "../core/include/utils/generic_auto.h"

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
bool GenericAuto::run(bool blocking)
{
  if(state_list.empty())
    return true;

  do
  {
    if( state_list.front()() )
      state_list.pop();

    if(blocking)
      vexDelay(50);

  } while(blocking && !state_list.empty());

  // If the method is blocking, return true because it's finished
  // If non-blocking, return false because the list isn't empty yet
  return blocking;
}

void GenericAuto::add(state_ptr newState)
{
  state_list.push(newState);
}

