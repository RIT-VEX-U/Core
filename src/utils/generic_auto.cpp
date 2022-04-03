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
      vexDelay(20);

  } while(blocking && !state_list.empty());

  // If the method is blocking, return true because it's finished
  // If non-blocking, return false because the list isn't empty yet
  return blocking;
}

void GenericAuto::add(state_ptr new_state)
{
  state_list.push(new_state);
}

void GenericAuto::add_async(state_ptr async_state)
{
  state_ptr fn = [&async_state](){
    vex::task t([](void* fn_ptr){
      while(! (*(state_ptr*)fn_ptr)() )
       vexDelay(20);

      return 0;
    }, &async_state);
    return true;
  };

  state_list.push(fn);
}

void GenericAuto::add_delay(int ms)
{
  add([ms](){
    vexDelay(ms);
    return true;
  });
}