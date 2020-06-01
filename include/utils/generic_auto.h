#ifndef _GENAUTO_
#define _GENAUTO_

#include <vector>

typedef bool (*state_ptr)(void);

class GenericAuto
{
  public:
  
  /**
  * Create the GenericAuto object
  *
  * Example: GenericAuto auto1 = {func_ptr1, func_ptr2, ...};
  */
  GenericAuto(std::initializer_list<state_ptr> states);
  
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

  private:

  std::vector<state_ptr> state_list;

};

#endif