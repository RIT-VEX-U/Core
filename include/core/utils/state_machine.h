#pragma once
#include <string>
#include <type_traits>
#include <utility>

/**
 * @brief State Machine :))))))
 * A fun fun way of controlling stateful subsystems - used in the 2023-2024 Over
 * Under game for our overly complex intake-cata subsystem (see there for an
 * example)
 * The statemachine runs in a background thread and a user thread can interact
 * with it through current_state and send_message.
 *
 * Designwise:
 * the System class should hold onto any motors, feedback controllers, etc that
 * are persistent in the system States themselves should hold any data that
 * *only* that state needs. For example if a state should be exitted after a
 * certain amount of time, it should hold a timer rather than the System holding
 * that timer. (see Junder from 2024 for an example of this design)
 *
 * @tparam System The system that this is the base class of `class Thing :
 * public StateMachine<Thing>
 * @tparam IDType The ID enum that recognizes states. Hint hint, use an `enum
 * class`
 * @tparam Message the message enum that a state or an outside can send and that
 * states respond to
 * @tparam delay_ms the delay to wait between each state processing to allow
 * other threads to work
 * @tparam do_log true if you want print statements describing incoming messages
 * and current states. If true, it is expected that IDType and Message have a
 * function called to_string that takes them as its only parameter and returns a
 * std::string
 */
template <typename System, typename IDType, typename Message, int32_t delay_ms, bool do_log = false>
class StateMachine {
  static_assert(std::is_enum<Message>::value, "Message should be an enum (it's easier that way)");
  static_assert(std::is_enum<IDType>::value, "IDType should be an enum (it's easier that way)");

 public:
  /**
   * @brief MaybeMessage
   * a message of Message type or nothing
   * MaybeMessage m = {}; // empty
   * MaybeMessage m = Message::EnumField1
   */
  class MaybeMessage {
   public:
    /**
     * @brief Empty message - when theres no message
     */
    MaybeMessage() : exists(false) {}
    /**
     * @brief Create a maybemessage with a message
     * @param msg the message to hold on to
     */
    MaybeMessage(Message msg) : exists(true), thing(msg) {}
    /**
     * @brief check if the message is here
     * @return true if there is a message
     */
    bool has_message() { return exists; }
    /**
     * @brief Get the message stored. The return value is invalid unless
     * has_message returned true
     * @return The message if it exists. Undefined otherwise
     */
    Message message() { return thing; }

   private:
    bool exists;
    Message thing;
  };
  /**
   * Abstract class that all states for this machine must inherit from
   * States MUST override respond() and id() in order to function correctly
   * (the compiler won't have it any other way)
   */
  struct State {
    // run once when we enter the state
    virtual void entry(System&) {}
    // run continously while in the state
    virtual MaybeMessage work(System&) { return {}; }
    // run once when we exit the state
    virtual void exit(System&) {}
    // respond to a message when one comes in
    virtual State* respond(System& s, Message m) = 0;
    // Identify
    virtual IDType id() const = 0;

    // virtual destructor cuz c++
    virtual ~State() {}
  };

  // Data that gets passed to the runner thread. Don't worry too much about
  // this
  using thread_data = std::pair<State*, StateMachine*>;

  /**
   * @brief Construct a state machine and immediatly start running it
   * @param initial the state that the machine will begin in
   */
  StateMachine(State* initial) : runner(thread_runner, new thread_data{initial, this}) {}

  /**
   * @brief retrieve the current state of the state machine. This is safe to
   * call from external threads
   * @return the current state
   */
  IDType current_state() const {
    mut.lock();
    auto t = cur_type;
    mut.unlock();
    return t;
  }
  /**
   * @brief send a message to the state machine from outside
   * @param msg the message to send
   * This is safe to call from external threads
   */
  void send_message(Message msg) {
    mut.lock();
    incoming_msg = msg;
    mut.unlock();
  }

 private:
  vex::task runner;
  mutable vex::mutex mut;
  MaybeMessage incoming_msg;
  IDType cur_type;

  /**
   * @brief the thread that does the running of the state machine.
   * @param vptr the thread_data that we get passed to begin. cast it back
   * @return return value of thread (the thread never ends so this doesn't
   * really matter)
   */
  static int thread_runner(void* vptr) {
    thread_data* ptr = static_cast<thread_data*>(vptr);
    State* cur_state = ptr->first;

    StateMachine& sys = *ptr->second;
    System& derived = *static_cast<System*>(&sys);

    cur_state->entry(derived);

    sys.cur_type = cur_state->id();

    auto respond_to_message = [&](Message msg) {
      if (do_log) {
        printf("responding to msg: %s\n", to_string(msg).c_str());
        fflush(stdout);
      }

      State* next_state = cur_state->respond(derived, msg);

      if (cur_state != next_state) {
        // switched states
        sys.mut.lock();

        cur_state->exit(derived);
        next_state->entry(derived);

        delete cur_state;

        cur_state = next_state;
        sys.cur_type = cur_state->id();

        sys.mut.unlock();
      }
    };

    while (true) {
      if (do_log) {
        std::string str = to_string(cur_state->id());
        std::string str2 = to_string(sys.cur_type);

        printf("state: %s %s\n", str.c_str(), str2.c_str());
      }

      // Internal Message passed
      MaybeMessage internal_msg = cur_state->work(derived);

      if (internal_msg.has_message()) {
        respond_to_message(internal_msg.message());
      }

      // External Message passed
      sys.mut.lock();
      MaybeMessage incoming = sys.incoming_msg;
      sys.incoming_msg = {};
      sys.mut.unlock();

      if (incoming.has_message()) {
        respond_to_message(incoming.message());
      }

      vexDelay(delay_ms);
    }
    return 0;
  }
};
