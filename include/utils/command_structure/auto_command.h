/**
 * File: auto_command.h
 * Desc:
 *    Interface for module-specifc commands
 */

#pragma once

#include "../core/include/utils/formatting.h"
#include "vex.h"
#include <atomic>
#include <functional>
#include <queue>
#include <vector>

/**
 * A Condition is a function that returns true or false
 * is_even is a predicate that would return true if a number is even
 * For our purposes, a Condition is a choice to be made at runtime
 * drive_sys.reached_point(10, 30) is a predicate
 * time.has_elapsed(10, vex::seconds) is a predicate
 * extend this class for different choices you wish to make

 */
class Condition {
  public:
    Condition *Or(Condition *b);
    Condition *And(Condition *b);
    virtual bool test() = 0;
    virtual std::string toString();
};

class AutoCommand {
  public:
    static constexpr double default_timeout = 10.0;
    /**
     * Executes the command
     * Overridden by child classes
     * @returns true when the command is finished, false otherwise
     */
    virtual bool run() { return true; }

    virtual std::string toString() { return "AutoCommand"; }
    /**
     * What to do if we timeout instead of finishing. timeout is specified by the timeout seconds in the constructor
     */
    virtual void on_timeout() {}
    AutoCommand *withTimeout(double t_seconds) {
        if (this->timeout_seconds < 0) {
            // should never be timed out
            return this;
        }
        this->timeout_seconds = t_seconds;
        return this;
    }
    AutoCommand *withCancelCondition(Condition *true_to_end) {
        this->true_to_end = true_to_end;
        return this;
    }
    /**
     * How long to run until we cancel this command.
     * If the command is cancelled, on_timeout() is called to allow any cleanup from the function.
     * If the timeout_seconds <= 0, no timeout will be applied and this command will run forever
     * A timeout can come in handy for some commands that can not reach the end due to some physical limitation such as
     * - a drive command hitting a wall and not being able to reach its target
     * - a command that waits until something is up to speed that never gets up to speed because of battery voltage
     * - something else...
     */
    double timeout_seconds = default_timeout;
    Condition *true_to_end = nullptr;
};

/**
 * FunctionCommand is fun and good way to do simple things
 * Printing, launching nukes, and other quick and dirty one time things
 */
class FunctionCommand : public AutoCommand {
  public:
    FunctionCommand(std::function<bool(void)> f) : f(f) {}
    bool run() { return f(); }
    std::string toString() override { return "Function Command"; }

  private:
    std::function<bool(void)> f;
};

// Times tested 3
// Test 1 -> false
// Test 2 -> false
// Test 3 -> true
// Returns false until the Nth time that it is called
// This is pretty much only good for implementing RepeatUntil
class TimesTestedCondition : public Condition {
  public:
    TimesTestedCondition(size_t N) : max(N) {}
    bool test() override {
        count++;
        if (count >= max) {
            return true;
        }
        return false;
    }

  private:
    size_t count = 0;
    size_t max;
};

/// @brief FunctionCondition is a quick and dirty Condition to wrap some expression that should be evaluated at runtime
class FunctionCondition : public Condition {
  public:
    FunctionCondition(
      std::function<bool()> cond, std::function<void(void)> timeout = []() {}
    )
        : cond(cond), timeout(timeout) {}
    bool test() override;

  private:
    std::function<bool()> cond;
    std::function<void(void)> timeout;
};

/// @brief IfTimePassed tests based on time since the command controller was constructed. Returns true if elapsed time >
/// time_s
class IfTimePassed : public Condition {
  public:
    IfTimePassed(double time_s);
    bool test() override;

  private:
    double time_s;
    vex::timer tmr;
};

/// @brief Waits until the condition is true
class WaitUntilCondition : public AutoCommand {
  public:
    WaitUntilCondition(Condition *cond) : cond(cond) {}
    bool run() override { return cond->test(); }
    std::string toString() override { return "waiting until " + cond->toString(); }

  private:
    Condition *cond;
};

/// @brief InOrder runs its commands sequentially then continues.
/// How to handle timeout in this case. Automatically set it to sum of commands timouts?

/// @brief InOrder runs its commands sequentially then continues.
/// How to handle timeout in this case. Automatically set it to sum of commands timouts?
class InOrder : public AutoCommand {
  public:
    InOrder(const InOrder &other) = default;
    InOrder(std::queue<AutoCommand *> cmds);
    InOrder(std::initializer_list<AutoCommand *> cmds);
    bool run() override;
    void on_timeout() override;
    std::string toString() override;

  private:
    AutoCommand *current_command = nullptr;
    std::queue<AutoCommand *> cmds;
    vex::timer tmr;
};

/// @brief  Parallel runs multiple commands in parallel and waits for all to finish before continuing.
/// if none finish before this command's timeout, it will call on_timeout on all children continue
class Parallel : public AutoCommand {
  public:
    Parallel(std::initializer_list<AutoCommand *> cmds);
    bool run() override;
    void on_timeout() override;
    std::string toString() override;

  private:
    std::vector<AutoCommand *> cmds;
    std::vector<vex::task *> runners;
};

/// @brief Branch chooses from multiple options at runtime. the function decider returns an index into the choices
/// vector If you wish to make no choice and skip this section, return NO_CHOICE; any choice that is out of bounds set
/// to NO_CHOICE
class Branch : public AutoCommand {
  public:
    Branch(Condition *cond, AutoCommand *false_choice, AutoCommand *true_choice);
    ~Branch();
    bool run() override;
    std::string toString() override;
    void on_timeout() override;

  private:
    AutoCommand *false_choice;
    AutoCommand *true_choice;
    Condition *cond;
    bool choice = false;
    bool chosen = false;
    vex::timer tmr;
};

/// @brief Async runs a command asynchronously
/// will simply let it go and never look back
/// THIS HAS A VERY NICHE USE CASE. THINK ABOUT IF YOU REALLY NEED IT
class Async : public AutoCommand {
  public:
    Async(AutoCommand *cmd) : cmd(cmd) {}
    bool run() override;
    std::string toString() override;

  private:
    AutoCommand *cmd = nullptr;
};

class RepeatUntil : public AutoCommand {
  public:
    /// @brief RepeatUntil that runs a fixed number of times
    /// @param cmds the cmds to repeat
    /// @param repeats the number of repeats to do
    RepeatUntil(InOrder cmds, size_t repeats);
    /// @brief RepeatUntil the condition
    /// @param cmds the cmds to run
    /// @param true_to_end we will repeat until true_or_end.test() returns true
    RepeatUntil(InOrder cmds, Condition *true_to_end);
    bool run() override;
    std::string toString() override;
    void on_timeout() override;

  private:
    const InOrder cmds;
    InOrder *working_cmds;
    Condition *cond;
};
