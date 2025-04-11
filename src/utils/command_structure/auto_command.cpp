#include "core/utils/command_structure/auto_command.h"

class OrCondition : public Condition {
public:
  OrCondition(Condition *A, Condition *B) : A(A), B(B) {}
  bool test() override {
    bool a = A->test();
    bool b = B->test();
    return a | b;
  }

private:
  Condition *A;
  Condition *B;
};

class AndCondition : public Condition {
public:
  AndCondition(Condition *A, Condition *B) : A(A), B(B) {}
  bool test() override {
    bool a = A->test();
    bool b = B->test();
    return a & b;
  }

private:
  Condition *A;
  Condition *B;
};
std::string Condition::toString() {return "Condition";}

Condition *Condition::Or(Condition *b) { return new OrCondition(this, b); }

Condition *Condition::And(Condition *b) { return new AndCondition(this, b); }

bool FunctionCondition::test() { return cond(); }
IfTimePassed::IfTimePassed(double time_s) : time_s(time_s), tmr() {}
bool IfTimePassed::test() { return tmr.value() > time_s; }

InOrder::InOrder(std::queue<AutoCommand *> cmds) : cmds(cmds) {
  timeout_seconds = -1.0; // never timeout unless with_timeout is explicitly called
}
InOrder::InOrder(std::initializer_list<AutoCommand *> cmds) : cmds(cmds) { timeout_seconds = -1.0; }

bool InOrder::run() {
  // outer loop finished
  if (cmds.size() == 0 && current_command == nullptr) {
    return true;
  }
  // retrieve and remove command at the front of the queue
  if (current_command == nullptr) {
    printf("TAKING INORDER: len =  %d\n", cmds.size());   
    current_command = cmds.front();
    cmds.pop();
    tmr.reset();
  }

  // run command
  bool cmd_finished = current_command->run();
  if (cmd_finished) {
    printf("InOrder Cmd finished\n");
    current_command = nullptr;
    return false; // continue onto next command
  }

  double seconds = tmr.value();

  bool should_timeout = current_command->timeout_seconds > 0.0;
  bool doTimeout = should_timeout && seconds > current_command->timeout_seconds;
  if (current_command->true_to_end != nullptr) {
    doTimeout = doTimeout || current_command->true_to_end->test();
  }

  // timeout
  if (doTimeout) {
    printf("InOrder timed out\n");
    current_command->on_timeout();
    current_command = nullptr;
    return false;
  }
  return false;
}

std::string InOrder::toString() {
  return "Running Inorder with length: " + int_to_string(cmds.size());
}

void InOrder::on_timeout() {
  if (current_command != nullptr) {
    current_command->on_timeout();
  }
}

struct parallel_runner_info {
  int index;
  std::vector<vex::task *> *runners;
  AutoCommand *cmd;
};
static int parallel_runner(void *arg) {
  parallel_runner_info *ri = (parallel_runner_info *)arg;
  vex::timer tmr;
  while (1) {
    bool finished = ri->cmd->run();
    if (finished) {
      break;
    }
    double t = (double)(tmr.time()) / 1000.0;
    bool timed_out = t > ri->cmd->timeout_seconds;
    bool doTimeout = timed_out && (ri->cmd->timeout_seconds > 0);
    if (ri->cmd->true_to_end != nullptr) {
      doTimeout = doTimeout || ri->cmd->true_to_end->test();
    }
    if (doTimeout) {
      ri->cmd->on_timeout();
    }
    vexDelay(20);
  }

  if ((*ri->runners)[ri->index] != nullptr) {
    delete (*ri->runners)[ri->index];
    (*ri->runners)[ri->index] = nullptr;
  }
  return 0;
}

// wait for all to finish
Parallel::Parallel(std::initializer_list<AutoCommand *> cmds) : cmds(cmds), runners(0) {}

bool Parallel::run() {
  if (runners.size() == 0) {
    // not initialized yet
    for (int i = 0; i < cmds.size(); i++) {
      parallel_runner_info *ri = new parallel_runner_info{
          .index = i,
          .runners = &runners,
          .cmd = cmds[i],
      };
      runners.push_back(new vex::task(parallel_runner, ri));
    }
  }

  bool all_finished = true;

  for (int i = 0; i < cmds.size(); i++) {
    if (runners[i] != nullptr) {
      all_finished = false;
    }
  }
  return all_finished;
}

std::string Parallel::toString() {
  return double_to_string(runners.size()) + " commands running in parallel";
}

void Parallel::on_timeout() {
  for (int i = 0; i < runners.size(); i++) {

    if (runners[i] != nullptr) {
      runners[i]->stop();
      if (cmds[i] != nullptr) {
        cmds[i]->on_timeout();
      }
      delete runners[i];
      runners[i] = nullptr;
      if (cmds[i] != nullptr) {
        delete cmds[i];
        cmds[i] = nullptr;
      }
    }
  }
}

Branch::Branch(Condition *cond, AutoCommand *false_choice, AutoCommand *true_choice)
    : false_choice(false_choice), true_choice(true_choice), cond(cond), choice(false), chosen(false), tmr() {
  this->timeout_seconds = -1;
}

Branch::~Branch() {
  delete false_choice;
  delete true_choice;
};
bool Branch::run() {
  if (!chosen) {
    choice = cond->test();
    chosen = true;
    tmr.reset();
  }

  double seconds = static_cast<double>(tmr.time()) / 1000.0;
  if (choice == false) {
    if (seconds > false_choice->timeout_seconds && false_choice->timeout_seconds != -1) {
      false_choice->on_timeout();
      chosen = false;
      return true;
    }
    bool finished = false_choice->run();
    if (finished) {
      chosen = false;
      return finished;
    }
  } else {
    if (seconds > true_choice->timeout_seconds && true_choice->timeout_seconds != -1) {
      true_choice->on_timeout();
      chosen = false;
      return true;
    }
    bool finished = true_choice->run();
    if (finished) {
      chosen = false;
      return finished;
    }
  }
  return false;
}

std::string Branch::toString(){
  return "Branch of " + false_choice->toString() + " and " + true_choice->toString() + " depending on " + cond->toString();
}
void Branch::on_timeout() {
  if (!chosen) {
    // dont need to do anything
    return;
  }

  if (choice == false) {
    false_choice->on_timeout();
  } else {
    true_choice->on_timeout();
  }
  chosen = false;
}

static int async_runner(void *arg) {
  AutoCommand *cmd = (AutoCommand *)arg;
  vex::timer tmr;
  while (1) {
    bool finished = cmd->run();
    if (finished) {
      break;
    }
    double t = (double)(tmr.time()) / 1000.0;
    bool timed_out = t > cmd->timeout_seconds;
    bool doTimeout = timed_out && cmd->timeout_seconds > 0;
    if (cmd->true_to_end != nullptr) {
      doTimeout = doTimeout || cmd->true_to_end->test();
    }
    if (doTimeout) {
      cmd->on_timeout();
      break;
    }
    vexDelay(20);
  }
  delete cmd;

  return 0;
}
bool Async::run() {
  vex::task *t = new vex::task(async_runner, (void *)cmd);
  (void)t;
  // lmao get memory leaked
  return true;
}

std::string Async::toString() {
  return "Async of " + cmd->toString();
}

RepeatUntil::RepeatUntil(InOrder cmds, size_t times) : RepeatUntil(cmds, new TimesTestedCondition(times)) {
  timeout_seconds = -1.0;
}

RepeatUntil::RepeatUntil(InOrder cmds, Condition *cond) : cmds(cmds), working_cmds(new InOrder(cmds)), cond(cond) {
  timeout_seconds = -1.0;
}

bool RepeatUntil::run() {
  bool finished = working_cmds->run();
  if (!finished) {
    // return if we're not done yet
    return false;
  }
  // this run finished

  bool res = cond->test();
  // we should finish
  if (res) {
    return true;
  }
  working_cmds = new InOrder(cmds);

  return false;
}

std::string RepeatUntil::toString(){
  InOrder pHCmds = cmds;
  return "Repeating " + pHCmds.toString() + " until " + true_to_end->toString();
}

void RepeatUntil::on_timeout() { working_cmds->on_timeout(); }