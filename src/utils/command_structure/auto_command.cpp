#include "../core/include/utils/command_structure/auto_command.h"

bool FunctionCondition::test()
{
    return cond();
}
IfTimePassed::IfTimePassed(double time_s) : time_s(time_s), tmr() {}
bool IfTimePassed::test()
{
    return (static_cast<double>(tmr.time()) / 1000.0) > time_s;
}

InOrder::InOrder(std::queue<AutoCommand *> cmds) : cmds(cmds)
{
    timeout_seconds = -1.0; // never timeout unless with_timeout is explicitly called
}
InOrder::InOrder(std::initializer_list<AutoCommand *> cmds) : cmds(cmds)
{
    timeout_seconds = -1.0;
}

bool InOrder::run()
{
    // outer loop finished
    if (cmds.size() == 0)
    {
        return true;
    }
    // retrieve and remove command at the front of the queue
    if (current_command == nullptr)
    {
        current_command = cmds.front();
        cmds.pop();
        tmr.reset();
    }

    // run command
    bool cmd_finished = current_command->run();
    if (cmd_finished)
    {
        current_command = nullptr;
        return false; // continue omn next command
    }

    double seconds = static_cast<double>(tmr.time()) / 1000.0;

    bool should_timeout = current_command->timeout_seconds > 0.0;
    bool command_timed_out = seconds > current_command->timeout_seconds;

    // timeout
    if (should_timeout && command_timed_out)
    {
        current_command->on_timeout();
        current_command = nullptr;
        return false;
    }
    return false;
}

void InOrder::on_timeout()
{
    if (current_command != nullptr)
    {
        current_command->on_timeout();
    }
}

struct parallel_runner_info
{
    int index;
    std::vector<vex::task *> *runners;
    AutoCommand *cmd;
};
static int parallel_runner(void *arg)
{
    parallel_runner_info *ri = (parallel_runner_info *)arg;
    vex::timer tmr;
    while (1)
    {
        bool finished = ri->cmd->run();
        if (finished)
        {
            break;
        }
        double t = (double)(tmr.time()) / 1000.0;
        bool timed_out = t > ri->cmd->timeout_seconds;
        bool should_timeout = ri->cmd->timeout_seconds > 0;

        if (timed_out && should_timeout)
        {
            ri->cmd->on_timeout();
        }
        vexDelay(20);
    }

    if ((*ri->runners)[ri->index] != nullptr)
    {
        delete (*ri->runners)[ri->index];
        (*ri->runners)[ri->index] = nullptr;
    }
    return 0;
}

// wait for all to finish
Parallel::Parallel(std::initializer_list<AutoCommand *> cmds) : cmds(cmds), runners(0) {}

bool Parallel::run()
{
    if (runners.size() == 0)
    {
        // not initialized yet
        for (int i = 0; i < cmds.size(); i++)
        {
            parallel_runner_info *ri = new parallel_runner_info{
                .index = i,
                .runners = &runners,
                .cmd = cmds[i],
            };
            runners.push_back(new vex::task(parallel_runner, ri));
        }
    }

    bool all_finished = true;

    for (int i = 0; i < cmds.size(); i++)
    {
        if (runners[i] != nullptr)
        {
            all_finished = false;
        }
    }
    return all_finished;
}
void Parallel::on_timeout()
{
    for (int i = 0; i < runners.size(); i++)
    {

        if (runners[i] != nullptr)
        {
            runners[i]->stop();
            if (cmds[i] != nullptr)
            {
                cmds[i]->on_timeout();
            }
            delete runners[i];
            runners[i] = nullptr;
            if (cmds[i] != nullptr)
            {
                delete cmds[i];
                cmds[i] = nullptr;
            }
        }
    }
}

Branch::Branch(Condition *cond, AutoCommand *false_choice, AutoCommand *true_choice) : false_choice(false_choice), true_choice(true_choice), cond(cond), choice(false), chosen(false), tmr() {}

Branch::~Branch()
{
    delete false_choice;
    delete true_choice;
};
bool Branch::run()
{
    if (!chosen)
    {
        choice = cond->test();
        chosen = true;
    }

    double seconds = static_cast<double>(tmr.time()) / 1000.0;
    if (choice == false)
    {
        if (seconds > false_choice->timeout_seconds && false_choice->timeout_seconds != -1)
        {
            false_choice->on_timeout();
        }
        return false_choice->run();
    }
    else
    {
        if (seconds > true_choice->timeout_seconds && true_choice->timeout_seconds != -1)
        {
            true_choice->on_timeout();
        }
        return true_choice->run();
    }
}
void Branch::on_timeout()
{
    if (!chosen)
    {
        // dont need to do anything
        return;
    }

    if (choice == false)
    {
        false_choice->on_timeout();
    }
    else
    {
        true_choice->on_timeout();
    }
}

static int async_runner(void *arg)
{
    AutoCommand *cmd = (AutoCommand *)arg;
    vex::timer tmr;
    while (1)
    {
        bool finished = cmd->run();
        if (finished)
        {
            break;
        }
        double t = (double)(tmr.time()) / 1000.0;
        bool timed_out = t > cmd->timeout_seconds;
        bool should_timeout = cmd->timeout_seconds > 0;
        if (timed_out && should_timeout)
        {
            cmd->on_timeout();
            break;
        }
        vexDelay(20);
    }
    delete cmd;

    return 0;
}
bool Async::run()
{
    vex::task *t = new vex::task(async_runner, (void *)cmd);
    (void)t;
    // lmao get memory leaked
    return true;
}
