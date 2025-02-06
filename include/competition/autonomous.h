#pragma once
#include "core.h"
#include "robot-config.h"
#include "autopathing/auto-red-safe.h"
#include "autopathing/auto-blue-safe.h"
#include "autopathing/basic-skills.h"
/**
 * Main entrypoint for the autonomous period
*/
void autonomous();

void colorSort();

AutoCommand *intake_command(double amt = 10.0);
AutoCommand *outtake_command(double amt = 10.0);

AutoCommand *stop_intake();
AutoCommand *conveyor_intake_command(double amt = 10.0);
AutoCommand *conveyor_stop_command();

AutoCommand *goal_grabber_command(bool value);

AutoCommand *alliance_score_command(bool hold = true);
AutoCommand *stow_command();
AutoCommand *handoff_command();

AutoCommand *wallstake_command();