#pragma once

// Subsystems package
#include "../core/include/subsystems/custom_encoder.h"
#include "../core/include/subsystems/flywheel.h"
#include "../core/include/subsystems/layout.h"
#include "../core/include/subsystems/lift.h"
#include "../core/include/subsystems/mecanum_drive.h"
#include "../core/include/subsystems/odometry/odometry_3wheel.h"
#include "../core/include/subsystems/odometry/odometry_base.h"
#include "../core/include/subsystems/odometry/odometry_nwheel.h"
#include "../core/include/subsystems/odometry/odometry_serial.h"
#include "../core/include/subsystems/odometry/odometry_tank.h"
#include "../core/include/subsystems/screen.h"
#include "../core/include/subsystems/tank_drive.h"

// Utils package
#include "../core/include/utils/command_structure/auto_command.h"
#include "../core/include/utils/command_structure/basic_command.h"
#include "../core/include/utils/command_structure/command_controller.h"
#include "../core/include/utils/command_structure/delay_command.h"
#include "../core/include/utils/command_structure/drive_commands.h"
#include "../core/include/utils/command_structure/flywheel_commands.h"

#include "../core/include/utils/controls/bang_bang.h"
#include "../core/include/utils/controls/feedback_base.h"
#include "../core/include/utils/controls/feedforward.h"
#include "../core/include/utils/controls/motion_controller.h"
#include "../core/include/utils/controls/asymmetric_motion_controller.h"
#include "../core/include/utils/controls/pid.h"
#include "../core/include/utils/controls/pidff.h"
#include "../core/include/utils/controls/take_back_half.h"
#include "../core/include/utils/controls/trapezoid_profile.h"
#include "../core/include/utils/controls/asymmetric_trapezoid_profile.h"

#include "../core/include/utils/auto_chooser.h"
#include "../core/include/utils/generic_auto.h"
#include "../core/include/utils/geometry.h"
#include "../core/include/utils/graph_drawer.h"
#include "../core/include/utils/logger.h"
#include "../core/include/utils/math_util.h"
#include "../core/include/utils/moving_average.h"
#include "../core/include/utils/pure_pursuit.h"
#include "../core/include/utils/serializer.h"
#include "../core/include/utils/state_machine.h"
#include "../core/include/utils/vector2d.h"

// Base package
#include "../core/include/robot_specs.h"
