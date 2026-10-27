#pragma once

// Base package
#include "core/robot_specs.h"

// Competition package
#include "core/competition/command_structure/auto_command.h"
#include "core/competition/command_structure/command_controller.h"
#include "core/competition/command_structure/delay_command.h"
#include "core/competition/command_structure/drive_commands.h"
#include "core/competition/initializer.h"

// Device package
#include "core/device/cobs_device.h"
#include "core/device/vdb/crc32.h"
#include "core/device/vdb/protocol.h"
#include "core/device/vdb/types.h"
#include "core/device/vdb_device.h"

// Math package
#include "core/math/controls/bang_bang.h"
#include "core/math/controls/feedback_base.h"
#include "core/math/controls/feedforward.h"
#include "core/math/controls/motion_controller.h"
#include "core/math/controls/pid.h"
#include "core/math/controls/pidff.h"
#include "core/math/controls/state_space/linear_plant_inversion_feedforward.h"
#include "core/math/controls/state_space/linear_quadratic_regulator.h"
#include "core/math/controls/trapezoid_profile.h"
#include "core/math/estimator/kalman_filter.h"
#include "core/math/estimator/unscented_kalman_filter.h"
#include "core/math/geometry/point2d.h"
#include "core/math/geometry/pose2d.h"
#include "core/math/geometry/rotation2d.h"
#include "core/math/geometry/transform2d.h"
#include "core/math/geometry/translation2d.h"
#include "core/math/geometry/twist2d.h"
#include "core/math/moving_average.h"
#include "core/math/numerical/numerical_integration.h"
#include "core/math/systems/dare_solver.h"
#include "core/math/systems/discretization.h"
#include "core/math/systems/linear_system.h"
#include "core/math/units.h"
#include "core/math/util.h"

// Screen package
#include "core/screen/legacy_bridge.h"
#include "core/screen/legacy.h"
#include "core/screen/pl_mpeg.h"
#include "core/screen/screen_controller.h"
#include "core/screen/video.h"

// Subsystems package
#include "core/subsystems/custom_encoder.h"
#include "core/subsystems/flywheel.h"
#include "core/subsystems/lift.h"
#include "core/subsystems/odometry/odometry_base.h"
#include "core/subsystems/odometry/odometry_nwheel.h"
#include "core/subsystems/odometry/odometry_serial.h"
#include "core/subsystems/odometry/odometry_tank.h"
#include "core/subsystems/tank_drive.h"

// Utils package
#include "core/utils/graph_drawer.h"
#include "core/utils/interpolating_map.h"
#include "core/utils/logger.h"
