#pragma once
#include "../core/include/subsystems/odometry/odometry_base.h"
#include "../core/include/utils/controls/pid.h"
#include "../core/include/utils/controls/pidff.h"
#include "../core/include/utils/graph_drawer.h"
#include "screen_pages.h"
#include "vex.h"
#include <cassert>
#include <functional>
#include <map>
#include <vector>

namespace screen {

/**
 * @brief Start the screen background task. Once you start this, no need to draw to the screen manually elsewhere
 * @param screen reference to the vex screen
 * @param pages drawing pages
 * @param first_page optional, which page to start the program at. by default 0
 */
void start_screen(vex::brain::lcd &screen, std::vector<screen_pages::Page *> pages, int first_page = 0);

/// @brief stops the screen. If you have a drive team that hates fun call this at the start of opcontrol
void stop_screen();

} // namespace screen