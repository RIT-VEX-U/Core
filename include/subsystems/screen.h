#pragma once
#include "vex.h"
#include <vector>

/// @brief the type of function that the screen controller expects
/// @param screen a reference to the screen to draw on
/// @param x the x position this widget has been told to start at
/// @param y the y position this widget has been told to start at

typedef void (*screenFunc)(vex::brain::lcd &screen, int x, int y, int width, int height, bool first_run);

void draw_mot_header(vex::brain::lcd &screen, int x, int y, int width);
// name should be no longer than 15 characters
void draw_mot_stats(vex::brain::lcd &screen, int x, int y, int width, const char *name, vex::motor &motor, int animation_tick);
void draw_dev_stats(vex::brain::lcd &screen, int x, int y, int width, const char *name, vex::device &dev, int animation_tick);

void draw_battery_stats(vex::brain::lcd &screen, int x, int y, double voltage, double percentage);



void draw_lr_arrows(vex::brain::lcd &screen, int bar_width, int width, int height);

int handle_screen_thread(vex::brain::lcd &screen, std::vector<screenFunc> pages, int first_page);
void StartScreen(vex::brain::lcd &screen, std::vector<screenFunc> pages, int first_page = 0);