#include "\core\subsystems\screen\legacy_bridge.h"

LegacyScreen::LegacyPage::LegacyPage( vex::brain::lcd screen, std::initializer_list<Page *> pages) : was_pressed(false), index(0),
 x_press(0), y_press(0), pages(pages), screen(screen) {}

LegacyScreen::LegacyPage::LegacyPage( vex::brain::lcd screen, std::vector<Page *> pages) : was_pressed(false), index(0),
 x_press(0), y_press(0), pages(pages), screen(screen) {}