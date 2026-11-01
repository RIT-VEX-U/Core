#include "core/screen/screen_book.h"

ScreenBook::ScreenBook(vex::brain::lcd screen, std::initializer_list<Page *> pages) : was_pressed(false), index(0),
 x_press(0), y_press(0), pages(pages), screen(screen) {}

ScreenBook::ScreenBook(vex::brain::lcd screen, std::vector<Page *> pages) : was_pressed(false), index(0),
 x_press(0), y_press(0), pages(pages), screen(screen) {}