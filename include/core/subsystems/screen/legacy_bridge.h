#pragma once

#include <initializer_list>
#include <vector>

#include "legacy.h"
#include "\core\subsystems\screen\screen_controller.h"

namespace screen {

class LegacyPage {
public:

    LegacyPage(vex::brain::lcd screen, std::initializer_list<Page *> pages);
    LegacyPage(vex::brain::lcd screen, std::vector<Page *> pages);
    ~LegacyPage();

    constexpr std::function<void()> handle() {
        return [this]() {
            Page *front_page = this->pages[this->index];
            std::optional<Translation2d> pressing;

            if((pressing = ScreenController::get_press_pos()).has_value()) {
                this->x_press = pressing->x();
                this->y_press = pressing->y();
            }
            bool just_pressed = pressing.has_value() && !this->was_pressed;

            if (just_pressed && this->x_press < 40) {
                this->index--;
                if (this->index < 0) {
                    this->index += this->pages.size();
                }
            }
            if (just_pressed && this->x_press > 440) {
                this->index++;
                this->index %= this->pages.size();
            }

            // Update all pages
            for (auto page : this->pages) {
                if (page == front_page) {
                    page->update(this->was_pressed, this->x_press, this->y_press);
                } else {
                    page->update(false, 0, 0);
                }
            }

            // Draw First Page
            int frame = ScreenController::get_frame_count();
            if (frame % 2 == 0) {
                this->screen.clearScreen(vex::color::black);
                this->screen.setPenColor("#FFFFFF");
                this->screen.setFillColor("#000000");
                front_page->draw(this->screen, false, frame / 5);

                // Draw side boxes
                this->screen.setPenColor("#202020");
                this->screen.setFillColor("#202020");
                this->screen.drawRectangle(0, 0, 40, 240);
                this->screen.drawRectangle(440, 0, 40, 240);
                this->screen.setPenColor("#FFFFFF");
                // left arrow
                this->screen.drawLine(30, 100, 15, 120);
                this->screen.drawLine(30, 140, 15, 120);
                // right arrow
                this->screen.drawLine(450, 100, 465, 120);
                this->screen.drawLine(450, 140, 465, 120);
            }

            this->was_pressed = pressing.has_value();
        };
    }

private:

    bool was_pressed;
    int index;
    int x_press;
    int y_press;

    std::vector<Page *> pages;
    vex::brain::lcd screen;
};

/// @brief Generates a pre-initialization function for an Initializer to use if selecting through the InitializerPage system
/// @param brain The VEX Brain containing the screen to display the InitializerPage objects on
/// @param initializer The initializer object for which this page provides a GUI of
/// @param o An optional callback to handle other matters during pre-initialization
/// @return A pre-initialization function that handles the screen
inline std::function<void()> pre_initialize(vex::brain& brain, Initializer& initializer, LegacyPage** page, std::function<void()> o = nullptr) {
    return [&, o]() {
        if(o) o();

        std::vector<Page*> pages; size_t initializations = 0;
        do {
            pages.push_back(new InitializerPage(initializer, initializations));
            initializations += 8;
        } while(initializations < initializer.initialization_count());

        *page = new LegacyPage(brain.Screen, pages);
        ScreenController::set((*page)->handle());
    };
}

} // namespace screen