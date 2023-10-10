#pragma once
#include "vex.h"
#include <vector>
#include <functional>

namespace screen
{
    /**
     * @brief Start the screen background task. Once you start this, no need to draw to the screen manually elsewhere
     * @param screen reference to the vex screen
     * @param pages drawing pages
     * @param first_page optional, which page to start the program at. by default 0
     */
    void start_screen(vex::brain::lcd &screen, std::vector<Page *> pages, int first_page = 0);

    void stop_screen();
    /// @brief Page describes one part of the screen slideshow
    class Page
    {
    public:
        /**
         * @brief collect data, respond to screen input, do fast things (runs at
         * 50hz even if you're not focused on this Page (only drawn page gets
         * touch updates))
         * @param was_pressed true if the screen has been pressed
         * @param x x position of screen press (if the screen was pressed)
         * @param y y position of screen press (if the screen was pressed)
         */
        virtual void update(bool was_pressed, int x, int y);
        /**
         * @brief draw stored data to the screen (runs at 10 hz and only runs if
         * this page is in front)
         * @param first_draw true if we just switched to this page
         * @param frame_number frame of drawing we are on (basically an animation
         * tick)
         */
        virtual void draw(vex::brain::lcd &screen, bool first_draw,
                          unsigned int frame_number);
    };

    /// @brief  type of function needed for update
    using update_func_t = std::function<void(bool, int, int)>;

    /// @brief  type of function needed for draw
    using draw_func_t = std::function<void(vex::brain::lcd &screen, bool, unsigned int)>;

    class MotorPage : public Page
    {
    public:
        MotorPage(std::map<std::string, vex::motor&> motors) : motors(motors) {}
        void update(bool was_pressed, int x, int y) override;
        void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number) override;

    private:
        std::map<std::string, vex::motor&> motors;
    };

    /// @brief Simple page that stores no internal data. the draw and update functions use only global data rather than storing anything
    class FunctionPage : public Page
    {
    public:
        FunctionPage(update_func_t update_f, draw_func_t draw_t);

        void update(bool was_pressed, int x, int y) override;
        void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number) override;

    private:
        update_func_t update_f;
        draw_func_t draw_f;
    };

}
