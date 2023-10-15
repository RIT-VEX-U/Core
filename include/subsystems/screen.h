#pragma once
#include "vex.h"
#include <vector>
#include <functional>
#include <map>
#include "../core/include/subsystems/odometry/odometry_base.h"

namespace screen
{
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

    /**
     * @brief Start the screen background task. Once you start this, no need to draw to the screen manually elsewhere
     * @param screen reference to the vex screen
     * @param pages drawing pages
     * @param first_page optional, which page to start the program at. by default 0
     */
    void start_screen(vex::brain::lcd &screen, std::vector<Page *> pages, int first_page = 0);

    void stop_screen();

    /// @brief  type of function needed for update
    using update_func_t = std::function<void(bool, int, int)>;

    /// @brief  type of function needed for draw
    using draw_func_t = std::function<void(vex::brain::lcd &screen, bool, unsigned int)>;

    class StatsPage : public Page
    {
    public:
        StatsPage(std::map<std::string, vex::motor &> motors);
        void update(bool was_pressed, int x, int y) override;
        void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number) override;

    private:
        void draw_motor_stats(const std::string &name, vex::motor &mot, unsigned int frame, int x, int y, vex::brain::lcd &scr);

        std::map<std::string, vex::motor &> motors;
        static const int y_start = 0;
        static const int per_column = 4;
        static const int row_height = 20;
        static const int row_width = 200;
    };

    /**
     * @brief a page that shows odometry position and rotation and a map (if an sd card with the file is on)
    */
    class OdometryPage : public Page
    {
    public:
        OdometryPage(OdometryBase &odom, double width, double height, bool do_trail);
        void update(bool was_pressed, int x, int y) override;
        void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number) override;

    private:
        static const int path_len = 40;
        static constexpr char const *field_filename = "vex_field_240p.png";

        OdometryBase &odom;
        double width;
        double height;
        uint8_t *buf = nullptr;
        int buf_size = 0;
        pose_t path[path_len];
        int path_index = 0;
        bool do_trail;        
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
