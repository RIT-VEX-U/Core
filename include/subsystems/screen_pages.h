#pragma once
#include "screen_widgets.h"
#include <map>
namespace screen_pages {
/**
 * @brief The type of function needed for update
 */
using update_func_t = std::function<void(bool, int, int)>;

/**
 * @brief The type of function needed for drawing
 */
using draw_func_t = std::function<void(vex::brain::lcd &screen, bool, unsigned int)>;

/**
 * @brief Page describes one part of the screen slideshow
 */
class Page {
public:
    /**
     * @brief Collect data, respond to screen input, do fast things (runs at
     * 50hz even if you're not focused on this Page (only the drawn page gets
     * touch updates))
     * @param was_pressed True if the screen has been pressed
     * @param x The x-position of screen press (if the screen was pressed)
     * @param y The y-position of screen press (if the screen was pressed)
     */
    virtual void update(bool was_pressed, int x, int y);
    /**
     * @brief Draw stored data to the screen (runs at 10 hz and only runs if
     * this page is in front)
     * @param first_draw True if we just switched to this page
     * @param frame_number The frame of drawing we are on (basically an
     * animation tick)
     */
    virtual void draw(vex::brain::lcd &screen, bool first_draw, unsigned int frame_number);
};

/**
 * @brief This isn't implemented anywhere else, so it does nothing for now
 */
class WidgetPage : public Page {
public:
    WidgetPage(screen_widgets::WidgetConfig &cfg) : base_widget(cfg) {}
    void update(bool was_pressed, int x, int y) override;

    void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number) override {
        draw_widget(base_widget, {.x1 = 20, .y1 = 0, .x2 = 440, .y2 = 240});
    }

private:
    screen_widgets::WidgetConfig &base_widget;
};

/**
 * @brief Draws motor stats and battery stats to the screen
 */
class StatsPage : public Page {
public:
    /**
     * @brief Creates a stats page
     * @param motors A map of string to motor that we want to draw on this page
     */
    StatsPage(std::map<std::string, vex::motor &> motors);
    /// @brief @see Page#update
    void update(bool was_pressed, int x, int y) override;
    /// @brief @see Page#draw
    void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number) override;

private:
    void draw_motor_stats(const std::string &name, vex::motor &mot, unsigned int frame, int x, int y,
                        vex::brain::lcd &scr);

    std::map<std::string, vex::motor &> motors;
    static const int y_start = 0;
    static const int per_column = 4;
    static const int row_height = 20;
    static const int row_width = 200;
};

/**
 * @brief A page that shows odometry position and rotation and a map (if an sd
 * card with the file is on)
 */
class OdometryPage : public Page {
public:
    /**
     * @brief Create an odometry trail. Make sure odometry is initilized before
     * now
     * @param odom The odometry system to monitor
     * @param robot_width The width (side to side) of the robot in inches. Used
     * for visualization
     * @param robot_height The robot_height (front to back) of the robot in
     * inches. Used for visualization
     * @param do_trail Whether or not to calculate and draw the trail. Drawing
     * and storing takes a very *slight* extra amount of processing power
     */
    OdometryPage(OdometryBase &odom, double robot_width, double robot_height, bool do_trail);
    /**
     * @brief @see Page#update
     */
    void update(bool was_pressed, int x, int y) override;
    /**
     * @brief @see Page#draw
     */
    void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number) override;

private:
    static const int path_len = 40;
    static constexpr char const *field_filename = "vex_field_240p.png";

    OdometryBase &odom;
    double robot_width;
    double robot_height;
    uint8_t *buf = nullptr;
    int buf_size = 0;
    pose_t path[path_len];
    int path_index = 0;
    bool do_trail;
    GraphDrawer velocity_graph;
};

/**
 * @brief Simple page that stores no internal data. The draw and update
 * functions use only global data rather than storing anything
 */
class FunctionPage : public Page {
public:
    /**
     * @brief Creates a function page
     * @param update_f The function called every tick to respond to user input
     * or do data collection
     * @param draw_t The function called to draw to the screen
     */
    FunctionPage(update_func_t update_f, draw_func_t draw_t);
    /**
     * @brief @see Page#update
     */
    void update(bool was_pressed, int x, int y) override;
    /**
     * @brief @see Page#draw
     */
    void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number) override;

private:
    update_func_t update_f;
    draw_func_t draw_f;
};

/**
 * @brief PIDPage provides a way to tune a pid controller on the screen
 */
class PIDPage : public Page {
public:
    /**
     * @brief Create a PIDPage
     * @param pid The pid controller we're changing
     * @param name A name to recognize this pid controller if we've got multiple
     * pid screens
     * @param onchange A function that is called when a tuning parameter is
     * changed. If you need to update stuff on that change register a handler
     * here
     */
    PIDPage(
        PID &pid, std::string name, std::function<void(void)> onchange = []() {});
    PIDPage(
        PIDFF &pidff, std::string name, std::function<void(void)> onchange = []() {});

    /**
     * @brief @see Page#update
     */
    void update(bool was_pressed, int x, int y) override;
    /**
     * @brief @see Page#draw
     */
    void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number) override;

private:
    /**
     * @brief reset d
     */
    void zero_d_f() { cfg.d = 0; }
    /**
     * @brief reset i
     */
    void zero_i_f() { cfg.i = 0; }

    PID::pid_config_t &cfg;
    PID &pid;
    const std::string name;
    std::function<void(void)> onchange;

    screen_widgets::SliderWidget p_slider;
    screen_widgets::SliderWidget i_slider;
    screen_widgets::SliderWidget d_slider;
    screen_widgets::ButtonWidget zero_i;
    screen_widgets::ButtonWidget zero_d;

    GraphDrawer graph;
};

/**
 * @brief Go to the next page
 */
void next_page();

/**
 * @brief Go to the previous page
 */
void prev_page();

/**
 * @brief Go to a specific page
 * @param page The page to go to
 */
void goto_page(size_t page);
}