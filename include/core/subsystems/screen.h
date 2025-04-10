#pragma once
#include "core/subsystems/odometry/odometry_base.h"
#include "core/utils/controls/pid.h"
#include "core/utils/controls/pidff.h"
#include "core/utils/graph_drawer.h"
#include "core/utils/math/geometry/pose2d.h"
#include "core/utils/math/geometry/translation2d.h"
#include "vex.h"
#include <cassert>
#include <functional>
#include <map>
#include <vector>

namespace screen {
/// @brief Widget that does something when you tap it. The function is only called once when you first tap it
class ButtonWidget {
  public:
    /// @brief Create a Button widget
    /// @param onpress the function to be called when the button is tapped
    /// @param rect the area the button should take up on the screen
    /// @param name the label put on the button
    ButtonWidget(std::function<void(void)> onpress, Rect rect, std::string name)
        : onpress(onpress), rect(rect), name(name) {}
    /// @brief Create a Button widget
    /// @param onpress the function to be called when the button is tapped
    /// @param rect the area the button should take up on the screen
    /// @param name the label put on the button
    ButtonWidget(void (*onpress)(), Rect rect, std::string name) : onpress(onpress), rect(rect), name(name) {}

    /// @brief responds to user input
    /// @param was_pressed if the screen is pressed
    /// @param x x position if the screen was pressed
    /// @param y y position if the screen was pressed
    /// @return true if the button was pressed
    bool update(bool was_pressed, int x, int y);
    /// @brief draws the button to the screen
    void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number);

  private:
    std::function<void(void)> onpress;
    Rect rect;
    std::string name = "";
    bool was_pressed_last = false;
};

/// @brief Widget that updates a double value. Updates by reference so watch out for race conditions cuz the screen
/// stuff lives on another thread
class SliderWidget {
  public:
    /// @brief Creates a slider widget
    /// @param val reference to the value to modify
    /// @param low minimum value to go to
    /// @param high maximum value to go to
    /// @param rect rect to draw it
    /// @param name name of the value
    SliderWidget(double &val, double low, double high, Rect rect, std::string name)
        : value(val), low(low), high(high), rect(rect), name(name) {}

    /// @brief responds to user input
    /// @param was_pressed if the screen is pressed
    /// @param x x position if the screen was pressed
    /// @param y y position if the screen was pressed
    /// @return true if the value updated
    bool update(bool was_pressed, int x, int y);
    /// @brief @ref Page::draws the slide to the screen
    void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number);

  private:
    double &value;

    double low;
    double high;

    Rect rect;
    std::string name = "";
};

struct WidgetConfig;

struct SliderConfig {
    double &val;
    double low;
    double high;
};
struct ButtonConfig {
    std::function<void()> onclick;
};
struct CheckboxConfig {
    std::function<void(bool)> onupdate;
};
struct LabelConfig {
    std::string label;
};

struct TextConfig {
    std::function<std::string()> text;
};
struct SizedWidget {
    int size;
    WidgetConfig &widget;
};
struct WidgetConfig {
    enum Type {
        Col,
        Row,
        Slider,
        Button,
        Checkbox,
        Label,
        Text,
        Graph,
    };
    Type type;
    union {
        std::vector<SizedWidget> widgets;
        SliderConfig slider;
        ButtonConfig button;
        CheckboxConfig checkbox;
        LabelConfig label;
        TextConfig text;
        GraphDrawer *graph;
    } config;
};

class Page;
/// @brief Page describes one part of the screen slideshow
class Page {
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
    virtual void draw(vex::brain::lcd &screen, bool first_draw, unsigned int frame_number);
};

struct ScreenRect {
    uint32_t x1;
    uint32_t y1;
    uint32_t x2;
    uint32_t y2;
};
void draw_widget(WidgetConfig &widget, ScreenRect rect);

class WidgetPage : public Page {
  public:
    WidgetPage(WidgetConfig &cfg) : base_widget(cfg) {}
    void update(bool was_pressed, int x, int y) override;

    void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number) override {
        draw_widget(base_widget, {.x1 = 20, .y1 = 0, .x2 = 440, .y2 = 240});
    }

  private:
    WidgetConfig &base_widget;
};

/**
 * @brief Start the screen background task. Once you start this, no need to draw to the screen manually elsewhere
 * @param screen reference to the vex screen
 * @param pages drawing pages
 * @param first_page optional, which page to start the program at. by default 0
 */
void start_screen(vex::brain::lcd &screen, std::vector<Page *> pages, int first_page = 0);

void next_page();
void prev_page();
void goto_page(size_t page);

/// @brief stops the screen. If you have a drive team that hates fun call this at the start of opcontrol
void stop_screen();

/// @brief  type of function needed for update
using update_func_t = std::function<void(bool, int, int)>;

/// @brief  type of function needed for draw
using draw_func_t = std::function<void(vex::brain::lcd &screen, bool, unsigned int)>;

/// @brief Draws motor stats and battery stats to the screen
class StatsPage : public Page {
  public:
    /// @brief Creates a stats page
    /// @param motors a map of string to motor that we want to draw on this page
    StatsPage(std::map<std::string, vex::motor &> motors);
    /// @brief @see Page#update
    void update(bool was_pressed, int x, int y) override;
    /// @brief @see Page#draw
    void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number) override;

  private:
    void
    draw_motor_stats(const std::string &name, vex::motor &mot, unsigned int frame, int x, int y, vex::brain::lcd &scr);

    std::map<std::string, vex::motor &> motors;
    static const int y_start = 0;
    static const int per_column = 4;
    static const int row_height = 20;
    static const int row_width = 200;
};

/**
 * @brief a page that shows odometry position and rotation and a map (if an sd card with the file is on)
 */
class OdometryPage : public Page {
  public:
    /// @brief Create an odometry trail. Make sure odometry is initilized before now
    /// @param odom the odometry system to monitor
    /// @param robot_width the width (side to side) of the robot in inches. Used for visualization
    /// @param robot_height the robot_height (front to back) of the robot in inches. Used for visualization
    /// @param do_trail whether or not to calculate and draw the trail. Drawing and storing takes a very *slight* extra
    /// amount of processing power
    OdometryPage(OdometryBase &odom, double robot_width, double robot_height, bool do_trail);
    /// @brief @see Page#update
    void update(bool was_pressed, int x, int y) override;
    /// @brief @see Page#draw
    void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number) override;

  private:
    static const int path_len = 40;
    static constexpr char const *field_filename = "vex_field_240p.png";

    OdometryBase &odom;
    double robot_width;
    double robot_height;
    uint8_t *buf = nullptr;
    int buf_size = 0;
    Pose2d path[path_len];
    int path_index = 0;
    bool do_trail;
    GraphDrawer velocity_graph;
};

/// @brief Simple page that stores no internal data. the draw and update functions use only global data rather than
/// storing anything
class FunctionPage : public Page {
  public:
    /// @brief Creates a function page
    /// @param update_f the function called every tick to respond to user input or do data collection
    /// @param draw_t the function called to draw to the screen
    FunctionPage(update_func_t update_f, draw_func_t draw_t);
    /// @brief @see Page#update
    void update(bool was_pressed, int x, int y) override;
    /// @brief @see Page#draw
    void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number) override;

  private:
    update_func_t update_f;
    draw_func_t draw_f;
};

/// @brief PIDPage provides a way to tune a pid controller on the screen
class PIDPage : public Page {
  public:
    /// @brief Create a PIDPage
    /// @param pid the pid controller we're changing
    /// @param name a name to recognize this pid controller if we've got multiple pid screens
    /// @param onchange a function that is called when a tuning parameter is changed. If you need to update stuff on
    /// that change register a handler here
    PIDPage(PID &pid, std::string name, std::function<void(void)> onchange = []() {});
    PIDPage(PIDFF &pidff, std::string name, std::function<void(void)> onchange = []() {});

    /// @brief @see Page#update
    void update(bool was_pressed, int x, int y) override;
    /// @brief @see Page#draw
    void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number) override;

  private:
    /// @brief reset d
    void zero_d_f() { cfg.d = 0; }
    /// @brief reset i
    void zero_i_f() { cfg.i = 0; }

    PID::pid_config_t &cfg;
    PID &pid;
    const std::string name;
    std::function<void(void)> onchange;

    SliderWidget p_slider;
    SliderWidget i_slider;
    SliderWidget d_slider;
    ButtonWidget zero_i;
    ButtonWidget zero_d;

    GraphDrawer graph;
};

} // namespace screen
