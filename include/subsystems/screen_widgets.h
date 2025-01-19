#pragma once

namespace screen_widgets {
/**
 * @brief Widget that does something when you tap it. The function is only
 * called once when you first tap it
 */
class ButtonWidget {
public:
    /**
     * @brief Create a Button widget
     * @param onpress The function to be called when the button is tapped
     * @param rect The area the button should take up on the screen
     * @param name The label put on the button
     */
    ButtonWidget(std::function<void(void)> onpress, Rect rect, std::string name)
        : onpress(onpress), rect(rect), name(name) {}
    /**
     * @brief Create a Button widget
     * @param onpress The function to be called when the button is tapped
     * @param rect The area the button should take up on the screen
     * @param name The label put on the button
     */
    ButtonWidget(void (*onpress)(), Rect rect, std::string name) : onpress(onpress), rect(rect), name(name) {}

    /**
     * @brief Updates a button widget
     *
     * If some part of the button was recently pressed, the button updates
     * @param was_pressed If the screen was pressed or not
     * @param x The x-position of where the screen was pressed (if it was)
     * @param y The y-position of where the screen was pressed (if it was)
     * @return True if the button was pressed, false otherwise
     */
    bool update(bool was_pressed, int x, int y);
    /**
     * @brief Draws a button widget
     * @param scr The screen to draw the widget to
     */
    void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number);

private:
    std::function<void(void)> onpress;
    Rect rect;
    std::string name = "";
    bool was_pressed_last = false;
};

/**
 * @brief Widget that updates a double value. Updates by reference so watch out
 * for race conditions cuz the screen stuff lives on another thread
 */
class SliderWidget {
public:
    /**
     * @brief Creates a slider widget
     * @param val Reference to the value to modify
     * @param low Minimum value to go to
     * @param high Maximum value to go to
     * @param rect Rect to draw it
     * @param name Name of the value
     */
    SliderWidget(double &val, double low, double high, Rect rect, std::string name)
        : value(val), low(low), high(high), rect(rect), name(name) {}

    /**
     * @brief Updates a slider widget
     *
     * If some part of the slider was pressed, the selected point moves to where
     * it was pressed
     * @param was_pressed If the screen was pressed or not
     * @param x The x-position of where the screen was pressed (if it was)
     * @param y The y-position of where the screen was pressed (if it was)
     * @return True if the screen was pressed, false otherwise
     */
    bool update(bool was_pressed, int x, int y);
    /**
     * @brief Draws a slider widget
     *
     * If the height is greater than 0, it draws the slider 
     * @param scr The screen to draw the widget to
     */
    void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number);

private:
    double &value;

    double low;
    double high;

    Rect rect;
    std::string name = "";
};

struct WidgetConfig;

/**
 * @brief Stores the memory address of the current value, the minimum value, and
 * the maximum value value
 */
struct SliderConfig {
    double &val;
    double low;
    double high;
};

/**
 * @brief Stores the onclick function for a button widget
 */
struct ButtonConfig {
    std::function<void()> onclick;
};

/**
 * @brief Stores the onupdate function for a checkbox widget
 */
struct CheckboxConfig {
    std::function<void(bool)> onupdate;
};

/**
 * @brief Stores the text for a label widget
 */
struct LabelConfig {
    std::string label;
};

/**
 * @brief Stores a size and widget
 */
struct SizedWidget {
    int size;
    WidgetConfig &widget;
};

/**
 * @brief The generic config for a widget of any type
 */
struct WidgetConfig {
    enum Type {
        Col,
        Row,
        Slider,
        Button,
        Checkbox,
        Label,
        Graph,
    };
    Type type;
    union {
        std::vector<SizedWidget> widgets;
        SliderConfig slider;
        ButtonConfig button;
        CheckboxConfig checkbox;
        LabelConfig label;
        GraphDrawer *graph;
    } config;
};

/**
 * @brief Stores the four corners of a rectangle for where a widget will go
 */
struct ScreenRect {
    uint32_t x1;
    uint32_t y1;
    uint32_t x2;
    uint32_t y2;
};

/**
 * @brief Draws a specific widget depending on the type
 */
void draw_widget(WidgetConfig &widget, ScreenRect rect);
}