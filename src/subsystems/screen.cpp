#include "../core/include/subsystems/screen.h"

namespace screen
{
    /**
     * @brief The ScreenData class holds the data that will be passed to the
     * screen thread
     * you probably shouldnt have to use it
     */
    struct ScreenData
    {
        ScreenData(const std::vector<Page *> &m_pages, int m_page, vex::brain::lcd &m_screen)
            : pages(m_pages), page(m_page), screen(m_screen) {}
        std::vector<Page *> pages;
        int page = 0;
        vex::brain::lcd screen;
    };

    static vex::thread *screen_thread = nullptr;
    static bool running = false;
    static int screen_thread_func(void *screen_data_v);

    /// @brief start_screen begins a screen. only call this once per program (a
    /// good place is vexcodeInit)
    /// This is a set and forget type function. You don't have to wait on it or
    /// start it in a new thread
    /// @param screen the brain screen
    /// @param pages the list of pages in your UI slideshow
    /// @param first_page the page to start on (by default 0)
    void start_screen(vex::brain::lcd &screen, std::vector<Page *> pages,
                      int first_page)
    {
        if (running)
        {
            printf("THERE IS ALREADY A SCREEN THREAD RUNNING\n");
            return;
        }
        ScreenData *data = new ScreenData{pages, first_page, screen};

        screen_thread = new vex::thread(screen_thread_func, static_cast<void *>(data));
    }

    void stop_screen() { running = false; }

    /**
     * @brief runs the screen thread
     * This should only be called by start_screen
     * If you are calling this, maybe don't
     */
    int screen_thread_func(void *screen_data_v)
    {
        ScreenData screen_data = *static_cast<ScreenData *>(screen_data_v);
        running = true;
        unsigned int frame = 0;

        bool was_pressed = false;
        int x_press = 0;
        int y_press = 0;

        while (running)
        {
            Page *front_page = screen_data.pages[screen_data.page];
            bool pressing = screen_data.screen.pressing();

            if (pressing)
            {
                pressing = true;
                x_press = screen_data.screen.xPosition();
                y_press = screen_data.screen.yPosition();
            }
            bool just_pressed = pressing && !was_pressed;

            if (just_pressed && x_press < 40)
            {
                screen_data.page--;
                if (screen_data.page < 0)
                {
                    screen_data.page += screen_data.pages.size();
                }
            }
            if (just_pressed && x_press > 440)
            {
                screen_data.page++;
                screen_data.page %= screen_data.pages.size();
            }

            // Update all pages
            for (auto page : screen_data.pages)
            {
                if (page == front_page)
                {
                    page->update(was_pressed, x_press, y_press);
                }
                else
                {
                    page->update(false, 0, 0);
                }
            }

            // Draw First Page
            if (frame % 5 == 0)
            {
                screen_data.screen.clearScreen(vex::color::black);
                screen_data.screen.setPenColor("#FFFFFF");
                screen_data.screen.setFillColor("#000000");
                front_page->draw(screen_data.screen, false, frame / 5);

                // Draw side boxes
                screen_data.screen.setPenColor("#202020");
                screen_data.screen.setFillColor("#202020");
                screen_data.screen.drawRectangle(0, 0, 40, 240);
                screen_data.screen.drawRectangle(440, 0, 40, 240);
                screen_data.screen.setPenColor("#FFFFFF");
                // left arrow
                screen_data.screen.drawLine(30, 100, 15, 120);
                screen_data.screen.drawLine(30, 140, 15, 120);
                // right arrow
                screen_data.screen.drawLine(450, 100, 465, 120);
                screen_data.screen.drawLine(450, 140, 465, 120);
            }

            screen_data.screen.render();
            frame++;
            was_pressed = pressing;
            vexDelay(20);
        }

        return 0;
    }
    /**
     * @brief FunctionPage
     * @param update_f drawing function
     * @param draw_f drawing function
     */
    FunctionPage::FunctionPage(update_func_t update_f, draw_func_t draw_f)
        : update_f(update_f), draw_f(draw_f)
    {
    }

    /// @brief update uses the supplied update function to update this page
    void FunctionPage::update(bool was_pressed, int x, int y)
    {
        update_f(was_pressed, x, y);
    }
    /// @brief draw uses the supplied draw function to draw to the screen
    void FunctionPage::draw(vex::brain::lcd &screen, bool first_draw,
                            unsigned int frame_number)
    {
        draw_f(screen, first_draw, frame_number);
    }

    StatsPage::StatsPage(std::map<std::string, vex::motor &> motors) : motors(motors) {}
    void StatsPage::update(bool was_pressed, int x, int y)
    {
        (void)x;
        (void)y;
        (void)was_pressed;
    }
    void StatsPage::draw_motor_stats(const std::string &name, vex::motor &mot, unsigned int frame, int x, int y, vex::brain::lcd &scr)
    {
        const vex::color hot_col = vex::color(120, 0, 0);
        const vex::color med_col = vex::color(140, 100, 0);
        const vex::color ok_col = vex::black;
        const vex::color not_plugged_in_col = vex::color(255, 0, 0);
        bool installed = mot.installed();
        double temp = 0;
        int port = mot.index() + 1;
        vex::color col = ok_col;
        if (installed)
        {
            temp = mot.temperature(vex::temperatureUnits::celsius);
        }

        if (temp > 40)
        {
            col = med_col;
        }
        else if (temp > 50)
        {
            col = hot_col;
        }
        if (!installed && frame % 2 == 0)
        {
            col = not_plugged_in_col;
        }

        scr.drawRectangle(x, y, row_width, row_height, col);
        scr.printAt(x + 2, y + 16, false, " %2d   %2.0fC   %.7s", port, temp, name.c_str());
    }
    void StatsPage::draw(vex::brain::lcd &scr, bool first_draw, unsigned int frame_number)
    {
        int num = 0;
        int x = 40;
        int y = y_start + row_height;
        scr.drawRectangle(x, y_start, row_width, row_height);
        scr.printAt(x, y_start + 16, false, " port temp  name");
        for (auto &kv : motors)
        {
            if (num > per_column)
            {
                scr.drawRectangle(x + row_width, y_start, row_width, row_height);
                scr.printAt(x + row_width, y_start + 16, false, " port temp  name");

                y = y_start + row_height;
                x += row_width;
                num = 0;
            }

            draw_motor_stats(kv.first, kv.second, frame_number, x, y, scr);
            y += row_height;
            num++;
        }
        vex::brain b;
        scr.printAt(50, 220, "Battery: %2.1fv  %2.0fC %d%%", b.Battery.voltage(), b.Battery.temperature(vex::temperatureUnits::celsius), b.Battery.capacity());
    }

    OdometryPage::OdometryPage(OdometryBase &odom, double width, double height, bool do_trail) : odom(odom), width(width), height(height), do_trail(do_trail)
    {
        vex::brain b;
        if (b.SDcard.exists(field_filename))
        {
            buf_size = b.SDcard.size(field_filename);
            buf = (uint8_t *)malloc(buf_size);
            b.SDcard.loadfile(field_filename, buf, buf_size);
        }
        pose_t pos = odom.get_position();
        for (int i = 0; i < path_len; i++)
        {
            path[i] = pos;
        }
    }

    int in_to_px(double in)
    {
        double p = in / (6.0 * 24.0);
        return (int)(p * 240);
    }

    void OdometryPage::draw(vex::brain::lcd &scr, bool first_draw, unsigned int frame_number)
    {
        if (do_trail)
        {
            path_index++;
            path_index %= path_len;
            path[path_index] = odom.get_position();
        }

        auto to_px = [](const point_t p) -> point_t
        {
            return {(double)in_to_px(p.x) + 200, (double)in_to_px(-p.y) + 240};
        };

        auto draw_line = [to_px, &scr](const point_t from, const point_t to)
        {
            scr.drawLine((int)to_px(from).x, (int)to_px(from).y, (int)to_px(to).x, (int)to_px(to).y);
        };

        pose_t pose = path[path_index];
        point_t pos = pose.get_point();
        fflush(stdout);
        scr.printAt(45, 30, "(%.2f, %.2f)", pose.x, pose.y);
        scr.printAt(45, 50, "%.2f deg", pose.rot);

        if (buf == nullptr)
        {
            scr.printAt(180, 110, "Field Image Not Found");
            return;
        }

        scr.drawImageFromBuffer(buf, 200, 0, buf_size);

        point_t pos_px = to_px(pos);
        scr.drawCircle((int)pos_px.x, (int)pos_px.y, 3, vex::color::white);

        if (do_trail)
        {
            pose_t last_pos = path[(path_index + 1) % path_len];
            for (int i = path_index + 2; i < path_index + path_len; i++)
            {
                int j = i % path_len;
                pose_t pose = path[j];
                scr.setPenColor(vex::color(80, 80, 80));
                draw_line(pose.get_point(), last_pos.get_point());
                last_pos = pose;
            }
        }
        scr.setPenColor(vex::color::white);

        Mat2 mat = Mat2::FromRotationDegrees(pose.rot - 90);
        const point_t to_left = point_t{-width / 2.0, 0};
        const point_t to_front = point_t{0.0, height / 2.0};

        const point_t fl = pos + mat * (+to_left + to_front);
        const point_t fr = pos + mat * (-to_left + to_front);
        const point_t bl = pos + mat * (+to_left - to_front);
        const point_t br = pos + mat * (-to_left - to_front);
        const point_t front = pos + mat * (to_front * 2.0);

        draw_line(fl, fr);
        draw_line(fr, br);
        draw_line(br, bl);
        draw_line(bl, fl);

        draw_line(pos, front);
    }
    void OdometryPage::update(bool was_pressed, int x, int y)
    {
        (void)x;
        (void)y;
        (void)was_pressed;
    }
} // namespace screen

/*
void draw_battery_stats(vex::brain::lcd &screen, int x, int y, double
voltage, double percentage)
{
    double low_pct = 70.0;
    double med_pct = 85.0;

    vex::color low_col = vex::color(120, 0, 0);
    vex::color med_col = vex::color(140, 100, 0);
    vex::color high_col = vex::black;

    vex::color bg_col = vex::black;
    vex::color border_col = vex::white;

    if (percentage < low_pct)
    {
        bg_col = low_col;
    }
    else if (percentage < med_pct)
    {
        bg_col = med_col;
    }
    else
    {
        bg_col = high_col;
    }

    screen.setPenWidth(3);
    screen.setFillColor(bg_col);
    screen.setPenColor(border_col);
    screen.setFont(vex::mono15);

    screen.drawRectangle(x + 3, y, 200 - 3, 20);
    screen.printAt(x + 5, y + 15, "battery: %.1fv  %d%%", voltage,
(int)percentage);
}

void draw_mot_header(vex::brain::lcd &screen, int x, int y, int width)
{
    vex::color bg_col = vex::black;
    vex::color border_col = vex::white;
    screen.setPenWidth(3);
    screen.setFillColor(bg_col);
    screen.setPenColor(border_col);
    screen.setFont(vex::mono15);

    screen.drawRectangle(x + 3, y, width - 3, 20);
    screen.printAt(x + 5, y + 15, "motor name");
    int name_width = 110;

    screen.printAt(x + name_width + 7, y + 15, "temp");

    screen.drawLine(x + name_width, y, x + name_width, y + 20);

    screen.drawLine(x + name_width + 50, y, x + name_width + 50, y + 20);

    screen.printAt(x + name_width + 55, y + 15, "port");
}

void draw_mot_stats(vex::brain::lcd &screen, int x, int y, int width, const
char *name, vex::motor &motor, int animation_tick)
{
    double tempC = motor.temperature(vex::celsius);
    bool pluggedin = motor.installed();
    int port = motor.index() + 1;

    // used for flashing
    vex::color bg_col = vex::black;
    vex::color hot_col = vex::color(120, 0, 0);
    vex::color med_col = vex::color(140, 100, 0);
    vex::color low_col = vex::black; // color(0,100,0);
    vex::color not_plugged_in_col = vex::color(255, 0, 0);

    double lowtemp = 40;
    double hightemp = 50;
    if (tempC < lowtemp)
    {
        bg_col = low_col;
    }
    else if (tempC >= lowtemp && tempC < hightemp)
    {
        bg_col = med_col;
    }
    else if (tempC >= hightemp)
    {
        bg_col = hot_col;
    }

    if (!pluggedin)
    {
        if ((animation_tick / 8) % 2 == 0)
        {
            bg_col = not_plugged_in_col;
        }
    }

    vex::color border_col = vex::white;
    screen.setPenWidth(3);
    screen.setFillColor(bg_col);
    screen.setPenColor(border_col);
    screen.setFont(vex::mono15);

    screen.drawRectangle(x + 3, y, width - 3, 20);

    // name
    screen.printAt(x + 7, y + 15, name);
    int name_width = 110;

    // temp
    screen.drawLine(x + name_width, y, x + name_width, y + 20);
    screen.printAt(x + name_width + 10, y + 15, "%dC", (int)tempC);

    // PORT
    screen.drawLine(x + name_width + 50, y, x + name_width + 50, y + 20);
    char *warning = (char *)"!";
    if (pluggedin)
    {
        warning = (char *)"";
    }
    screen.printAt(x + name_width + 60, y + 15, "%d%s", port, warning);
}

//
void draw_dev_stats(vex::brain::lcd &screen, int x, int y, int width, const
char *name, vex::device &dev, int animation_tick)
{
    bool pluggedin = dev.installed();
    int port = dev.index() + 1;

    // used for flashing
    vex::color bg_col = vex::black;
    vex::color not_plugged_in_col = vex::color(255, 0, 0);

    if (!pluggedin)
    {
        if ((animation_tick / 8) % 2 == 0)
        {
            bg_col = not_plugged_in_col;
        }
    }

    vex::color border_col = vex::white;
    screen.setPenWidth(3);
    screen.setFillColor(bg_col);
    screen.setPenColor(border_col);
    screen.setFont(vex::mono15);

    screen.drawRectangle(x + 3, y, width - 3, 20);

    // name
    screen.printAt(x + 5, y + 15, name);
    int name_width = 110;

    // temp
    // screen.drawLine(x + name_width, y, x + name_width, y + 20);
    // screen.printAt(x + name_width + 10, y + 15, "%dC", (int)tempC);

    // PORT
    screen.drawLine(x + name_width + 50, y, x + name_width + 50, y + 20);
    char *warning = (char *)"!";
    if (pluggedin)
    {
        warning = (char *)"";
    }
    screen.printAt(x + name_width + 60, y + 15, "%d%s", port, warning);
}

void draw_lr_arrows(vex::brain::lcd &screen, int bar_width, int width, int
height)
{
    auto bar_col = vex::color(40, 40, 40);
    auto arrow_col = vex::color(255, 255, 255);
    // draw touch zones
    screen.setFillColor(bar_col);
    screen.setPenColor(bar_col);

    screen.drawRectangle(0, 0, bar_width, height);
    screen.drawRectangle(width - bar_width, 0, width, height);

    // draw arrows
    screen.setPenColor(arrow_col);
    screen.drawLine(bar_width / 3, height / 2, 2 * bar_width / 3, height / 2
- 20); screen.drawLine(bar_width / 3, height / 2, 2 * bar_width / 3, height
/ 2 + 20);

    screen.drawLine(width - bar_width / 3, height / 2, width - 2 * bar_width
/ 3, height / 2 - 20); screen.drawLine(width - bar_width / 3, height / 2,
width - 2 * bar_width / 3, height / 2 + 20);
}

int handle_screen_thread(vex::brain::lcd &screen, std::vector<screenFunc>
pages, int first_page)
{
    unsigned int num_pages = pages.size();
    unsigned int current_page = first_page;

    int width = 480;
    int height = 240;

    int bar_width = 40;

    bool was_pressing = false;
    bool first_draw = true;
    while (true)
    {
        pages[current_page](screen, bar_width, 0, width - bar_width * 2,
height, first_draw);

        first_draw = false;
        // handle presses
        if (screen.pressing() && !was_pressing)
        {

            int x = screen.xPosition();
            // int y = screen.yPosition();

            if (x < bar_width)
            {
                if (current_page == 0)
                {
                    current_page = num_pages;
                }
                current_page--;
                first_draw = true;
            }

            if (x > width - bar_width)
            {
                current_page++;
                current_page %= num_pages;
                first_draw = true;
            }
        }
        draw_lr_arrows(screen, bar_width, width, height);
        was_pressing = screen.pressing();
        screen.render();
        vexDelay(40);
    }

    return 0;
}

void StartScreen(vex::brain::lcd &screen, std::vector<screenFunc> pages, int
first_page)
{
    // hold onto arguments here so we don't lose them and can use then in
the lambda down there. capture semantics are not fun static
std::vector<screenFunc> my_pages = pages; static vex::brain::lcd my_screen =
screen; static int my_first_page = first_page; vex::task screenTask([]() {
return handle_screen_thread(my_screen, my_pages, my_first_page); });
}
*/
