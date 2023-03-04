#include "../core/include/subsystems/screen.h"

void draw_battery_stats(vex::brain::lcd &screen, int x, int y, double voltage, double percentage)
{
    double low_pct = 70.0;
    double med_pct = 85.0;

    vex::color low_col = vex::color(120, 0, 0);
    vex::color med_col = vex::color(140, 100, 0);
    vex::color high_col = vex::black;

    vex::color bg_col = vex::black;
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

    screen.setFillColor(bg_col);
    screen.setPenColor(vex::white);
    screen.setFont(vex::mono15);

    screen.drawRectangle(x, y, 200, 20);
    screen.printAt(x + 2, y + 15, "battery: %.1fv  %d%%", voltage, (int)percentage);
}

void draw_mot_header(vex::brain::lcd &screen, int x, int y, int width)
{
    vex::color bg_col = vex::black;
    vex::color border_col = vex::white;
    screen.setFillColor(bg_col);
    screen.setPenColor(border_col);
    screen.setFont(vex::mono15);

    screen.drawRectangle(x, y, width, 20);
    screen.printAt(x + 5, y + 15, "motor name");
    int name_width = 110;

    screen.printAt(x + name_width + 7, y + 15, "temp");

    screen.drawLine(x + name_width, y, x + name_width, y + 20);

    screen.drawLine(x + name_width + 50, y, x + name_width + 50, y + 20);

    screen.printAt(x + name_width + 55, y + 15, "port");
}

void draw_mot_stats(vex::brain::lcd &screen, int x, int y, int width, const char *name, vex::motor &motor)
{
    double tempC = motor.temperature(vex::celsius);
    bool pluggedin = motor.installed();
    int port = motor.index() + 1;

    // used for flashing
    static int count = 0;
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
        if ((count / 16) % 2 == 0)
        {
            bg_col = not_plugged_in_col;
        }
        else
        {
            bg_col = bg_col;
        }
    }

    vex::color border_col = vex::white;
    screen.setFillColor(bg_col);
    screen.setPenColor(border_col);
    screen.setFont(vex::mono15);

    screen.drawRectangle(x, y, width, 20);

    // name
    screen.printAt(x + 5, y + 15, name);
    int name_width = 110;

    // temp
    screen.drawLine(x + name_width, y, x + name_width, y + 20);
    screen.printAt(x + name_width + 10, y + 15, "%dC", (int)tempC);

    // PORT
    screen.drawLine(x + name_width + 50, y, x + name_width + 50, y + 20);
    char *warning = "!";
    if (pluggedin)
    {
        warning = "";
    }
    screen.printAt(x + name_width + 60, y + 15, "%d%s", port, warning);
    count++;
}

void draw_lr_arrows(vex::brain::lcd &screen, int bar_width, int width, int height)
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
    screen.drawLine(bar_width / 3, height / 2, 2 * bar_width / 3, height / 2 - 20);
    screen.drawLine(bar_width / 3, height / 2, 2 * bar_width / 3, height / 2 + 20);

    screen.drawLine(width - bar_width / 3, height / 2, width - 2 * bar_width / 3, height / 2 - 20);
    screen.drawLine(width - bar_width / 3, height / 2, width - 2 * bar_width / 3, height / 2 + 20);
}

int handle_screen_thread(vex::brain::lcd &screen, std::vector<screenFunc> pages, int first_page)
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
        pages[current_page](screen, bar_width, 0, width - bar_width * 2, height, first_draw);

        first_draw = false;
        // handle presses
        if (screen.pressing() && !was_pressing)
        {

            int x = screen.xPosition();
            int y = screen.yPosition();

            if (x < bar_width)
            {
                if (current_page == 0)
                {
                    current_page = num_pages;
                }
                current_page--;
                first_draw = true;
            }

            if (x > width - bar_width * 2)
            {
                current_page++;
                current_page %= num_pages;
                first_draw = true;
            }
            draw_lr_arrows(screen, bar_width, width, height);
        }
        was_pressing = screen.pressing();
        screen.render();
        vexDelay(40);
    }

    return 0;
}

void StartScreen(vex::brain::lcd &screen, std::vector<screenFunc> pages, int first_page)
{
    // hold onto arguments here so we don't lose them and can use then in the lambda down there. capture semantics are not fun
    static std::vector<screenFunc> my_pages = pages;
    static vex::brain::lcd my_screen = screen;
    static int my_first_page = first_page;
    vex::task screenTask([]()
                         { return handle_screen_thread(my_screen, my_pages, my_first_page); });
}