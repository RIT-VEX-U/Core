#include "../core/include/subsystems/screen.h"
#include "../core/include/utils/math_util.h"

namespace screen {
/**
 * @brief The ScreenData class holds the data that will be passed to the
 * screen thread
 * 
 * You probably shouldn't have to use it
 */
struct ScreenData {
  ScreenData(const std::vector<screen_pages::Page *> &m_pages, int m_page, vex::brain::lcd &m_screen)
      : pages(m_pages), page(m_page), screen(m_screen) {}
  std::vector<screen_pages::Page *> pages;
  int page = 0;
  vex::brain::lcd screen;
};

static vex::thread *screen_thread = nullptr;
static bool running = false;
static int screen_thread_func(void *screen_data_v);
static ScreenData *screen_data_ptr;

/**
 * @brief start_screen begins a screen. Only call this once per program (a good
 * place is vexcodeInit)
 *
 * This is a set and forget type function. You don't have to wait on it or start
 * it in a new thread
 * @param screen The brain screen
 * @param pages The list of pages in the UI slideshow
 * @param first_page The page to start on (default is 0)
 */
void start_screen(vex::brain::lcd &screen, std::vector<screen_pages::Page *> pages, int first_page) {
  if (pages.size() == 0) {
    printf("No pages, not starting screen");
    return;
  }
  first_page %= pages.size();

  if (running) {
    printf("THERE IS ALREADY A SCREEN THREAD RUNNING\n");
    return;
  }

  ScreenData *data = new ScreenData{pages, first_page, screen};

  screen_thread = new vex::thread(screen_thread_func, static_cast<void *>(data));
}

/**
 * @brief stop_screen stops a screen
 */
void stop_screen() { running = false; }

/**
 * @brief Runs the screen thread
 *
 * This should only be called by start_screen
 *
 * If you are calling this, maybe don't
 */
int screen_thread_func(void *screen_data_v) {
  ScreenData &screen_data = *static_cast<ScreenData *>(screen_data_v);
  screen_data_ptr = static_cast<ScreenData *>(screen_data_v);
  running = true;
  unsigned int frame = 0;

  bool was_pressed = false;
  int x_press = 0;
  int y_press = 0;

  while (running) {
    screen_pages::Page *front_page = screen_data.pages[screen_data.page];
    bool pressing = screen_data.screen.pressing();

    if (pressing) {
      pressing = true;
      x_press = screen_data.screen.xPosition();
      y_press = screen_data.screen.yPosition();
    }
    bool just_pressed = pressing && !was_pressed;

    if (just_pressed && x_press < 40) {
      screen_data.page--;
      if (screen_data.page < 0) {
        screen_data.page += screen_data.pages.size();
      }
    }
    if (just_pressed && x_press > 440) {
      screen_data.page++;
      screen_data.page %= screen_data.pages.size();
    }

    // Update all pages
    for (auto page : screen_data.pages) {
      if (page == front_page) {
        page->update(was_pressed, x_press, y_press);
      } else {
        page->update(false, 0, 0);
      }
    }

    // Draw first page
    if (frame % 2 == 0) {
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
      // Left arrow
      screen_data.screen.drawLine(30, 100, 15, 120);
      screen_data.screen.drawLine(30, 140, 15, 120);
      // Right arrow
      screen_data.screen.drawLine(450, 100, 465, 120);
      screen_data.screen.drawLine(450, 140, 465, 120);
    }

    screen_data.screen.render();
    frame++;
    was_pressed = pressing;
    vexDelay(5);
  }

  return 0;
}
} // End namespace screen


namespace screen_widgets {
/**
 * @brief Draws a label on the screen
 * @param scr The screen to draw to
 * @param lbl The string to write
 * @param rect Where to put the text
 */
void draw_label(vex::brain::lcd &scr, std::string lbl, ScreenRect rect) {
  uint32_t height = scr.getStringHeight(lbl.c_str());
  scr.printAt(rect.x1 + 1, rect.y1 + height, true, "%s", lbl.c_str());
}

/**
 * @brief Draws a specific widget depending on the type
 *
 * Currently the only ones implemented in this method are label widgets
 */
void draw_widget(vex::brain::lcd &scr, WidgetConfig &widget, ScreenRect rect) {
  switch (widget.type) {
  case screen_widgets::WidgetConfig::Type::Col:
  case screen_widgets::WidgetConfig::Type::Row:
  case screen_widgets::WidgetConfig::Type::Slider:
  case screen_widgets::WidgetConfig::Type::Button:
  case screen_widgets::WidgetConfig::Type::Checkbox:
  case screen_widgets::WidgetConfig::Type::Graph:
    printf("unimplemented\n");
    break;
  case screen_widgets::WidgetConfig::Type::Label:
    draw_label(scr, widget.config.label.label, rect);
    break;
  }
}

bool SliderWidget::update(bool was_pressed, int x, int y) {
  const double margin = 10.0;
  if (was_pressed) {
    double dx = x;
    double dy = y;
    if (rect.contains(point_t{dx, dy})) {
      double pct = (dx - rect.min.x - margin) / (rect.dimensions().x - 2 * margin);
      pct = clamp(pct, 0.0, 1.0);
      value = (low + pct * (high - low));
    }
    return true;
  }
  return false;
}

void SliderWidget::draw(vex::brain::lcd &scr, bool first_draw [[maybe_unused]],
                        unsigned int frame_number [[maybe_unused]]) {
  if (rect.height() <= 0) {
    printf("Slider: %s has no height. Cant use it.", name.c_str());
  }
  double xl = rect.min.x;
  double xh = rect.max.x;
  double xmid = (xl + xh) / 2.0;
  double y = rect.min.y + rect.height() / 2;
  const double margin = 5.0;

  scr.setPenColor(vex::color(50, 50, 50));
  scr.setFillColor(vex::color(50, 50, 50));
  scr.setPenWidth(1);

  scr.drawRectangle(rect.min.x, rect.min.y, rect.dimensions().x, rect.dimensions().y);

  scr.setPenColor(vex::color(200, 200, 200));
  scr.setPenWidth(4);

  scr.drawLine(xl + margin, y, xh - margin, y);

  double pct = (value - low) / (high - low);
  double vx = pct * (rect.dimensions().x - (2 * margin)) + rect.min.x + margin;
  const double handle_width = 4;
  const double handle_height = 4;

  scr.drawRectangle(vx - (handle_width / 2), y - (handle_height / 2), handle_width, handle_height);
  int text_w = scr.getStringWidth((name + "        ").c_str());
  scr.printAt(xmid - text_w / 2, y - 15, false, "%s: %.5f", name.c_str(), value);
}

bool ButtonWidget::update(bool was_pressed, int x, int y) {
  if (was_pressed && !was_pressed_last && rect.contains({(double)x, (double)y})) {
    onpress();
    was_pressed_last = was_pressed;
    return true;
  }
  was_pressed_last = was_pressed;
  return false;
}

void ButtonWidget::draw(vex::brain::lcd &scr, bool first_draw [[maybe_unused]],
                        unsigned int frame_number [[maybe_unused]]) {
  scr.setPenColor(vex::white);
  scr.setPenWidth(1);
  scr.setFillColor(vex::color(50, 50, 50));
  scr.drawRectangle(rect.min.x, rect.min.y, rect.width(), rect.height());
  int w = scr.getStringWidth(name.c_str());
  int h = scr.getStringHeight(name.c_str());
  scr.printAt(rect.center().x - w / 2, rect.center().y + h / 2, name.c_str());
}
} // End namespace screen_widgets

namespace screen_pages {
void prev_page() {
  screen::screen_data_ptr->page--;
  if (screen::screen_data_ptr->page < 0) {
    screen::screen_data_ptr->page += screen::screen_data_ptr->pages.size();
  }
}
void next_page() {
  screen::screen_data_ptr->page++;
  screen::screen_data_ptr->page %= screen::screen_data_ptr->pages.size();
}
void goto_page(size_t page) {
  screen::screen_data_ptr->page = page;
  screen::screen_data_ptr->page %= screen::screen_data_ptr->pages.size();
}

FunctionPage::FunctionPage(update_func_t update_f, draw_func_t draw_f) : update_f(update_f), draw_f(draw_f) {}

/**
 * @brief Update uses the supplied update function to update this page
 */
void FunctionPage::update(bool was_pressed, int x, int y) {
  update_f(was_pressed, x, y);
}
/**
 * @brief Draw uses the supplied draw function to draw to the screen
 */
void FunctionPage::draw(vex::brain::lcd &screen, bool first_draw, unsigned int frame_number) {
  draw_f(screen, first_draw, frame_number);
}

StatsPage::StatsPage(std::map<std::string, vex::motor &> motors) : motors(motors) {}
void StatsPage::update(bool was_pressed, int x, int y) {
  (void)x;
  (void)y;
  (void)was_pressed;
}

/**
 * @brief Draws the stats of a specific motor to the screen
 * @param name The motor name
 * @param mot The vex motor
 * @param frame The frame to draw to
 * @param x The x-position to draw to
 * @param y The y-position to draw to
 */
void StatsPage::draw_motor_stats(const std::string &name, vex::motor &mot, unsigned int frame, int x, int y,
                                 vex::brain::lcd &scr) {
  const vex::color hot_col = vex::color(120, 0, 0);
  const vex::color med_col = vex::color(140, 100, 0);
  const vex::color ok_col = vex::black;
  const vex::color not_plugged_in_col = vex::color(255, 0, 0);
  bool installed = mot.installed();
  double temp = 0;
  int port = mot.index() + 1;
  vex::color col = ok_col;
  if (installed) {
    temp = mot.temperature(vex::temperatureUnits::celsius);
  }

  if (temp > 40) {
    col = med_col;
  } else if (temp > 50) {
    col = hot_col;
  }
  if (!installed && frame % 2 == 0) {
    col = not_plugged_in_col;
  }

  scr.drawRectangle(x, y, row_width, row_height, col);
  scr.printAt(x + 2, y + 16, false, " %2d   %2.0fC   %.7s", port, temp, name.c_str());
}

/**
 * @brief Draws the stats of all the motors and the battery to the screen
 * @param scr The screen to draw to
 */
void StatsPage::draw(vex::brain::lcd &scr, bool first_draw [[maybe_unused]],
                     unsigned int frame_number [[maybe_unused]]) {
  int num = 0;
  int x = 40;
  int y = y_start + row_height;
  scr.setPenWidth(1);

  scr.drawRectangle(x, y_start, row_width, row_height);
  scr.printAt(x, y_start + 16, false, " port temp name");
  for (auto &kv : motors) {
    if (num > per_column) {
      scr.drawRectangle(x + row_width, y_start, row_width, row_height);
      scr.printAt(x + row_width, y_start + 16, false, " port temp name");

      y = y_start + row_height;
      x += row_width;
      num = 0;
    }

    draw_motor_stats(kv.first, kv.second, frame_number, x, y, scr);
    y += row_height;
    num++;
  }
  vex::brain b;
  scr.printAt(50, 220, "Battery: %2.1fv  %2.0fC %d%%", b.Battery.voltage(),
              b.Battery.temperature(vex::temperatureUnits::celsius), b.Battery.capacity());
}

OdometryPage::OdometryPage(OdometryBase &odom, double width, double height, bool do_trail)
    : odom(odom), robot_width(width), robot_height(height), do_trail(do_trail),
      velocity_graph(30, 0.0, 0.0, {vex::green}, 1) {
  vex::brain b;
  if (b.SDcard.exists(field_filename)) {
    buf_size = b.SDcard.size(field_filename);
    buf = (uint8_t *)malloc(buf_size);
    b.SDcard.loadfile(field_filename, buf, buf_size);
  }
  pose_t pos = odom.get_position();
  for (int i = 0; i < path_len; i++) {
    path[i] = pos;
  }
}

/**
 * @brief Converts inches to pixels
 * @param in The number of inches to convert
 * @return The number of pixels
 */
int in_to_px(double in) {
  double p = in / (6.0 * 24.0);
  return (int)(p * 240);
}

/**
 * @brief Draws the odometry data to the screen
 * @param scr The screen to draw to
 */
void OdometryPage::draw(vex::brain::lcd &scr, bool first_draw [[maybe_unused]],
                        unsigned int frame_number [[maybe_unused]]) {
  pose_t pose = odom.get_position();
  path[path_index] = pose;

  if (do_trail && frame_number % 5 == 0) {
    path_index++;
    path_index %= path_len;
  }

  auto to_px = [](const point_t p) -> point_t { return {(double)in_to_px(p.x) + 200, (double)in_to_px(-p.y) + 240}; };

  auto draw_line = [to_px, &scr](const point_t from, const point_t to) {
    scr.drawLine((int)to_px(from).x, (int)to_px(from).y, (int)to_px(to).x, (int)to_px(to).y);
  };

  point_t pos = pose.get_point();
  fflush(stdout);
  scr.printAt(45, 30, "(%.2f, %.2f)", pose.x, pose.y);
  scr.printAt(45, 50, "%.2f deg", pose.rot);

  double speed = odom.get_speed();
  scr.printAt(45, 80, "%.2f speed", speed);
  velocity_graph.add_samples({speed});
  velocity_graph.draw(scr, 30, 100, 170, 120);

  if (buf == nullptr) {
    scr.printAt(180, 110, "Field Image Not Found");
    return;
  }

  scr.drawImageFromBuffer(buf, 200, 0, buf_size);

  point_t pos_px = to_px(pos);
  scr.drawCircle((int)pos_px.x, (int)pos_px.y, 3, vex::color::white);

  if (do_trail) {
    pose_t last_pos = path[(path_index + 1) % path_len];
    for (int i = path_index + 2; i < path_index + path_len; i++) {
      int j = i % path_len;
      pose_t pose = path[j];
      scr.setPenWidth(2);
      scr.setPenColor(vex::color(255, 255, 80));
      draw_line(pose.get_point(), last_pos.get_point());
      last_pos = pose;
    }
  }
  scr.setPenColor(vex::color::white);

  Mat2 mat = Mat2::FromRotationDegrees(pose.rot - 90);
  const point_t to_left = point_t{-robot_width / 2.0, 0};
  const point_t to_front = point_t{0.0, robot_height / 2.0};

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

/**
 * @brief Updates the odometry page
 * @param was_pressed If the screen was pressed
 * @param x The x-position of where the screen was pressed
 * @param y The y-position of where the screen was pressed
 */
void OdometryPage::update(bool was_pressed, int x, int y) {
  (void)x;
  (void)y;
  (void)was_pressed;
}

PIDPage::PIDPage(PID &pid, std::string name, std::function<void(void)> onchange)
    : cfg(pid.config), pid(pid), name(name), onchange(onchange),
      p_slider(cfg.p, 0.0, 0.5, Rect{{60, 20}, {210, 60}}, "P"),
      i_slider(cfg.i, 0.0, 0.05, Rect{{60, 80}, {180, 120}}, "I"),
      d_slider(cfg.d, 0.0, 0.05, Rect{{60, 140}, {180, 180}}, "D"),
      zero_i([this]() { zero_i_f(); }, Rect{{180, 80}, {220, 120}}, "0"),
      zero_d([this]() { zero_d_f(); }, Rect{{180, 140}, {220, 180}}, "0"), graph(40, 0, 0, {vex::red, vex::green}, 2) {}

PIDPage::PIDPage(PIDFF &pidff, std::string name, std::function<void(void)> onchange)
    : PIDPage((pidff.pid), name, onchange) {}

/**
 * @brief Updates the PIDPage with new data
 * 
 * Passes in the data to the update function of each widget seprately
 * @param was_pressed If the screen was pressed or not
 * @param x The x-position of where the screen was pressed
 * @param y The y-position of where the screen was pressed
 */
void PIDPage::update(bool was_pressed, int x, int y) {
  bool updated = false;
  updated |= p_slider.update(was_pressed, x, y);
  updated |= i_slider.update(was_pressed, x, y);
  updated |= d_slider.update(was_pressed, x, y);

  updated |= zero_i.update(was_pressed, x, y);
  updated |= zero_d.update(was_pressed, x, y);
  if (updated) {
    onchange();
  }
}

/**
 * @brief Draws the PIDPage
 *
 * Draws each widget separately then writes the PID data to the screen
 * @param scr The screen to draw to
 */
void PIDPage::draw(vex::brain::lcd &scr, bool first_draw [[maybe_unused]], unsigned int frame_number [[maybe_unused]]) {
  p_slider.draw(scr, first_draw, frame_number);
  i_slider.draw(scr, first_draw, frame_number);
  d_slider.draw(scr, first_draw, frame_number);
  zero_i.draw(scr, first_draw, frame_number);
  zero_d.draw(scr, first_draw, frame_number);

  graph.add_samples({pid.get_target(), pid.get_sensor_val()});

  graph.draw(scr, 230, 20, 200, 200);

  scr.setPenColor(vex::white);
  scr.printAt(60, 215, false, "%s", name.c_str());

  scr.setPenColor(vex::red);
  scr.printAt(240, 20, false, "%.2f", pid.get_target());
  scr.setPenColor(vex::green);
  scr.printAt(300, 20, false, "%.2f", pid.get_sensor_val());
}
} // End namespace screen_pages