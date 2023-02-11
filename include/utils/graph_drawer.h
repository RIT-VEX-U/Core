#pragma once

#include <string>
#include <stdio.h>
#include "vex.h"

class GraphDrawer
{
public:
  GraphDrawer(vex::brain::lcd &screen, int num_samples, std::string x_label, std::string y_label, vex::color col, bool draw_border, double lower_bound, double upper_bound) : Screen(screen), sample_index(0), xlabel(x_label), ylabel(y_label), col(col), border(draw_border), upper(upper_bound), lower(lower_bound)
  {
    std::vector<Vector2D::point_t> temp(num_samples, Vector2D::point_t{.x = 0.0, .y = 0.0});
    samples = temp;
  }
  void add_sample(Vector2D::point_t sample)
  {
    samples[sample_index] = sample;
    sample_index++;
    sample_index %= samples.size();
  }
  void draw(int x, int y, int width, int height)
  {
    if (samples.size() < 1)
    {
      return;
    }

    double current_min = samples[0].y;
    double current_max = samples[0].y;
    double earliest_time = samples[0].x;
    double latest_time = samples[0].x;
    // collect data
    for (int i = 0; i < samples.size(); i++)
    {
      Vector2D::point_t p = samples[i];
      if (p.x < earliest_time)
      {
        earliest_time = p.x;
      }
      else if (p.x > latest_time)
      {
        latest_time = p.x;
      }

      if (p.y < current_min)
      {
        current_min = p.y;
      }
      else if (p.y > current_max)
      {
        current_max = p.y;
      }
    }
    if (fabs(latest_time - earliest_time) < 0.001)
    {
      Screen.printAt(width / 2, height / 2, "Not enough Data");
      return;
    }
    if (upper != lower)
    {
      current_min = lower;
      current_max = upper;
    }

    if (border)
    {
      Screen.setPenWidth(1);
      Screen.setPenColor(vex::white);
      Screen.setFillColor(bgcol);
      Screen.drawRectangle(x, y, width, height);
    }

    double time_range = latest_time - earliest_time;
    double sample_range = current_max - current_min;
    double x_s = (double)x;
    double y_s = (double)y + (double)height;
    Screen.setPenWidth(4);
    Screen.setPenColor(col);
    for (int i = sample_index; i < samples.size() + sample_index - 1; i++)
    {
      Vector2D::point_t p = samples[i % samples.size()];
      double x_pos = x_s + ((p.x - earliest_time) / time_range) * (double)width;
      double y_pos = y_s + ((p.y - current_min) / sample_range) * (double)(-height);

      Vector2D::point_t p2 = samples[(i + 1) % samples.size()];
      double x_pos2 = x_s + ((p2.x - earliest_time) / time_range) * (double)width;
      double y_pos2 = y_s + ((p2.y - current_min) / sample_range) * (double)(-height);

      Screen.drawLine((int)x_pos, (int)y_pos, (int)x_pos2, (int)y_pos2);
    }
  }

private:
  vex::brain::lcd &Screen;
  std::vector<Vector2D::point_t> samples;
  int sample_index = 0;
  std::string xlabel;
  std::string ylabel;
  vex::color col = vex::red;
  vex::color bgcol = vex::transparent;
  bool border;
  double upper;
  double lower;
};
