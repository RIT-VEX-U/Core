#pragma once

#include <string>
#include <stdio.h>
#include <vector>
#include <cmath>
#include "vex.h"
#include "../core/include/utils/geometry.h"
#include "../core/include/utils/vector2d.h"

class GraphDrawer
{
public:
  /**
   * Construct a GraphDrawer
   * @brief a helper class to graph values on the brain screen
   * @param screen a reference to Brain.screen we can save for later
   * @param num_samples the graph works on a fixed window and will plot the last `num_samples` before the history is forgotten. Larger values give more context but may slow down if you have many graphs or an exceptionally high
   * @param x_label the name of the x axis (currently unused)
   * @param y_label the name of the y axis (currently unused)
   * @param draw_border whether to draw the border around the graph. can be turned off if there are multiple graphs in the same space ie. a graph of error and output
   * @param lower_bound the bottom of the window to graph. if lower_bound == upperbound, the graph will scale to it's datapoints
   * @param upper_bound the top of the window to graph. if lower_bound == upperbound, the graph will scale to it's datapoints
   */
  GraphDrawer(vex::brain::lcd &screen, int num_samples, std::string x_label, std::string y_label, vex::color col, bool draw_border, double lower_bound, double upper_bound);
  /**
   * add_sample adds a point to the graph, removing one from the back
   * @param sample an x, y coordinate of the next point to graph
   */
  void add_sample(point_t sample);
  /**
   * draws the graph to the screen in the constructor
   * @param x x position of the top left of the graphed region
   * @param y y position of the top left of the graphed region
   * @param width the width of the graphed region
   * @param height the height of the graphed region
   */
  void draw(int x, int y, int width, int height);

private:
  vex::brain::lcd &Screen;
  std::vector<point_t> samples;
  int sample_index = 0;
  std::string xlabel;
  std::string ylabel;
  vex::color col = vex::red;
  vex::color bgcol = vex::transparent;
  bool border;
  double upper;
  double lower;
};
