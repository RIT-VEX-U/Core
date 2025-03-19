#pragma once

#include "core/utils/math/geometry/translation2d.h"
#include "vex.h"
#include <cmath>
#include <stdio.h>
#include <string>
#include <vector>

class GraphDrawer {
  public:
    /// @brief Creates a graph drawer with the specified number of series (each series is a separate line)
    /// @param num_samples the number of samples to graph at a time (40 will graph the last 40 data points)
    /// @param lower_bound the bottom of the window when displaying (if upper_bound = lower_bound, auto calculate
    /// bounds)
    /// @param upper_bound the top of the window when displaying (if upper_bound = lower_bound, auto calculate bounds)
    /// @param colors the colors of the series. must be of size num_series
    /// @param num_series the number of series to graph
    GraphDrawer(
      int num_samples, double lower_bound, double upper_bound, std::vector<vex::color> colors, size_t num_series = 1
    );
    /**
     * add_samples adds a point to the graph, removing one from the back
     * @param sample an x, y coordinate of the next point to graph
     */
    void add_samples(std::vector<Translation2d> sample);

    /**
     * add_samples adds a point to the graph, removing one from the back
     * @param sample a y coordinate of the next point to graph, the x coordinate is gotten from vex::timer::system();
     * (time in ms)
     */
    void add_samples(std::vector<double> sample);

    /**
     * draws the graph to the screen in the constructor
     * @param x x position of the top left of the graphed region
     * @param y y position of the top left of the graphed region
     * @param width the width of the graphed region
     * @param height the height of the graphed region
     */
    void draw(vex::brain::lcd &screen, int x, int y, int width, int height);

  private:
    std::vector<std::vector<Translation2d>> series;
    int sample_index = 0;
    std::vector<vex::color> cols;
    vex::color bgcol = vex::transparent;
    bool border;
    double upper;
    double lower;
    bool auto_fit = false;
};
