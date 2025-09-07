#include "core/utils/graph_drawer.h"

/// @brief Creates a graph drawer with the specified number of series (each series is a separate line)
/// @param num_samples the number of samples to graph at a time (40 will graph the last 40 data points)
/// @param lower_bound the bottom of the window when displaying (if upper_bound = lower_bound, auto calculate bounds)
/// @param upper_bound the top of the window when displaying (if upper_bound = lower_bound, auto calculate bounds)
/// @param colors the colors of the series. must be of size num_series
/// @param num_series the number of series to graph
GraphDrawer::GraphDrawer(int num_samples, double lower_bound, double upper_bound, std::vector<vex::color> colors,
                         size_t num_series)
    : cols(colors), auto_fit(lower_bound == upper_bound) {
  if (colors.size() != num_series) {
    printf("The number of colors does not match the number of series in graph drawer\n");
  }
  series = std::vector<std::vector<Translation2d>>(num_series);
  for (size_t i = 0; i < num_series; i++) {
    series[i] = std::vector<Translation2d>(num_samples, {0.0, 0.0});
  }

  if (auto_fit) {
    upper = -1000.0;
    lower = 1000.0;
  } else {
    upper = upper_bound;
    lower = lower_bound;
  }
}

/**
 * add_samples adds a point to the graph, removing one from the back
 * @param sample an x, y coordinate of the next point to graph
 */
void GraphDrawer::add_samples(std::vector<Translation2d> new_samples) {
  if (series.size() != new_samples.size()) {
    printf("Mismatch between # of samples given and number of series. %s : %d\n", __FILE__, __LINE__);
  }
  for (size_t i = 0; i < series.size(); i++) {
    series[i][sample_index] = new_samples[i];
    if (auto_fit) {
      upper = fmax(upper, new_samples[i].y());
      lower = fmin(lower, new_samples[i].y());
    }
  }
  sample_index++;
  sample_index %= series[0].size();
}

void GraphDrawer::add_samples(std::vector<double> new_samples) {
  if (series.size() != new_samples.size()) {
    printf("Mismatch between # of samples given and number of series. %s : %d\n", __FILE__, __LINE__);
  }
  for (size_t i = 0; i < series.size(); i++) {
    series[i][sample_index] = {(double)vex::timer::system(), new_samples[i]};
    if (auto_fit) {
      upper = fmax(upper, new_samples[i]);
      lower = fmin(lower, new_samples[i]);
    }
  }
  sample_index++;
  sample_index %= series[0].size();
}

/**
 * draws the graph to the screen in the constructor
 * @param x x position of the top left of the graphed region
 * @param y y position of the top left of the graphed region
 * @param width the width of the graphed region
 * @param height the height of the graphed region
 */
void GraphDrawer::draw(vex::brain::lcd& screen, int x, int y, int width, int height) {
  if (series[0].size() < 1) {
    return;
  }
  if (cols.size() != series.size()) {
    printf("The number of colors does not match the number of series in graph drawer\n");
  }

  size_t newest_index = (sample_index - 1);
  if (sample_index < 0) {
    sample_index += series[0].size();
  }

  double earliest_time = series[0][sample_index].x();
  double latest_time = series[0][newest_index].x();
  // collect data
  if (std::abs(latest_time - earliest_time) < 0.001) {
    screen.printAt(width / 2, height / 2, "Not enough Data");
    return;
  }

  if (border) {
    screen.setPenWidth(1);
    screen.setPenColor(vex::white);
    screen.setFillColor(bgcol);
    screen.drawRectangle(x, y, width, height);
  }

  double time_range = latest_time - earliest_time;
  double sample_range = upper - lower;
  screen.setPenWidth(2);

  for (int j = 0; j < series.size(); j++) {
    double x_s = (double)x;
    double y_s = (double)y + (double)height;
    const std::vector<Translation2d>& samples = series[j];

    screen.setPenColor(cols[j]);
    for (int i = sample_index; i < samples.size() + sample_index - 1; i++) {
      Translation2d p = samples[i % samples.size()];
      double x_pos = x_s + ((p.x() - earliest_time) / time_range) * (double)width;
      double y_pos = y_s + ((p.y() - lower) / sample_range) * (double)(-height);

      Translation2d p2 = samples[(i + 1) % samples.size()];
      double x_pos2 = x_s + ((p2.x() - earliest_time) / time_range) * (double)width;
      double y_pos2 = y_s + ((p2.y() - lower) / sample_range) * (double)(-height);

      screen.drawLine((int)x_pos, (int)y_pos, (int)x_pos2, (int)y_pos2);
    }
  }
}
