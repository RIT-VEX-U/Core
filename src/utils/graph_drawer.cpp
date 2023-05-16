#include "../core/include/utils/graph_drawer.h"

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
GraphDrawer::GraphDrawer(vex::brain::lcd &screen, int num_samples, std::string x_label, std::string y_label, vex::color col, bool draw_border, double lower_bound, double upper_bound) : Screen(screen), sample_index(0), xlabel(x_label), ylabel(y_label), col(col), border(draw_border), upper(upper_bound), lower(lower_bound)
{
    std::vector<point_t> temp(num_samples, point_t{.x = 0.0, .y = 0.0});
    samples = temp;
}

/**
 * add_sample adds a point to the graph, removing one from the back
 * @param sample an x, y coordinate of the next point to graph
 */
void GraphDrawer::add_sample(point_t sample)
{
    samples[sample_index] = sample;
    sample_index++;
    sample_index %= samples.size();
}

/**
 * draws the graph to the screen in the constructor
 * @param x x position of the top left of the graphed region
 * @param y y position of the top left of the graphed region
 * @param width the width of the graphed region
 * @param height the height of the graphed region
 */
void GraphDrawer::draw(int x, int y, int width, int height)
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
        point_t p = samples[i];
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
    if (std::abs(latest_time - earliest_time) < 0.001)
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
        point_t p = samples[i % samples.size()];
        double x_pos = x_s + ((p.x - earliest_time) / time_range) * (double)width;
        double y_pos = y_s + ((p.y - current_min) / sample_range) * (double)(-height);

        point_t p2 = samples[(i + 1) % samples.size()];
        double x_pos2 = x_s + ((p2.x - earliest_time) / time_range) * (double)width;
        double y_pos2 = y_s + ((p2.y - current_min) / sample_range) * (double)(-height);

        Screen.drawLine((int)x_pos, (int)y_pos, (int)x_pos2, (int)y_pos2);
    }
}
