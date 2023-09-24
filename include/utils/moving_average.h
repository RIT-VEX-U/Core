#pragma once
#include <vector>

/**
 * MovingAverage
 *
 * A moving average is a way of smoothing out noisy data. For many sensor readings, the noise is roughly symmetric around the actual value. 
 * This means that if you collect enough samples those that are too high are cancelled out by the samples that are too low leaving the real value.
 *
 * The MovingAverage class provides a simple interface to do this smoothing from our noisy sensor values. 
 *
 * WARNING: because we need a lot of samples to get the actual value, the value given by the MovingAverage will 'lag' behind the actual value that the sensor is reading. 
 * Using a MovingAverage is thus a tradeoff between accuracy and lag time (more samples) vs. less accuracy and faster updating (less samples).  
 *
 */
class MovingAverage {
  public:
  /*
   * Create a moving average calculator with 0 as the default value
   *
   * @param buffer_size    The size of the buffer. The number of samples that constitute a valid reading
   */
  MovingAverage(int buffer_size);
  /*
   * Create a moving average calculator with a specified default value
   * @param buffer_size    The size of the buffer. The number of samples that constitute a valid reading
   * @param starting_value The value that the average will be before any data is added
   */
  MovingAverage(int buffer_size, double starting_value);

  /*
  * Add a reading to the buffer
  * Before:
  * [ 1 1 2 2 3 3] => 2
  *   ^
  * After:
  * [ 2 1 2 2 3 3] => 2.16
  *     ^ 
  * @param n  the sample that will be added to the moving average.
  */
  void add_entry(double n);

  /**
   * Returns the average based off of all the samples collected so far
   * @return the calculated average. sum(samples)/numsamples
   */
  double get_average();

  /**
  * How many samples the average is made from
  * @return the number of samples used to calculate this average
  */ 
  int get_size();


  private:
    int buffer_index;               //index of the next value to be overridden
    std::vector<double> buffer;     //all current data readings we've taken 
    double current_avg;             //the current value of the data

};