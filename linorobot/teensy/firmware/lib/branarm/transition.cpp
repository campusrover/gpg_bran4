#include <cmath>

class Transition
{
public:
  // Helper functions
  double
  check_exceptions (double calc, double change_in_value, double start_value)
  {
    if (change_in_value > start_value)
      {
        if (calc > (change_in_value + start_value))
          {
            return change_in_value + start_value;
          }
      }
    else if (change_in_value < start_value)
      {
        if (calc < (change_in_value + start_value))
          {
            return change_in_value + start_value;
          }
      }
    return calc;
  }

  double
  check_duration (double calc, double current_time, double duration, double start_time, double target_angle)
  {
    if (current_time > duration)
      {
        return target_angle;
      }
    return calc;
  }

  // Tweening functions
  int
  linear_tween (double current_time, double start_value, double change_in_value, double duration, double start_time,
                double target_angle)
  {
    double calc = ((change_in_value * current_time) / duration) + start_value;
    calc = check_exceptions (calc, change_in_value, start_value);
    calc = check_duration (calc, current_time, duration, start_time, target_angle);
    return static_cast<int> (calc);
  }

  // Ease In Out Quad
  int
  ease_in_out_quad (double current_time, double start_value, double change_in_value, double duration, double start_time,
                    double target_angle)
  {
    current_time /= duration / 2;
    if (current_time < 1)
      {
        return static_cast<int> (change_in_value / 2 * current_time * current_time + start_value);
      }
    current_time--;
    double calc = -change_in_value / 2 * (current_time * (current_time - 2) - 1) + start_value;
    calc = check_exceptions (calc, change_in_value, start_value);
    calc = check_duration (calc, current_time, duration, start_time, target_angle);
    return static_cast<int> (calc);
  }

  // Ease In Out Cubic
  int
  ease_in_out_cubic (double current_time, double start_value, double change_in_value, double duration,
                     double start_time, double target_angle)
  {
    current_time /= duration / 2;
    if (current_time < 1)
      {
        return static_cast<int> (change_in_value / 2 * current_time * current_time * current_time + start_value);
      }
    current_time -= 2;
    double calc = change_in_value / 2 * (current_time * current_time * current_time + 2) + start_value;
    calc = check_exceptions (calc, change_in_value, start_value);
    calc = check_duration (calc, current_time, duration, start_time, target_angle);
    return static_cast<int> (calc);
  }

  // Ease In Out Quart
  int
  ease_in_out_quart (double current_time, double start_value, double change_in_value, double duration,
                     double start_time, double target_angle)
  {
    current_time /= duration / 2;
    if (current_time < 1)
      {
        return static_cast<int> (change_in_value / 2 * current_time * current_time * current_time * current_time
                                 + start_value);
      }
    current_time -= 2;
    double calc = -change_in_value / 2 * (current_time * current_time * current_time * current_time - 2) + start_value;
    calc = check_exceptions (calc, change_in_value, start_value);
    calc = check_duration (calc, current_time, duration, start_time, target_angle);
    return static_cast<int> (calc);
  }

  // Ease In Out Quint
  int
  ease_in_out_quint (double current_time, double start_value, double change_in_value, double duration,
                     double start_time, double target_angle)
  {
    current_time /= duration / 2;
    if (current_time < 1)
      {
        return static_cast<int> (change_in_value / 2 * current_time * current_time * current_time * current_time
                                     * current_time
                                 + start_value);
      }
    current_time -= 2;
    double calc = change_in_value / 2 * (current_time * current_time * current_time * current_time * current_time + 2)
                  + start_value;
    calc = check_exceptions (calc, change_in_value, start_value);
    calc = check_duration (calc, current_time, duration, start_time, target_angle);
    return static_cast<int> (calc);
  }
};
