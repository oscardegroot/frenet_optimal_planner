/** math_utils.cc
 * 
 * Copyright (C) 2019 SS47816 & Advanced Robotics Center, National University of Singapore & Micron Technology
 * 
 * Commonly used math functions
 */

#include "math_utils.h"

namespace fop
{

// Return PI
constexpr double pi() { return M_PI; }

// Convert degrees to radians
double deg2rad(const double x) { return x * pi() / 180; }

// Convert radians to degrees
double rad2deg(const double x) { return x * 180 / pi(); }

// Convert metre per second to kilometers per hour
double mpsTokph(const double x) { return x * 3.6; }

// Convert kilometers per hour to meter per second
double kphTomps(const double x) { return x / 3.6; }

// Convert angle into range [-pi, +pi]
double unifyAngleRange(const double angle)
{
  auto new_angle = angle;
  while (new_angle > M_PI)
  {
    new_angle -= 2 * M_PI;
  }
  while (new_angle < -M_PI)
  {
    new_angle += 2 * M_PI;
  }
  return new_angle;
}

// Limit the value within [lower_bound, upper_bound]
double limitWithinRange(double value, const double lower_bound, const double upper_bound)
{
  value = std::max(value, lower_bound);
  value = std::min(value, upper_bound);
  return value;
}

// Calculate the Euclideam distance between two points
double distance(const double x1, const double y1, const double x2, const double y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Calculate the Euclideam distance between two poses
double distance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b)
{
  return distance(a.position.x, a.position.y, b.position.x, b.position.y);
}

// Calculate the Euclideam distance between two points
double magnitude(const double x, const double y, const double z)
{
  return sqrt(x*x + y*y + z*z);
}

} // end of namespace fop