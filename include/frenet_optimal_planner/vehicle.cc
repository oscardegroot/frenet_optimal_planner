#include "vehicle.h"

namespace fop
{
  double Vehicle::length() { return 5.0; };
  double Vehicle::width() { return 2.0; };
  double Vehicle::L() { return 2.75; };
  double Vehicle::Lf() { return 1.25; };
  double Vehicle::Lr() { return 1.5; };

  double Vehicle::max_steering_angle() { return deg2rad(35); };
  double Vehicle::max_speed() { return kph2mps(120); };               // TODO: change back to 3.33
  double Vehicle::max_acceleration() { return max_speed()/20.0; };
  double Vehicle::max_deceleration() { return -max_speed()/7.0; };
  double Vehicle::max_curvature(const double delta_t) { return steering_angle_rate()/Lr()*delta_t; }; // TODO: origin 1.0  0.476 (for 30 degree), 
  double Vehicle::steering_angle_rate() { return max_steering_angle()/3.0; }; 

} // namespace fop