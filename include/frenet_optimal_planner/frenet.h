#ifndef FRENET_H_
#define FRENET_H_

#include <cmath>
#include <vector>
#include <iostream>

#include "Eigen/Dense"
#include "lane.h"
#include "math_utils.h"
#include "vehicle_state.h"


namespace fop
{

class FrenetState
{
 public:
  // Constructor
  FrenetState(){};
  // Destructor
  virtual ~FrenetState(){};
  
  double T;
  double s;
  double s_d;
  double s_dd;
  double s_ddd;
  double d;
  double d_d;
  double d_dd;
  double d_ddd;
};

class FrenetPath
{
 public:
  // Constructor
  FrenetPath(); // {};
  FrenetPath(const int lane_id, FrenetState& end_state, const double fix_cost, const double heu_cost);
  // Destructor
  virtual ~FrenetPath() {};

  friend bool operator < (const FrenetPath& lhs, const FrenetPath& rhs);
  friend bool operator > (const FrenetPath& lhs, const FrenetPath& rhs);

  Eigen::Vector3i idx;
  int lane_id;
  bool is_generated;  // label if this trajectory has been generated or not
  bool is_searched;   // label if this trajectory has been searched before
  // checks
  bool constraint_passed;
  bool collision_passed;
  bool shifted;
  
  // costs
  double fix_cost;    // fixed cost term
  double dyn_cost;    // cost terms to be determined after generation
  double heu_cost;    // heuristic cost term
  double lat_cost;    // deviation from reference
  double speed_cost;  // deviation from maximum speed
  double time_cost;   // length of trajectory
  double est_cost;    // cost term estimated before generation (fix_cost + heu_cost)
  double final_cost;  // final cost for a generated trajectory
  
  // time list
  std::vector<double> t;
  // longitudinal
  std::vector<double> s;
  std::vector<double> s_d;
  std::vector<double> s_dd;
  std::vector<double> s_ddd;
  // lateral
  std::vector<double> d;
  std::vector<double> d_d;
  std::vector<double> d_dd;
  std::vector<double> d_ddd;
  // state
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> ds;
  std::vector<double> c;

  std::vector<double> r_x;
  std::vector<double> r_y;
  std::vector<double> r_yaw;

  FrenetState end_state;
};

// Convert the position in Cartesian coordinates to Frenet frame
std::pair<FrenetState, double> getFrenet(const VehicleState& current_state, const Lane& lane);
std::pair<FrenetState, double> getFrenet(const VehicleState& current_state, const Path& path);

}  // end of namespace fop


#endif  // FRENET_H_