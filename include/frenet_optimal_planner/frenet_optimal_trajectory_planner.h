#ifndef FRENET_OPTIMAL_TRAJECTORY_PLANNER_H_
#define FRENET_OPTIMAL_TRAJECTORY_PLANNER_H_

#include <cmath>
#include <vector>
#include <iostream>
#include <future>
#include <queue>

#include "frenet.h"
#include "math_utils.h"
#include "quintic_polynomial.h"
#include "quartic_polynomial.h"
#include "spline.h"
#include "vehicle_state.h"
#include "vehicle.h"

// #include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <autoware_msgs/DetectedObjectArray.h>

#include "sat_collision_checker.h"

#include <lmpcc_msgs/obstacle_array.h>
#include <lmpcc_msgs/obstacle_gmm.h>
#include <lmpcc_msgs/gaussian.h>
#include <frenet_optimal_planner/ObservedRisk.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>




// #define TRUE_SIZE_LENGTH 3
// #define TRUE_SIZE_MARGIN 0.3

class SignalPublisher
{
public:
  SignalPublisher(ros::NodeHandle &nh, const std::string &signal_name)
  {
    pub_ = nh.advertise<std_msgs::Float32>("/frenet_optimal_planner/" + signal_name, 1); /* Publish the sample size when it is incorrect */
  }

  void Publish(double signal_value)
  {
    msg.data = signal_value;
    pub_.publish(msg);
  }

private:
  ros::Publisher pub_;
  std_msgs::Float32 msg;
};

namespace fop
{

class FrenetOptimalTrajectoryPlanner
{
 public:
  struct Setting
  {
   public:
    Setting() {};
    virtual ~Setting() {};

    // General Settings
    double tick_t;              // time tick [s]
    bool enable_debug;
    bool replan_all_time;

    // Sampling Parameters
    double center_offset;       // offset from the center of the lane [m]
    int num_width;              // number of road width samples
    double max_t;               // max prediction time [s]
    double min_t;               // min prediction time [s]
    int num_t;                  // number of time samples
    double highest_speed;       // highest target speed [m/s]
    double lowest_speed;        // lowest target speed [m/s]
    int num_speed;              // number of speed samples

    // Hard Constraints
    double max_speed;           // maximum speed [m/s]
    double max_accel;           // maximum acceleration [m/ss]
    double max_decel;           // maximum deceleration [m/ss]
    double max_curvature;       // maximum curvature [rad/m]
    double max_jerk_s;          // maximum longitudinal jerk [m/s^3]
    double max_jerk_d;          // maximum lateral jerk [m/s^3]
    // double steering_angle_rate; // [rad/s]

    // Cost Weights
    double k_jerk;              // jerk cost weight
    double k_time;              // time cost weight
    double k_diff;              // speed and lateral offset cost weight
    double k_lat;               // lateral overall cost weight
    double k_lon;               // longitudinal overall cost weight
    double k_obstacle;          // obstacle cost weight
    double k_heuristic;         // heuristic cost weight

    // Collision Parameters
    double safety_margin_lon;   // lon safety margin [ratio]
    double safety_margin_lat;   // lat safety margin [ratio]
    double safety_margin_soft;  // soft safety margin [ratio]
    double vehicle_width;       // vehicle width [m]
    double vehicle_length;      // vehicle length [m]
    double obstacle_radius;
  };

  class TestResult
  {
   public:
    int length;
    int count;
    std::vector<int> numbers;
    std::vector<int> numbers_min;
    std::vector<int> numbers_max;
    std::vector<int> total_numbers;
    std::vector<double> time;
    std::vector<double> time_min;
    std::vector<double> time_max;
    std::vector<double> total_time;

    double total_fix_cost, total_dyn_cost;
    double total_dist;


    TestResult();
    TestResult(const int length);
    std::vector<double> updateCount(const std::vector<int> numbers, const std::vector<std::chrono::_V2::system_clock::time_point> timestamps,
                                    const double fix_cost, const double dyn_cost, const double dist);
    void printSummary();
  };

  /* --------------------------------- Methods -------------------------------- */

  // Constructors
  FrenetOptimalTrajectoryPlanner();
  FrenetOptimalTrajectoryPlanner(Setting& settings);

  // Destructor
  virtual ~FrenetOptimalTrajectoryPlanner(){};

  void updateSettings(Setting& settings);

  frenet_optimal_planner::ObservedRisk risk_srv_;
  ros::ServiceClient risk_planned_traj_client_;

  /* Public Functions */
  // Generate reference curve as the frenet s coordinate
  std::pair<Path, Spline2D> generateReferenceCurve(const fop::Lane& lane);

  // Plan for the optimal trajectory
  // the returning type can be changed
  std::vector<fop::FrenetPath> frenetOptimalPlanning(fop::Spline2D& cubic_spline, const fop::FrenetState& frenet_state, const int lane_id,
                                                     const double left_width, const double right_width, const double current_speed, const bool check_collision, const bool use_async, 
                                                     const lmpcc_msgs::obstacle_array& obstacles, const bool use_heuristic, fop::Path& curr_traj, double r_x_,  ros::ServiceClient risk_planned_traj_client);

 
  std::vector<FrenetPath> all_trajs_;
  std::shared_ptr<std::vector<fop::FrenetPath>> all_trajs_fop_;

  
  std::priority_queue<FrenetPath, std::vector<FrenetPath>, std::greater<std::vector<FrenetPath>::value_type>> candidate_trajs_;
  FrenetPath best_traj_, prev_best_traj_;
  Eigen::Vector3i prev_best_idx_;

private:
  Setting settings_;
  TestResult test_result_;
  FrenetState start_state_;
  SATCollisionChecker sat_collision_checker_;

  // Sample candidate trajectories
  std::vector<fop::FrenetPath> generateFrenetPaths(const fop::FrenetState& frenet_state, const int lane_id,
                                                   const double left_bound, const double right_bound, const double current_speed);

  // Convert paths from frenet frame to gobal map frame
  int calculateGlobalPaths(std::vector<fop::FrenetPath>& frenet_traj_list, fop::Spline2D& cubic_spline, double r_x);
  // Compute costs for candidate trajectories
  int computeCosts(std::vector<fop::FrenetPath>& frenet_trajs, const double curr_speed);

  // Check for vehicle kinematic constraints
  bool checkConstraints(FrenetPath& traj);
  
  
  bool checkCollisions(FrenetPath& ego_traj, const lmpcc_msgs::obstacle_array& obstacle_trajs, 
                       const bool use_async, int& num_checks, fop::Path& curr_traj, ros::ServiceClient risk_planned_traj_client);

  std::pair<bool, int> checkTrajCollision(const FrenetPath& ego_traj, const lmpcc_msgs::obstacle_array& obstacle_trajs, 
                                          const double margin_lon, const double margin_lat, fop::Path& curr_traj);
  
};

}  // namespace fop

#endif  // FRENET_OPTIMAL_TRAJECTORY_PLANNER_H_