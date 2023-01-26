#include "frenet_optimal_trajectory_planner.h"

namespace fop
{

FrenetOptimalTrajectoryPlanner::TestResult::TestResult() : count(0), total_fix_cost(0.0), total_dyn_cost(0.0), total_dist(0.0)
{
  this->numbers = std::vector<int>(5, int(0));
  this->numbers_min = std::vector<int>(5, int(100000));
  this->numbers_max = std::vector<int>(5, int(0));
  this->total_numbers = std::vector<int>(5, int(0));

  this->time = std::vector<double>(6, double(0));
  this->time_min = std::vector<double>(6, double(100000));
  this->time_max = std::vector<double>(6, double(0));
  this->total_time = std::vector<double>(6, double(0));

  this->numbers.shrink_to_fit();
  this->numbers_min.shrink_to_fit();
  this->numbers_max.shrink_to_fit();
  this->total_numbers.shrink_to_fit();

  this->time.shrink_to_fit();
  this->time_min.shrink_to_fit();
  this->time_max.shrink_to_fit();
  this->total_time.shrink_to_fit();
}

FrenetOptimalTrajectoryPlanner::TestResult::TestResult(const int length) : length(length), count(0), total_fix_cost(0.0), total_dyn_cost(0.0), total_dist(0)
{
  this->numbers = std::vector<int>(length, int(0));
  this->numbers_min = std::vector<int>(length, int(100000));
  this->numbers_max = std::vector<int>(length, int(0));
  this->total_numbers = std::vector<int>(length, int(0));

  this->time = std::vector<double>(length+1, double(0.0));
  this->time_min = std::vector<double>(length+1, double(100000));
  this->time_max = std::vector<double>(length+1, double(0.0));
  this->total_time = std::vector<double>(length+1, double(0.0));

  this->numbers.shrink_to_fit();
  this->numbers_min.shrink_to_fit();
  this->numbers_max.shrink_to_fit();
  this->total_numbers.shrink_to_fit();

  this->time.shrink_to_fit();
  this->time_min.shrink_to_fit();
  this->time_max.shrink_to_fit();
  this->total_time.shrink_to_fit();
} 

std::vector<double> FrenetOptimalTrajectoryPlanner::TestResult::updateCount(const std::vector<int> numbers, const std::vector<std::chrono::_V2::system_clock::time_point> timestamps,
                                                                            const double fix_cost, const double dyn_cost, const double dist)
{
  if (numbers.size() != this->length || timestamps.size() != this->length + 1)
  {
    std::cout << "Recorded TestResult for this planning iteration is invalid" << std::endl;
    return std::vector<double>{4, 0.0};
  }
  
  this->count++;

  // Update the numbers for the current iteration
  this->numbers = numbers;
  for (int i = 0; i < this->length; ++i)
  {
    this->numbers_min[i] = std::min(this->numbers_min[i], this->numbers[i]);
    this->numbers_max[i] = std::max(this->numbers_max[i], this->numbers[i]);
  }

  // Add the current numbers to total numbers
  std::transform(this->total_numbers.begin(), this->total_numbers.end(), numbers.begin(), this->total_numbers.begin(), std::plus<int>());
  
  // Calculate the elapsed_time for the current iteration, in [ms]
  for (int i = 0; i < timestamps.size() - 1; i++)
  {
    const std::chrono::duration<double, std::milli> elapsed_time = timestamps[i+1] - timestamps[i];
    this->time[i] = elapsed_time.count();
  }
  const std::chrono::duration<double, std::milli> elapsed_time = timestamps.back() - timestamps.front();
  this->time.back() = elapsed_time.count();
  
  // Update the time for the current iteration
  for (int i = 0; i < this->length+1; ++i)
  {
    this->time_min[i] = std::min(this->time_min[i], this->time[i]);
    this->time_max[i] = std::max(this->time_max[i], this->time[i]);
  }
  
  // Add the current elapsed_time to total time, in [ms]
  std::transform(this->total_time.begin(), this->total_time.end(), this->time.begin(), this->total_time.begin(), std::plus<double>());

  // Add the current optimal cost & distance from previous best
  total_fix_cost += fix_cost;
  total_dyn_cost += dyn_cost;
  total_dist += dist;
  // cost_history.emplace_back(cost);
  // dist_history.emplace_back(dist);
  
  std::vector<double> stats;
  stats.push_back(this->time[5]);
  stats.push_back(std::round(1000/this->time[5]));
  stats.push_back(double(this->numbers[2]));
  stats.push_back(fix_cost + dyn_cost);
  return stats;
}

void FrenetOptimalTrajectoryPlanner::TestResult::printSummary()
{
  const int count = std::max(1, int(this->count));
  /*
  // Print Summary for this iteration
  std::cout << " " << std::endl;
  std::cout << "Summary: This Planning Iteration (iteration no." << this->count << ")" << std::endl;
  std::cout << "Step 1 : Generated               " << this->numbers[0] << " Trajectories in " << this->time[0] << " ms" << std::endl;
  std::cout << "Step 2 : Converted               " << this->numbers[1] << " Trajectories in " << this->time[1] << " ms" << std::endl;
  std::cout << "Step 3 : Computed Cost for       " << this->numbers[2] << " Trajectories in " << this->time[2] << " ms" << std::endl;
  std::cout << "Step 4 : Checked Constraints for " << this->numbers[3] << " Trajectories in " << this->time[3] << " ms" << std::endl;
  std::cout << "Step 5 : Checked Collisions for  " << this->numbers[4] << " PolygonPairs in " << this->time[4] << " ms" << std::endl;
  std::cout << "Total  : Planning Took           " << this->time[5] << " ms (or " << 1000/this->time[5] << " Hz)" << std::endl;
  std::cout << "Dist   : Distance to History Best" << this->total_dist << std::endl;
  
  // Print Summary for average performance
  std::cout << " " << std::endl;
  std::cout << "Summary: Average Performance (" << this->count << " iterations so far)" << std::endl;
  std::cout << "Step 1 : Generated               " << this->total_numbers[0]/this->count << " Trajectories in " << this->total_time[0]/this->count << " ms" << std::endl;
  std::cout << "Step 2 : Converted               " << this->total_numbers[1]/this->count << " Trajectories in " << this->total_time[1]/this->count << " ms" << std::endl;
  std::cout << "Step 3 : Computed Cost for       " << this->total_numbers[2]/this->count << " Trajectories in " << this->total_time[2]/this->count << " ms" << std::endl;
  std::cout << "Step 4 : Checked Constraints for " << this->total_numbers[3]/this->count << " Trajectories in " << this->total_time[3]/this->count << " ms" << std::endl;
  std::cout << "Step 5 : Checked Collisions for  " << this->total_numbers[4]/this->count << " PolygonPairs in " << this->total_time[4]/this->count << " ms" << std::endl;
  std::cout << "Total  : Planning Took           " << this->total_time[5]/this->count << " ms (or " << 1000/(this->total_time[5]/this->count) << " Hz)" << std::endl;
  std::cout << "Cost   : Optimal's Fix Cost      " << this->total_fix_cost/count << std::endl;
  std::cout << "Cost   : Optimal's Dyn Cost      " << this->total_dyn_cost/count << std::endl;
  std::cout << "Cost   : Optimal's Total Cost    " << (this->total_fix_cost + this->total_dyn_cost)/count << std::endl;
  std::cout << "Dist   : Distance to History Best" << this->total_dist/count << std::endl;
  */
}

FrenetOptimalTrajectoryPlanner::FrenetOptimalTrajectoryPlanner()
{
  this->settings_ = Setting();
  this->test_result_ = TestResult(5);
}

FrenetOptimalTrajectoryPlanner::FrenetOptimalTrajectoryPlanner(FrenetOptimalTrajectoryPlanner::Setting& settings)
{
  this->settings_ = settings;
  this->test_result_ = TestResult(5);
  ros::NodeHandle nh;
  risk_planned_traj_client_ = nh.serviceClient<frenet_optimal_planner::ObservedRisk>("planned_traj_risk");

}

void FrenetOptimalTrajectoryPlanner::updateSettings(Setting& settings)
{
  this->settings_ = settings;
}

std::pair<Path, Spline2D> FrenetOptimalTrajectoryPlanner::generateReferenceCurve(const fop::Lane& lane)
{
  Path ref_path = Path();
  auto cubic_spline = fop::Spline2D(lane);

  std::vector<double> s;
  for (double i = 0; i < cubic_spline.s_.back(); i += 0.1)
  {
    s.emplace_back(i);
  }

  for (int i = 0; i < s.size(); i++)
  {
    fop::VehicleState state = cubic_spline.calculatePosition(s[i]);
    ref_path.x.emplace_back(state.x);
    ref_path.y.emplace_back(state.y);
    ref_path.yaw.emplace_back(cubic_spline.calculateYaw(s[i]));
  }

  return std::pair<Path, Spline2D>{ref_path, cubic_spline};
}


std::vector<fop::FrenetPath> 
FrenetOptimalTrajectoryPlanner::frenetOptimalPlanning(fop::Spline2D& cubic_spline, const fop::FrenetState& frenet_state, const int lane_id,
                                                      const double left_width, const double right_width, const double current_speed, const bool check_collision, const bool use_async, 
                                                      const lmpcc_msgs::obstacle_array& obstacles, const bool use_heuristic, fop::Path& curr_trajectory, double r_x,  ros::ServiceClient risk_planned_traj_client)
{
  // Clear the canidate trajectories from the last planning cycle
  std::priority_queue<FrenetPath, std::vector<FrenetPath>, std::greater<std::vector<FrenetPath>::value_type>> empty;
  std::swap(candidate_trajs_, empty);
  
  // Initialize a series of results to be recorded
  std::vector<int> numbers;
  std::vector<std::chrono::_V2::system_clock::time_point> timestamps;
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());
  // we don't need it at the moment, since we already have the predictions from the pedestrians simulator package
  //const auto obstacle_trajs = predictTrajectories(obstacles);

  // Sample a list of FrenetPaths
  all_trajs_fop_ = std::make_shared<std::vector<fop::FrenetPath>>(generateFrenetPaths(frenet_state, lane_id, left_width, right_width, current_speed));
  numbers.push_back(all_trajs_fop_->size());
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // Convert to global paths
  const int num_conversion_checks = calculateGlobalPaths(*all_trajs_fop_, cubic_spline, r_x); 
  numbers.push_back(num_conversion_checks);
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  // Compute costs
  const int num_cost_checks = computeCosts(*all_trajs_fop_, current_speed);

  //std::cout << "number of all generated trajectroties is: " << all_trajs_fop->size() << std::endl;
  //std::cout << "number of candidate trajectories is: " << candidate_trajs_.size() << std::endl;
  numbers.push_back(num_cost_checks);
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  int num_iter = 0;
  int num_constraint_checks = 0;
  int num_collision_checks = 0;

  bool best_traj_found = false;

  std::priority_queue<FrenetPath, std::vector<FrenetPath>, std::greater<std::vector<FrenetPath>::value_type>> candidate_trajs_debug;
  candidate_trajs_debug = candidate_trajs_;
  FrenetPath traj_debug;
  while (!candidate_trajs_debug.empty())
  {
    traj_debug = candidate_trajs_debug.top();
    candidate_trajs_debug.pop();

  }

  int traj_number = 0;
  while(!best_traj_found && !candidate_trajs_.empty())
  {
    best_traj_ = candidate_trajs_.top();
    candidate_trajs_.pop();
    traj_number++;
    // Check the constraints
    num_constraint_checks++;
    bool is_safe;
    if (!checkConstraints(best_traj_))
    {
      continue;
    }
    else
    {
      //std::cout << "constraint check is passed" << std::endl;
      // Check for collisions
      if (check_collision)
      {
        //std::cout << "let's compute risk" << std::endl;
        is_safe = checkCollisions(best_traj_, obstacles, use_async, num_collision_checks, curr_trajectory, risk_planned_traj_client);
        /*
        if (!is_safe)
        {
          std::cout << "The trajectory is not safe" << std::endl;
        }
        */
      }
      else
      {
        std::cout << "Collision Checking Skipped" << std::endl;
        is_safe = true;
      }
    }
    
    if (is_safe)
    {
      // added to avoid mis-calculation of r_x that results in a far starting point from current robot's state
      // if (abs(r_x - best_traj_.r_x[0]) < 0.1)
      {
        best_traj_found = true;
        std::cout << "the number of best traj is: " << traj_number << std::endl;
      }
      if (settings_.enable_debug)
      {
        std::cout << "best traj starting x-position: " << best_traj_.x[0] << std::endl;
        std::cout << "best traj starting y-position: " << best_traj_.y[0] << std::endl;
        std::cout << "best traj starting s-position: " << best_traj_.s[0] << std::endl;
        std::cout << "best traj starting d-position: " << best_traj_.d[0] << std::endl;
        std::cout << "best traj starting r_x: " << best_traj_.r_x[0] << std::endl;
        std::cout << "best traj starting r_y: " << best_traj_.r_y[0] << std::endl;
        std::cout << "best traj starting r_yaw: " << best_traj_.r_yaw[0] << std::endl;
        std::cout << "best traj shifted: " << best_traj_.shifted << std::endl;
      }




      /*
      std::cout << "best trajectory found" << std::endl;
      std::cout << "traj. lateral cost: " << best_traj_.lat_cost << std::endl;
      std::cout << "traj. speed cost: " << best_traj_.speed_cost << std::endl;
      std::cout << "traj. time cost: " << best_traj_.time_cost << std::endl;
      std::cout << "traj. fix cost: " << best_traj_.fix_cost << std::endl;
      std::cout << "traj. dyn cost: " << best_traj_.dyn_cost << std::endl;
      std::cout << "traj. final cost: " << best_traj_.final_cost << std::endl;
      */
      break;
    }
  }

  double fix_cost = 0.0;
  double dyn_cost = 0.0;
  double dist = 0.0;
  if (best_traj_found && prev_best_traj_.collision_passed) // ensure the previous best exists
  {
    fix_cost = best_traj_.fix_cost;
    dyn_cost = best_traj_.dyn_cost;
    for (int i = 0; i < 3; i++)
    {
      const double l = best_traj_.idx(i) - prev_best_idx_(i);
      dist += std::pow(l, 2);
    }
  }

  numbers.push_back(num_constraint_checks);
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());
  numbers.push_back(num_collision_checks);
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());
  test_result_.updateCount(std::move(numbers), std::move(timestamps), fix_cost, dyn_cost, dist);
  test_result_.printSummary();

  if (best_traj_found)
  {
    prev_best_traj_ = best_traj_;
    prev_best_idx_ = best_traj_.idx;
    
    if (settings_.enable_debug)
    {
      std::cout << "start position of traj. of the robot: " << best_traj_.x[0] << std::endl;
    }
    
    return std::vector<FrenetPath>{1, best_traj_};
  }
  else
  {
    ROS_ERROR("Local Planner: No valid trajectory is obtained");
    return std::vector<FrenetPath>{};
  }
}

std::vector<fop::FrenetPath> FrenetOptimalTrajectoryPlanner::generateFrenetPaths(const fop::FrenetState& frenet_state, const int lane_id,
                                                                                 const double left_bound, const double right_bound, const double current_speed)
{
  // list of frenet paths generated
  std::vector<fop::FrenetPath> frenet_trajs;
  std::vector<double> goal_ds;

  int idx_i = 0;
  // generate different goals with a lateral offset
  const double delta_width = (left_bound - settings_.center_offset)/((settings_.num_width - 1)/2);
  for (double d = right_bound; d <= left_bound; d += delta_width)  // left being positive
  {
    int idx_k = 0;

    // calculate lateral offset cost
    const double lat_norm = std::max(std::pow(left_bound - settings_.center_offset, 2), std::pow(right_bound - settings_.center_offset, 2));
    const double lat_cost = settings_.k_diff*std::pow(d - settings_.center_offset, 2)/lat_norm;

    // generate d_t polynomials
    const double delta_t = (settings_.max_t - settings_.min_t)/(settings_.num_t - 1);
    for (double T = settings_.min_t; T <= settings_.max_t; T += delta_t)
    {
      int idx_j = 0;

      // calculate time cost (encourage longer planning horizon)
      const double time_cost = settings_.k_time*(1 - (T - settings_.min_t)/(settings_.max_t - settings_.min_t));

      fop::FrenetPath frenet_traj = fop::FrenetPath();
      frenet_traj.lane_id = lane_id;

      // start lateral state [d, d_d, d_dd]
      std::vector<double> start_d;
      start_d.emplace_back(frenet_state.d);
      start_d.emplace_back(frenet_state.d_d);
      start_d.emplace_back(frenet_state.d_dd);

      // end lateral state [d, d_d, d_dd]
      std::vector<double> end_d;
      end_d.emplace_back(d);
      end_d.emplace_back(0.0);
      end_d.emplace_back(0.0);

      // generate lateral quintic polynomial
      fop::QuinticPolynomial lateral_quintic_poly = fop::QuinticPolynomial(start_d, end_d, T);

      // store the this lateral trajectory into frenet_traj
      for (double t = 0.0; t <= T; t += settings_.tick_t)
      {
        frenet_traj.t.emplace_back(t);
        frenet_traj.d.emplace_back(lateral_quintic_poly.calculatePoint(t));
        frenet_traj.d_d.emplace_back(lateral_quintic_poly.calculateFirstDerivative(t));
        frenet_traj.d_dd.emplace_back(lateral_quintic_poly.calculateSecondDerivative(t));
        frenet_traj.d_ddd.emplace_back(lateral_quintic_poly.calculateThirdDerivative(t));
      }

      // generate longitudinal quartic polynomial
      const double delta_speed = (settings_.highest_speed - settings_.lowest_speed)/(settings_.num_speed - 1);
      for (double sample_speed = settings_.lowest_speed; sample_speed <= settings_.highest_speed; sample_speed += delta_speed)
      {
        if (sample_speed <= 0)  // ensure target speed is positive
        {
          continue;
        }

        // copy the longitudinal path over
        fop::FrenetPath target_frenet_traj = frenet_traj;

        // start longitudinal state [s, s_d, s_dd]
        std::vector<double> start_s;
        start_s.emplace_back(frenet_state.s);
        start_s.emplace_back(frenet_state.s_d);
        start_s.emplace_back(0.0);

        // end longitudinal state [s_d, s_dd]
        std::vector<double> end_s;
        end_s.emplace_back(sample_speed);
        end_s.emplace_back(0.0);

        // generate longitudinal quartic polynomial
        fop::QuarticPolynomial longitudinal_quartic_poly = fop::QuarticPolynomial(start_s, end_s, T);

        // store the this longitudinal trajectory into target_frenet_traj
        for (double t = 0.0; t <= T; t += settings_.tick_t)
        {
          target_frenet_traj.s.emplace_back(longitudinal_quartic_poly.calculatePoint(t));
          target_frenet_traj.s_d.emplace_back(longitudinal_quartic_poly.calculateFirstDerivative(t));
          target_frenet_traj.s_dd.emplace_back(longitudinal_quartic_poly.calculateSecondDerivative(t));
          target_frenet_traj.s_ddd.emplace_back(longitudinal_quartic_poly.calculateThirdDerivative(t));
        }

        // calculate speed cost
        const double speed_cost = settings_.k_diff*pow((settings_.highest_speed - sample_speed)/settings_.highest_speed, 2);
        // fixed cost terms
        target_frenet_traj.fix_cost = settings_.k_lat * lat_cost 
                                    + settings_.k_lon * (time_cost + speed_cost);
        target_frenet_traj.lat_cost = lat_cost;
        target_frenet_traj.speed_cost = speed_cost;
        target_frenet_traj.time_cost  = time_cost;

        target_frenet_traj.idx = Eigen::Vector3i(idx_i, idx_j, idx_k);
        frenet_trajs.emplace_back(target_frenet_traj);

        idx_j++;
      }

      idx_k++;
    }

    idx_i++;
  }

  return frenet_trajs;
}

int FrenetOptimalTrajectoryPlanner::calculateGlobalPaths(std::vector<fop::FrenetPath>& frenet_traj_list, fop::Spline2D& cubic_spline, double r_x)
{
  int num_checks = 0;
  for (int i = 0; i < frenet_traj_list.size(); i++)
  {
    bool shifted = false;
    fop::VehicleState compensate_state = cubic_spline.calculatePosition(frenet_traj_list[i].s[0]);
    if (abs(r_x - compensate_state.x) > 1.0)
    {
      // this means the planned trajectory is shifted (dirty fix)
      shifted = true;
    }

    // calculate global positions
    for (int j = 0; j < frenet_traj_list[i].s.size(); j++)
    {
      fop::VehicleState state = cubic_spline.calculatePosition(frenet_traj_list[i].s[j]);
      if (shifted)
      {
        // 3 is the seperation distance between two path points. Needs to be determined through a yaml file 
        state.x = state.x - 3;
        frenet_traj_list[i].shifted = true;
      }
      else
      {
        frenet_traj_list[i].shifted = false;
      }

      double i_yaw = cubic_spline.calculateYaw(frenet_traj_list[i].s[j]);
      const double di = frenet_traj_list[i].d[j];
      const double frenet_x = state.x + di * cos(i_yaw + M_PI / 2.0);
      const double frenet_y = state.y + di * sin(i_yaw + M_PI / 2.0);
      if (!fop::isLegal(frenet_x) || !fop::isLegal(frenet_y))
      {
        break;
      }
      else
      {
        frenet_traj_list[i].x.emplace_back(frenet_x);
        frenet_traj_list[i].y.emplace_back(frenet_y);
        frenet_traj_list[i].r_x.emplace_back(state.x);
        frenet_traj_list[i].r_y.emplace_back(state.y);
        frenet_traj_list[i].r_yaw.emplace_back(i_yaw);
      }
    }
    // calculate yaw and ds
    for (int j = 0; j < frenet_traj_list[i].x.size() - 1; j++)
    {
      const double dx = frenet_traj_list[i].x[j+1] - frenet_traj_list[i].x[j];
      const double dy = frenet_traj_list[i].y[j+1] - frenet_traj_list[i].y[j];
      frenet_traj_list[i].yaw.emplace_back(atan2(dy, dx));
      frenet_traj_list[i].ds.emplace_back(sqrt(dx * dx + dy * dy));
    }

    frenet_traj_list[i].yaw.emplace_back(frenet_traj_list[i].yaw.back());
    frenet_traj_list[i].ds.emplace_back(frenet_traj_list[i].ds.back());

    // calculate curvature
    for (int j = 0; j < frenet_traj_list[i].yaw.size() - 1; j++)
    {
      double yaw_diff = fop::unifyAngleRange(frenet_traj_list[i].yaw[j+1] - frenet_traj_list[i].yaw[j]);
      frenet_traj_list[i].c.emplace_back(yaw_diff / frenet_traj_list[i].ds[j]);
    }

    num_checks++;
  }

  return num_checks;
}

/**
 * @brief Checks whether frenet paths are safe to execute based on constraints 
 * @param traj the trajectory to be checked
 * @return true if trajectory satisfies constraints
 */
bool FrenetOptimalTrajectoryPlanner::checkConstraints(FrenetPath& traj)
{
  bool passed = true;
  /*
  for (int i = 0; i < traj.c.size(); i++)
  {
    // commented by Khaled

    if (!std::isnormal(traj.x[i]) || !std::isnormal(traj.y[i]))
    {
      passed = false;
      std::cout << "Condition 0: Contains ilegal values" << traj.x[i] << std::endl;
      break;
    }
    
    if (traj.s_d[i] > settings_.max_speed)
    {
      passed = false;
      //std::cout << "Condition 1: Exceeded Max Speed" << std::endl;
      break;
    }
    else if (traj.s_dd[i] > settings_.max_accel)
    {
      passed = false;
      //std::cout << "Condition 2: Exceeded Max Acceleration" << std::endl;
      break;
    }
    else if (traj.s_dd[i] < settings_.max_decel)
    {
      passed = false;
      //std::cout << "Condition 3: Exceeded Max Deceleration" << std::endl;
      break;
    }
    // else if (std::abs(traj.c[i]) > settings_.max_curvature)
    // {
    //   passed = false;
    //   std::cout << "Condition 4: Exceeded max curvature = " << settings_.max_curvature
    //             << ". Curr curvature = " << (traj.c[i]) << std::endl;
    //   break;
    // }

  }
  */
  traj.constraint_passed = passed;
  return passed;
}


int FrenetOptimalTrajectoryPlanner::computeCosts(std::vector<fop::FrenetPath>& frenet_trajs, const double curr_speed)
{
  int num_checks = 0;
  for (auto& traj : frenet_trajs)
  {
    // calculate jerk costs
    double jerk_s, jerk_d = 0.0;
    double jerk_sqr_s, jerk_sqr_d = 0.0;
    for (int i = 0; i < traj.t.size(); i++)
    {
      // calculate total squared jerks
      jerk_sqr_s += std::pow(traj.s_ddd.back()/settings_.max_jerk_s, 2);
      jerk_s += std::abs(traj.s_ddd.back()/settings_.max_jerk_s);
      jerk_sqr_d += std::pow(traj.d_ddd.back()/settings_.max_jerk_d, 2);
      jerk_d += std::abs(traj.d_ddd.back()/settings_.max_jerk_d);
    }

    const double jerk_cost_s = jerk_sqr_s/jerk_s;
    const double jerk_cost_d = jerk_sqr_d/jerk_d;
    
    traj.dyn_cost = settings_.k_jerk * (settings_.k_lon * jerk_cost_s + settings_.k_lat * jerk_cost_d);
    traj.final_cost = traj.fix_cost + traj.dyn_cost;

    

    num_checks++;

    candidate_trajs_.push(traj);
  }

  return num_checks;
}


bool FrenetOptimalTrajectoryPlanner::checkCollisions(FrenetPath& ego_traj, const lmpcc_msgs::obstacle_array& obstacle_trajs, 
                                                     const bool use_async, int& num_checks, fop::Path& curr_trajectory, ros::ServiceClient risk_planned_traj_client)
{
  /*
  if (use_async)
  {
  
    std::future<std::pair<bool, int>> collision_check = std::async(std::launch::async, &FrenetOptimalTrajectoryPlanner::checkTrajCollision, this, 
                                                                   ego_traj, obstacle_trajs, settings_.safety_margin_lon, settings_.safety_margin_lat, curr_trajectory);
    const auto result = collision_check.get();
    ego_traj.collision_passed = result.first;
    num_checks += result.second;
   
  }
  else
  */
  {
    const auto result = checkTrajCollision(ego_traj, obstacle_trajs, settings_.safety_margin_lon, settings_.safety_margin_lat, curr_trajectory);
    ego_traj.collision_passed = result.first;
    num_checks += result.second;
  }

  return ego_traj.collision_passed;
}


/**
 * @brief Check for collisions at each point along a frenet path
 *
 * @param ego_traj the path to check
 * @param obstacles obstacles to check against for collision
 * @param margin collision margin in [m]
 * @return false if there is a collision along the path. Otherwise, true
 */
std::pair<bool, int> FrenetOptimalTrajectoryPlanner::checkTrajCollision(const FrenetPath& ego_traj, const lmpcc_msgs::obstacle_array& obstacles,
                                                                        const double margin_lon, const double margin_lat, fop::Path& curr_traj)
{
  int num_checks = 0;
  geometry_msgs::Polygon ego_rect, obstacle_rect;

  // fop::Path planned_traj;
  fop::Path planned_traj = curr_traj;
  // for (int i = 0; i < ego_traj.x.size(); i++)
  
  
  for (int i = 0; i < (ego_traj.x.size() - curr_traj.x.size()); i++)
  {
    planned_traj.x.push_back(ego_traj.x[i]);
    planned_traj.y.push_back(ego_traj.y[i]);
    planned_traj.yaw.push_back(ego_traj.yaw[i]);
  }
  
  for (int i = 0; i < obstacles.obstacles.size(); i++)
  {
    const int num_steps = planned_traj.x.size();

    // std::cout << "number of trajectory steps is: " << num_steps << std::endl;
    // Check for collisions between ego and obstacle trajectories
    for (int j = 0; j < num_steps; j++)
    {
      num_checks++;
      const double vehicle_center_x = planned_traj.x[j]; // + Vehicle::Lr() * cos(ego_traj.yaw[j]);
      const double vehicle_center_y = planned_traj.y[j]; // + Vehicle::Lr() * sin(ego_traj.yaw[j]);
      
      // use Jackal radius instead of vehicle's width and length
      double robot_radius    = 0.325;
      double obstacle_radius = settings_.obstacle_radius;
      double safety_distance = 0.0;

      double obstacle_x_pos;
      double obstacle_y_pos;
      double diff_x;
      double diff_y;
      double distance;
      
      // iterate over gaussian modes
      
      for (size_t mode = 0; mode < obstacles.obstacles[i].gaussians.size(); mode++)
      {
        obstacle_x_pos = obstacles.obstacles[i].gaussians[mode].mean.poses[j].pose.position.x;
        obstacle_y_pos = obstacles.obstacles[i].gaussians[mode].mean.poses[j].pose.position.y;
        diff_x = std::abs(obstacle_x_pos - vehicle_center_x);
        diff_y = std::abs(obstacle_y_pos - vehicle_center_y);
        distance = std::sqrt(std::pow(diff_x, 2) + std::pow(diff_y, 2));

        // std::cout << "distance between robot and obstacle is: " << distance << std::endl;
        if (distance < (robot_radius + obstacle_radius + safety_distance))
        {
          return std::pair<bool, int>{false, num_checks};

        }

      }
      
    }
  }
  
  /*
  // call the service where we calculate collision probability 
  risk_srv_.request.pred_traj.resize(planned_traj.x.size());
  risk_srv_.request.obstacles = obstacles.obstacles;
      
  for (size_t i = 0; i < planned_traj.x.size(); i++)
  {
    risk_srv_.request.pred_traj[i].pose.position.x = planned_traj.x[i];
    risk_srv_.request.pred_traj[i].pose.position.y = planned_traj.y[i];
    risk_srv_.request.pred_traj[i].pose.orientation.z = planned_traj.yaw[i];
  }
            
  if (risk_planned_traj_client_.call(risk_srv_))
  {
    std_msgs::Float64 msg;
    msg.data = risk_srv_.response.estimated_risk;

    if (msg.data <= 0.05)
    {
      return std::pair<bool, int>{true, num_checks};
    }
    else
    {
      return std::pair<bool, int>{false, num_checks};
    }
    // std::cout << "the computed risk is: " << msg.data << std::endl;
    // pred_traj_risk_pub_.publish(msg);
    // estimated_risk_vector_.push_back(risk_srv_.response.estimated_risk);
  }
  else
  {
    ROS_ERROR("Failed to call service planned_traj_risk");
  }
  */
  return std::pair<bool, int>{true, num_checks};
}

}  // namespace fop