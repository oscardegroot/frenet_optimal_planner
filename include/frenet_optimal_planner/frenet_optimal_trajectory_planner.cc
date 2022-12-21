#include "frenet_optimal_trajectory_planner.h"

namespace fop
{

FrenetOptimalTrajectoryPlanner::TestResult::TestResult() : count(0), total_fix_cost(0.0), total_dyn_cost(0.0), total_dist(0.0)
{
  this->numbers = std::vector<int>(4, int(0));
  this->numbers_min = std::vector<int>(4, int(100000));
  this->numbers_max = std::vector<int>(4, int(0));
  this->total_numbers = std::vector<int>(4, int(0));

  this->time = std::vector<double>(5, double(0));
  this->time_min = std::vector<double>(5, double(100000));
  this->time_max = std::vector<double>(5, double(0));
  this->total_time = std::vector<double>(5, double(0));

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
  std::cout << "the length is: " << this->length << std::endl;
  std::cout << "the numbers size is: " << numbers.size() << std::endl;
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
  stats.push_back(this->time[4]);
  stats.push_back(std::round(1000/this->time[4]));
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
  */

 // Print Summary for Worst Case performance
  std::cout << " " << std::endl;
  std::cout << "Summary: Worst Case Performance (" << count << " iterations so far)" << std::endl;
  std::cout << "Step 1 : Generated               " << this->numbers_max[0] << " End States   in " << this->time_max[0] << " ms" << std::endl;
  std::cout << "Step 2 : Generated & Evaluated   " << this->numbers_max[1] << " Trajectories in " << this->time_max[1] << " ms" << std::endl;
  std::cout << "Step 3 : Validated               " << this->numbers_max[2] << " Trajectories in " << this->time_max[2] << " ms" << std::endl;
  std::cout << "Total  : Planning Took           " << this->time_max[4] << " ms (or " << 1000/this->time_max[4] << " Hz)" << std::endl;
  // Print Summary for average performance
  std::cout << " " << std::endl;
  std::cout << "Summary: Average Performance (" << this->count << " planning cycles so far)" << std::endl;
  std::cout << "Step 1 : Generated               " << this->total_numbers[0]/this->count << " End States in " << this->total_time[0]/this->count << " ms" << std::endl;
  std::cout << "Step 2 : Generated & Evaluated   " << this->total_numbers[1]/this->count << " Trajectories in " << this->total_time[1]/this->count << " ms" << std::endl;
  std::cout << "Step 3 : Validated               " << this->total_numbers[2]/this->count << " Trajectories in " << this->total_time[2]/this->count << " ms" << std::endl;
  std::cout << "Total  : Planning Took           " << this->total_time[4]/this->count << " ms (or " << 1000/(this->total_time[4]/this->count) << " Hz)" << std::endl;
  std::cout << "Cost   : Optimal's Fix Cost      " << this->total_fix_cost/count << std::endl;
  std::cout << "Cost   : Optimal's Dyn Cost      " << this->total_dyn_cost/count << std::endl;
  std::cout << "Cost   : Optimal's Total Cost    " << (this->total_fix_cost + this->total_dyn_cost)/count << std::endl;
  std::cout << "Dist   : Distance to History Best" << this->total_dist/count << std::endl;
  
}

FrenetOptimalTrajectoryPlanner::FrenetOptimalTrajectoryPlanner()
{
  this->settings_ = Setting();
  this->test_result_ = TestResult(4);
}

FrenetOptimalTrajectoryPlanner::FrenetOptimalTrajectoryPlanner(FrenetOptimalTrajectoryPlanner::Setting& settings)
{
  this->settings_ = settings;
  this->test_result_ = TestResult(4);
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

std::pair<std::vector<fop::FrenetPath>, std::vector<double>> 
FrenetOptimalTrajectoryPlanner::frenetOptimalPlanning(fop::Spline2D& cubic_spline, const fop::FrenetState& start_state, const int lane_id,
                                                      const double left_width, const double right_width, const double current_speed, const bool check_collision, const bool use_async,
                                                      const lmpcc_msgs::obstacle_array& obstacles, const bool use_heuristic, fop::Path& curr_traj, double r_x)
{
  // Initialize a series of results to be recorded
  std::vector<int> numbers;
  std::vector<std::chrono::_V2::system_clock::time_point> timestamps;
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  /* --------------------------------- Construction Zone -------------------------------- */
  // Clear the canidate trajectories from the last planning cycle
  
  std::priority_queue<fop::FrenetPath, std::vector<fop::FrenetPath>, std::greater<std::vector<fop::FrenetPath>::value_type>> empty;
  std::swap(candidate_trajs_, empty);
  all_trajs_.clear();

  // Initialize start state
  start_state_ = start_state;

  // Sample all the end states in 3 dimensions, [d, v, t] and form the 3d traj candidate array
  //std::cout << "sample end states" << std::endl;
  auto trajs_3d = sampleEndStates(lane_id, left_width, right_width, current_speed, use_heuristic);
  //std::cout << "end states were sampled" << std::endl;
  numbers.emplace_back(trajs_3d.size()*trajs_3d[0].size()*trajs_3d[0][0].size());
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());

  int num_iter = 0;
  int num_trajs_generated = 0;
  int num_trajs_validated = 0;
  int num_collision_checks = 0;

  // Start the iterative search 
  Eigen::Vector3i best_idx;
  bool best_traj_found = false;  

  //std::cout << "let's enter the main while loop" << std::endl;
  while(!best_traj_found)
  {
    //std::cout << "number of candidate trajectories is: " << candidate_trajs_.size() << std::endl;
    // ################################ Initial Guess #####################################
    if (candidate_trajs_.empty())
    {
      if (!findInitGuess(trajs_3d, best_idx))
      {
        // all samples have been searched and no feasible candidate found
        // std::cout << "fop: Searched through all trajectories, found no suitable candidate" << std::endl;
        ROS_ERROR("fop: Searched through all trajectories, found no suitable candidate");
        break;
      }
    }
    else
    {
      best_idx = candidate_trajs_.top().idx;
    }

    // ################################ Search Process #####################################
    bool converged = false;
    while (!converged)
    {
      // std::cout << "fop: Search iteration " << num_iter << " convergence: " << converged << std::endl;
      // std::cout << "fop: Current idx " << best_idx(0) << best_idx(1) << best_idx(2) << std::endl;

      // Perform a search for the real best trajectory using gradients
      converged = findNextBest(trajs_3d, best_idx, num_trajs_generated);
      num_iter++;
    }
    numbers.emplace_back(num_trajs_generated);
    timestamps.emplace_back(std::chrono::high_resolution_clock::now());

    // ################################ Validation Process #####################################
    // std::cout << "fop: Validating Candiate Trajectory" << std::endl;

    if (!candidate_trajs_.empty())
    {
      auto candidate_traj = candidate_trajs_.top();
      //std::cout << "index of current safety-checked trajectory is: " << candidate_traj.idx(0) << candidate_traj.idx(1) << candidate_traj.idx(2) << std::endl;
      candidate_trajs_.pop();
      num_trajs_validated++;
      
      // Convert to the global frame
      convertToGlobalFrame(candidate_traj, cubic_spline, r_x);
      // Check for constraints
      bool is_safe = checkConstraints(candidate_traj);
      if (!is_safe)
      {
        std::cout << "failed constraints check" << std::endl;
        continue;
      }
      else
      {
        // Check for collisions
        if (check_collision)
        {
          is_safe = checkCollisions(candidate_traj, obstacles, use_async, num_collision_checks, curr_traj);
        }
        else
        {
          std::cout << "Collision Checking Skipped" << std::endl;
          is_safe = true;
        }
      }

      if (is_safe)
      {
        best_traj_found = true;
        best_traj_ = std::move(candidate_traj);
        break;
      }
    }
  }

  double fix_cost = 0.0;
  double dyn_cost = 0.0;
  int dist = 0.0;
  std::cout << "number of generated trajectories is: " << num_trajs_generated << std::endl;
  std::cout << "number of validated trajectories is: " << num_trajs_validated << std::endl;
  if (best_traj_found && prev_best_traj_.is_generated) // ensure the previous best exists
  {
    fix_cost = best_traj_.fix_cost;
    dyn_cost = best_traj_.dyn_cost;
    for (int i = 0; i < 3; i++)
    {
      const int l = std::abs(prev_best_traj_.idx(i) - best_traj_.idx(i));
      if (l <= 100)
      {
        dist += std::pow(l, 2);
      }
    }
  }

  std::cout << "fop: Search Done in " << num_iter << " iterations" << std::endl;
  numbers.emplace_back(num_trajs_validated);
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());
  numbers.emplace_back(num_collision_checks);
  timestamps.emplace_back(std::chrono::high_resolution_clock::now());
  const auto stats = test_result_.updateCount(numbers, timestamps, fix_cost, dyn_cost, dist);
  test_result_.printSummary();
  /* --------------------------------- Construction Zone -------------------------------- */
  
  // Convert the other unused candiates as well (for visualization only)
  
  for (auto& traj : all_trajs_)
  {
    convertToGlobalFrame(traj, cubic_spline, r_x);
  }

  std::vector<FrenetPath> result;
  if (best_traj_found)
  {
    prev_best_traj_ = best_traj_;
    result.emplace_back(best_traj_);
  }

  return std::pair<std::vector<FrenetPath>, std::vector<double>>{result, stats};
}


/**
 * @brief Checks whether frenet paths are safe to execute based on constraints 
 * @param trajs the 3D Cube of trajectories
 * @param idx the index of the lowest estimated cost trajectory
 * @return true if there are still trajs not searched before, false otherwise
 */
bool FrenetOptimalTrajectoryPlanner::findInitGuess(const std::vector<std::vector<std::vector<fop::FrenetPath>>>& trajs, Eigen::Vector3i& idx)
{
  double min_cost = std::numeric_limits<double>::max();
  bool found = false;

  // find the index of the traj with the lowest estimated cost
  for (int i = 0; i < trajs.size(); i++)
  {
    for (int j = 0; j < trajs[0].size(); j++)
    {
      for (int k = 0; k < trajs[0][0].size(); k++)
      {  
        if (!trajs[i][j][k].is_searched && trajs[i][j][k].est_cost < min_cost)
        //if (trajs[i][j][k].est_cost < min_cost)
        {
          min_cost = trajs[i][j][k].est_cost;
          idx(0) = i;
          idx(1) = j;
          idx(2) = k;
          found = true;
        }
      }
    }
  }

  return found;
}

bool FrenetOptimalTrajectoryPlanner::findNextBest(std::vector<std::vector<std::vector<FrenetPath>>>& trajs, Eigen::Vector3i& idx, int& num_traj)
{
  if (trajs[idx(0)][idx(1)][idx(2)].is_searched)
  {
    //std::cout << "FNB: At Current idx " << idx(0) << idx(1) << idx(2) << " converged" << std::endl;
    return true; // converged
  }
  else
  {
    // std::cout << "FNB: At Current idx " << idx(0) << idx(1) << idx(2) << " not converged" << std::endl;
    trajs[idx(0)][idx(1)][idx(2)].is_searched = true; // label this traj as searched
    const auto gradients = findGradients(trajs, idx, num_traj);

    int grad_dim = 0;
    double max_grad = gradients(0);
    for (int i = 1; i < 3; i++)
    {
      if (std::abs(gradients(i)) > std::abs(max_grad))
      {
        grad_dim = i;
        max_grad = gradients(i);
      }
    }

    idx(grad_dim) += max_grad > 0? -1 : 1; // move in the max gradient direction, towards lower cost
    
    return false; // not converged
  }
}

Eigen::Vector3d FrenetOptimalTrajectoryPlanner::findGradients(std::vector<std::vector<std::vector<fop::FrenetPath>>>& trajs, const Eigen::Vector3i& idx, int& num_traj)
{
  const Eigen::Vector3i sizes = {int(trajs.size()), int(trajs[0].size()), int(trajs[0][0].size())};
  const Eigen::Vector3i directions = findDirection(sizes, idx);

  // Center sample location which we want to find the gradient
  const double cost_center = getTrajAndRealCost(trajs, idx, num_traj);

  // Compute the gradients at each direction
  Eigen::Vector3d gradients;
  for (int dim = 0; dim < 3; dim++)
  {
    Eigen::Vector3i next_idx = idx;
    next_idx(dim) += directions(dim);
    if (directions(dim) >= 0) // the right side has neighbor
    {
      gradients(dim) = getTrajAndRealCost(trajs, next_idx, num_traj) - cost_center;
      if (gradients(dim) >= 0 && idx(dim) == 0) // the right neighbor has higher cost and there is no neighbor on the left side
      {
        gradients(dim) = 0.0; // set the gradient to zero
      }
    }
    else // the right side has no neighbor, calculate gradient using the left neighbor
    {
      gradients(dim) = cost_center - getTrajAndRealCost(trajs, next_idx, num_traj);
      if (gradients(dim) <= 0 && idx(dim) == sizes(dim)-1) // the left neighbor has higher cost and there is no neighbor on the right side
      {
        gradients(dim) = 0.0; // set the gradient to zero
      }
    }
  }

  return gradients;
}

Eigen::Vector3i FrenetOptimalTrajectoryPlanner::findDirection(const Eigen::Vector3i& sizes, const Eigen::Vector3i& idx)
{
  Eigen::Vector3i directions;
  for (int dim = 0; dim < 3; dim++)
  {
    if (idx(dim) >= sizes(dim)-1)
    {
      directions(dim) = -1;
    }
    else
    {
      directions(dim) = +1;
    }
  }

  return directions;
}

double FrenetOptimalTrajectoryPlanner::getTrajAndRealCost(std::vector<std::vector<std::vector<FrenetPath>>>& trajs, const Eigen::Vector3i& idx, int& num_traj)
{
  const int i = idx(0);  // width dimension
  const int j = idx(1);  // speed dimension
  const int k = idx(2);  // time  dimension
  
  if (trajs[i][j][k].is_generated)
  {
    return trajs[i][j][k].final_cost;
  }
  else
  {
    num_traj++;
    trajs[i][j][k].is_generated = true;
    trajs[i][j][k].idx = idx;
    
    // calculate the costs
    double jerk_s, jerk_d = 0.0;
    double jerk_sqr_s, jerk_sqr_d = 0.0;

    // generate lateral quintic polynomial
    QuinticPolynomial lateral_quintic_poly = QuinticPolynomial(start_state_, trajs[i][j][k].end_state);

    // store the this lateral trajectory into traj
    for (double t = 0.0; t <= trajs[i][j][k].end_state.T; t += settings_.tick_t)
    {
      trajs[i][j][k].t.emplace_back(t);
      trajs[i][j][k].d.emplace_back(lateral_quintic_poly.calculatePoint(t));
      trajs[i][j][k].d_d.emplace_back(lateral_quintic_poly.calculateFirstDerivative(t));
      trajs[i][j][k].d_dd.emplace_back(lateral_quintic_poly.calculateSecondDerivative(t));
      trajs[i][j][k].d_ddd.emplace_back(lateral_quintic_poly.calculateThirdDerivative(t));
      jerk_sqr_d += std::pow(trajs[i][j][k].d_ddd.back()/settings_.max_jerk_d, 2);
      jerk_d += std::abs(trajs[i][j][k].d_ddd.back()/settings_.max_jerk_d);
    }

    // generate longitudinal quartic polynomial
    QuarticPolynomial longitudinal_quartic_poly = QuarticPolynomial(start_state_, trajs[i][j][k].end_state);

    // store the this longitudinal trajectory into traj
    for (double t = 0.0; t <= trajs[i][j][k].end_state.T; t += settings_.tick_t)
    {
      trajs[i][j][k].s.emplace_back(longitudinal_quartic_poly.calculatePoint(t));
      trajs[i][j][k].s_d.emplace_back(longitudinal_quartic_poly.calculateFirstDerivative(t));
      trajs[i][j][k].s_dd.emplace_back(longitudinal_quartic_poly.calculateSecondDerivative(t));
      trajs[i][j][k].s_ddd.emplace_back(longitudinal_quartic_poly.calculateThirdDerivative(t));
      jerk_sqr_s += std::pow(trajs[i][j][k].s_ddd.back()/settings_.max_jerk_s, 2);
      jerk_s += std::abs(trajs[i][j][k].s_ddd.back()/settings_.max_jerk_s);
    }

    const double jerk_cost_s = jerk_sqr_s/jerk_s;
    const double jerk_cost_d = jerk_sqr_d/jerk_d;
    
    trajs[i][j][k].dyn_cost = settings_.k_jerk * (settings_.k_lon * jerk_cost_s + settings_.k_lat * jerk_cost_d);
    trajs[i][j][k].final_cost = trajs[i][j][k].fix_cost + trajs[i][j][k].dyn_cost;

    // Add this trajectory to the candidate queue
    all_trajs_.push_back(trajs[i][j][k]);
    //std::cout << "the added traj. index is: " << i << j << k << std::endl;
    candidate_trajs_.push(trajs[i][j][k]);

    return trajs[i][j][k].final_cost;
  }
}

void FrenetOptimalTrajectoryPlanner::convertToGlobalFrame(FrenetPath& traj, Spline2D& cubic_spline, double r_x)
{
  
  VehicleState compensate_state = cubic_spline.calculatePosition(traj.s[0]);
  // calculate global positions
  for (int j = 0; j < traj.s.size(); j++)
  {
    bool shifted = false;
    if (abs(r_x - compensate_state.x) > 1.0)
    {
      // this means the planned trajectory is shifted (dirty fix)
      shifted = true;
    }
    
    VehicleState state = cubic_spline.calculatePosition(traj.s[j]);
    if (shifted)
    {
      // 3 is the seperation distance between two path points. Needs to be determined through a yaml file.
      state.x = state.x - 3;
    }

    double i_yaw = cubic_spline.calculateYaw(traj.s[j]);
    const double di = traj.d[j];
    const double frenet_x = state.x + di * cos(i_yaw + M_PI / 2.0);
    const double frenet_y = state.y + di * sin(i_yaw + M_PI / 2.0);
    if (!isLegal(frenet_x) || !isLegal(frenet_y))
    {
      break;
    }
    else
    {
      traj.x.emplace_back(frenet_x);
      traj.y.emplace_back(frenet_y);
    }
  }
  // calculate yaw and ds
  for (int j = 0; j < traj.x.size() - 1; j++)
  {
    const double dx = traj.x[j+1] - traj.x[j];
    const double dy = traj.y[j+1] - traj.y[j];
    traj.yaw.emplace_back(atan2(dy, dx));
    traj.ds.emplace_back(sqrt(dx * dx + dy * dy));
  }

  traj.yaw.emplace_back(traj.yaw.back());
  traj.ds.emplace_back(traj.ds.back());

  // calculate curvature
  for (int j = 0; j < traj.yaw.size() - 1; j++)
  {
    double yaw_diff = unifyAngleRange(traj.yaw[j+1] - traj.yaw[j]);
    traj.c.emplace_back(yaw_diff / traj.ds[j]);
  }
}
std::vector<std::vector<std::vector<FrenetPath>>> FrenetOptimalTrajectoryPlanner::sampleEndStates(const int lane_id, const double left_bound, 
                                                                                                  const double right_bound, const double current_speed, 
                                                                                                  const bool use_heuristic)
{
  // list of frenet end states sampled
  std::vector<std::vector<std::vector<FrenetPath>>> trajs_3d;
  
  // Heuristic parameters
  const double max_sqr_dist = std::pow(settings_.num_width, 2) + std::pow(settings_.num_speed, 2) + std::pow(settings_.num_t, 2);
  
  // Sampling on the lateral direction
  const double delta_width = (left_bound - settings_.center_offset)/((settings_.num_width - 1)/2);
  for (int i = 0; i < settings_.num_width; i++)  // left being positive
  {
    std::vector<std::vector<FrenetPath>> trajs_2d;
    const double d = right_bound + i*delta_width;
    const double lat_norm = std::max(std::pow(left_bound - settings_.center_offset, 2), std::pow(right_bound - settings_.center_offset, 2));
    const double lat_cost = settings_.k_diff*std::pow(d - settings_.center_offset, 2)/lat_norm;

    // Sampling on the longitudial direction
    const double delta_v = (settings_.highest_speed - settings_.lowest_speed)/(settings_.num_speed - 1);
    for (int j = 0; j < settings_.num_speed; j++)
    {
      std::vector<FrenetPath> trajs_1d;
      const double v = settings_.lowest_speed + j*delta_v;
      const double speed_cost = settings_.k_diff*pow((settings_.highest_speed - v)/settings_.highest_speed, 2);

      // Sampling on the time dimension
      const double delta_t = (settings_.max_t - settings_.min_t)/(settings_.num_t - 1);
      for (int k = 0; k < settings_.num_t; k++)
      {
        FrenetState end_state;
        // end time
        end_state.T = settings_.min_t + k*delta_t;
        // end longitudial state [s, s_d, s_dd]
        end_state.s = 0.0;  // TBD later by polynomial
        end_state.s_d  = v;
        end_state.s_dd = 0.0;
        // end lateral state [d, d_d, d_dd]
        end_state.d = d;
        end_state.d_d  = 0.0;
        end_state.d_dd = 0.0;

        // Planning Horizon cost (encourage longer planning horizon)
        const double time_cost = settings_.k_time*(1.0 - (end_state.T - settings_.min_t)/(settings_.max_t - settings_.min_t));
        
        // fixed cost terms
        const double fix_cost = settings_.k_lat * lat_cost 
                              + settings_.k_lon * (time_cost + speed_cost);
        
        // estimated heuristic cost terms
        // const double heu_cost = settings_.k_lat * settings_.k_diff * pow(start_state_.d - end_state.d, 2);
        double heu_cost = 0.0;
        if (use_heuristic && prev_best_traj_.is_generated) // Add history heuristic
        {
          const double heu_sqr_dist = std::pow(i - prev_best_traj_.idx(0), 2) + std::pow(j - prev_best_traj_.idx(1), 2) + std::pow(k - prev_best_traj_.idx(2), 2);
          heu_cost = settings_.k_heuristic * heu_sqr_dist/max_sqr_dist;
        }

        trajs_1d.emplace_back(FrenetPath(lane_id, end_state, fix_cost, heu_cost));
      }

      trajs_2d.emplace_back(trajs_1d);
    }

    trajs_3d.emplace_back(trajs_2d);
  }

  return trajs_3d;
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
  for (int i = 0; i < traj.c.size(); i++)
  {
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
                                                     const bool use_async, int& num_checks, fop::Path& curr_trajectory)
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
      double obstacle_radius = 0.3;
      double safety_distance = 0.05;

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
        // prune trajectories that collide with constant velocity model (this saves much computation time)
        if (distance < (robot_radius + obstacle_radius + safety_distance))
        {
          return std::pair<bool, int>{false, num_checks};

        }

      }
      
    }
  }

  // call the service where we calculate collision probability 
  /*
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
      std::cout << "computed best trajectory risk is: " << msg.data << std::endl;
      return std::pair<bool, int>{true, num_checks};
    }
    else
    {
      return std::pair<bool, int>{false, num_checks};
    }
  }
  else
  {
    ROS_ERROR("Failed to call service planned_traj_risk");
  }
  */
  return std::pair<bool, int>{true, num_checks};
}


/*
std::vector<Path> FrenetOptimalTrajectoryPlanner::predictTrajectories(const autoware_msgs::DetectedObjectArray& obstacles)
{
  std::vector<Path> obstacle_trajs;

  for (const auto& obstacle : obstacles.objects)
  {
    Path obstacle_traj;

    obstacle_traj.x.push_back(obstacle.pose.position.x);
    obstacle_traj.y.push_back(obstacle.pose.position.y);
    tf2::Quaternion q_tf2(obstacle.pose.orientation.x, obstacle.pose.orientation.y,
                          obstacle.pose.orientation.z, obstacle.pose.orientation.w);
    tf2::Matrix3x3 m(q_tf2.normalize());
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    obstacle_traj.yaw.push_back(yaw);
    const double v = magnitude(obstacle.velocity.linear.x, obstacle.velocity.linear.y, obstacle.velocity.linear.z);
    obstacle_traj.v.push_back(v);
    
    const int steps = settings_.max_t/settings_.tick_t;
    for (int i = 0; i < steps; i++)
    {
      obstacle_traj.x.push_back(obstacle_traj.x.back() + v*settings_.tick_t*std::cos(yaw));
      obstacle_traj.x.push_back(obstacle_traj.y.back() + v*settings_.tick_t*std::sin(yaw));
      obstacle_traj.yaw.push_back(yaw);
      obstacle_traj.v.push_back(v);
    }

    obstacle_trajs.emplace_back(obstacle_traj);
  }

  return obstacle_trajs;
}
*/
}  // namespace fop