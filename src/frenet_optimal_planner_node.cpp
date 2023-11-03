#include "frenet_optimal_planner/frenet_optimal_planner_node.h"

namespace fop
{

  // FrenetOptimalTrajectoryPlanner settings
  FrenetOptimalTrajectoryPlanner::Setting SETTINGS = FrenetOptimalTrajectoryPlanner::Setting();

  // Constants values used as thresholds (Not for tuning)
  const double WP_MAX_SEP = 3.0;               // Maximum allowable waypoint separation
  const double WP_MIN_SEP = 0.0;               // Minimum allowable waypoint separation
  const double HEADING_DIFF_THRESH = M_PI / 2; // Maximum allowed heading diff between vehicle and path
  const double MAX_DIST_FROM_PATH = 10.0;      // Maximum allowed distance between vehicle and path

  /* List of dynamic parameters */
  // Hyperparameters for output path
  double TRAJ_MAX_SIZE; // Maximum size of the output path
  double TRAJ_MIN_SIZE; // Minimum size of the output path

  // Control Parameters
  double PID_Kp, PID_Ki, PID_Kd;
  // Stanley gains
  int NUM_WP_LOOK_AHEAD;       // Number of waypoints to look ahead for Stanley
  double STANLEY_OVERALL_GAIN; // Stanley overall gain
  double TRACK_ERROR_GAIN;     // Cross track error gain

  // Safety margins for collision check
  double LANE_WIDTH;
  double LEFT_LANE_WIDTH;  // Maximum left road width [m]
  double RIGHT_LANE_WIDTH; // Maximum right road width [m]
  // double NARROW_PATH_OFFSET = -0.3;

  bool CHECK_COLLISION;
  bool USE_ASYNC;
  bool USE_HEURISTIC;

  bool SETTINGS_UPDATED = false;

  // Dynamic parameter server callback function
  void dynamicParamCallback(frenet_optimal_planner::frenet_optimal_planner_Config &config, uint32_t level)
  {
    if (config.reset_world)
    {
      config.reset_world = false;
      SETTINGS.reset_world = true;
    }

    // General Settings
    CHECK_COLLISION = config.check_collision;
    USE_ASYNC = config.use_async;
    USE_HEURISTIC = config.use_heuristic;
    SETTINGS.tick_t = config.tick_t;
    SETTINGS.enable_debug = config.enable_debug;
    SETTINGS.replan_all_time = config.replan_all_time;

    // Sampling parameters (lateral)
    LANE_WIDTH = config.curr_lane_width;
    LEFT_LANE_WIDTH = config.left_lane_width;
    RIGHT_LANE_WIDTH = config.right_lane_width;
    SETTINGS.center_offset = config.center_offset;
    SETTINGS.num_width = config.num_width;
    // Sampling parameters (longitudinal)
    SETTINGS.max_t = config.max_t;
    SETTINGS.min_t = config.min_t;
    SETTINGS.num_t = config.num_t;

    SETTINGS.highest_speed = kph2mps(config.highest_speed);
    SETTINGS.lowest_speed = kph2mps(config.lowest_speed);
    SETTINGS.num_speed = config.num_speed;
    // Constraints
    // SETTINGS.max_speed = Vehicle::max_speed();
    // SETTINGS.max_accel = Vehicle::max_acceleration();
    // SETTINGS.max_decel = Vehicle::max_deceleration();
    // SETTINGS.max_curvature = Vehicle::max_curvature_front();
    // SETTINGS.steering_angle_rate = Vehicle::max_steering_rate();
    SETTINGS.max_speed = kph2mps(config.max_speed);
    SETTINGS.max_accel = config.max_acceleration;
    SETTINGS.max_decel = -config.max_deceleration;
    SETTINGS.max_curvature = config.max_curvature;
    SETTINGS.max_jerk_s = config.max_jerk_lon;
    SETTINGS.max_jerk_d = config.max_jerk_lat;
    // Cost Weights
    SETTINGS.k_heuristic = config.k_heuristic;
    SETTINGS.k_diff = config.k_diff;
    SETTINGS.k_time = config.k_time;
    SETTINGS.k_jerk = config.k_jerk;
    SETTINGS.k_lat = config.k_lat;
    SETTINGS.k_lon = config.k_lon;
    // SETTINGS.k_obstacle = config.k_obstacle;
    // Safety constraints
    SETTINGS.vehicle_length = config.vehicle_length;
    SETTINGS.vehicle_width = config.vehicle_width;
    SETTINGS.safety_margin_lon = config.safety_margin_lon;
    SETTINGS.safety_margin_lat = config.safety_margin_lat;
    SETTINGS.safety_margin_soft = config.safety_margin_soft;
    SETTINGS.obstacle_radius = config.obstacle_radius;
    // PID and Stanley gains
    PID_Kp = config.PID_Kp;
    PID_Ki = config.PID_Ki;
    PID_Kd = config.PID_Kd;
    NUM_WP_LOOK_AHEAD = config.num_wp_look_ahead;
    STANLEY_OVERALL_GAIN = config.stanley_overall_gain;
    TRACK_ERROR_GAIN = config.track_error_gain;
    // Hyperparameters for output path
    TRAJ_MAX_SIZE = config.traj_max_size;
    TRAJ_MIN_SIZE = config.traj_min_size;

    SETTINGS_UPDATED = true;
  }

  // Constructor
  FrenetOptimalPlannerNode::FrenetOptimalPlannerNode() : tf_listener(tf_buffer)
  {
    // topics
    std::string odom_topic;
    std::string lane_info_topic;
    std::string obstacles_topic;

    std::string ref_path_topic;
    std::string ref_path_visual_topic;
    std::string traj_visual_topic;
    std::string curr_traj_topic;
    std::string next_traj_topic;
    std::string sample_space_topic;
    std::string final_traj_topic;
    std::string candidate_trajs_topic;
    std::string vehicle_cmd_topic;

    // // std::cout<< "we are in the constructor" << std::endl;
    // Initialize the configuration (this is to get the way points from the yaml file)
    config_.reset(new frenet_planner_configuration());
    bool config_success = config_->initialize();
    /*
    if (config_success == false)
    {
      LMPCC_ERROR("Failed to initialize!");
      // std::cout << "States: \n"
                << " pd_config: " << std::boolalpha << config_success << "\n"
                << " pd config init success: " << std::boolalpha << config_->initialize_success_
                << std::endl;
      return false;
    }
    */

    // // std::cout << "x-reference path: " << config_->ref_x_.size() << std::endl;
    // feedWaypoints();

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
    f = boost::bind(&dynamicParamCallback, _1, _2);
    server.setCallback(f);

    // Hyperparameters from launch file
    private_nh.getParam("odom_topic", odom_topic);
    private_nh.getParam("lane_info_topic", lane_info_topic);
    private_nh.getParam("obstacles_topic", obstacles_topic);
    private_nh.getParam("curr_traj_topic", curr_traj_topic);
    private_nh.getParam("next_traj_topic", next_traj_topic);
    private_nh.getParam("ref_path_topic", ref_path_topic);
    private_nh.getParam("ref_path_visual_topic", ref_path_visual_topic);
    private_nh.getParam("traj_visual_topic", traj_visual_topic);
    private_nh.getParam("sample_space_topic", sample_space_topic);
    private_nh.getParam("final_traj_topic", final_traj_topic);
    private_nh.getParam("candidate_trajs_topic", candidate_trajs_topic);
    private_nh.getParam("vehicle_cmd_topic", vehicle_cmd_topic);

    // // std::cout << "reference path msg is: " << ref_path_topic << std::endl;

    // Instantiate FrenetOptimalTrajectoryPlanner
    frenet_planner_ = FrenetOptimalTrajectoryPlanner(SETTINGS);

    // Subscribe & Advertise
    odom_sub = nh.subscribe(odom_topic, 1, &FrenetOptimalPlannerNode::odomCallback, this);
    lane_info_sub = nh.subscribe(lane_info_topic, 1, &FrenetOptimalPlannerNode::laneInfoCallback, this);
    // subscribe to obstacles from pedestrian simulator package
    obstacles_sub = nh.subscribe("/pedestrian_simulator/pedestrians", 1, &FrenetOptimalPlannerNode::obstacleCallback, this);
    obstacle_prediction_sub = nh.subscribe("/pedestrian_simulator/trajectory_predictions", 1, &FrenetOptimalPlannerNode::obstacleTrajectoryPredictionsCallback, this);
    collision_sub_ = nh.subscribe("/lmpcc/collision_detected", 1, &FrenetOptimalPlannerNode::CollisionCallback, this);

    // obstacles_sub = nh.subscribe(obstacles_topic, 1, &FrenetOptimalPlannerNode::obstaclesCallback, this);
    ref_path_pub = nh.advertise<nav_msgs::Path>(ref_path_topic, 1);
    curr_traj_pub = nh.advertise<nav_msgs::Path>(curr_traj_topic, 1);
    next_traj_pub = nh.advertise<nav_msgs::Path>(next_traj_topic, 1);
    command_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // visualization topics
    sample_space_pub = nh.advertise<visualization_msgs::Marker>(sample_space_topic, 1);
    // final_traj_pub   = nh.advertise<visualization_msgs::Marker>(traj_visual_topic, 1);
    candidate_paths_pub = nh.advertise<visualization_msgs::MarkerArray>(candidate_trajs_topic, 1);
    ros_markers_reference_path_.reset(new ROSMarkerPublisher(nh, ref_path_visual_topic.c_str(), "map", 30));
    collision_space_markers_.reset(new ROSMarkerPublisher(nh, traj_visual_topic.c_str(), "map", 100));

    // Service clients for resetting
    std::string reset_environment_topic = "/lmpcc/reset_environment";
    reset_simulation_pub_ = nh.advertise<std_msgs::Empty>(reset_environment_topic.c_str(), 1);
    reset_simulation_client_ = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    reset_ekf_client_ = nh.serviceClient<robot_localization::SetPose>("/set_pose");

    risk_planned_traj_client_ = nh.serviceClient<frenet_optimal_planner::ObservedRisk>("planned_traj_risk");

    // visualizes black circles for pedestrians
    obstacle_markers_.reset(new ROSMarkerPublisher(nh, "lmpcc/received_obstacles", "map", 30));
    ros_markers_.reset(new ROSMarkerPublisher(nh, "obstacle_prediction/markers", "map", 500)); // 3500)); // was 1800

    total_exp_time_ = 0.0;

    // we can remove this, and call the main function (processreferencepath) from the obstacles callback function
    timer_ = nh.createTimer(ros::Duration(1.0 / CONTROL_FREQUENCY), &FrenetOptimalPlannerNode::runNode, this);

    // vehicle_cmd_pub = nh.advertise<autoware_msgs::VehicleCmd>(vehicle_cmd_topic, 1);
    // obstacles_pub   = nh.advertise<autoware_msgs::DetectedObjectArray>("local_planner/objects", 1);

    signal_publishers_.emplace_back(nh, "acceleration"); // For JSK visualization
    signal_publishers_.emplace_back(nh, "velocity");     // For JSK visualization
    signal_publishers_.emplace_back(nh, "rot_velocity"); // For JSK visualization

    // Initializing states
    regenerate_flag_ = false;
    target_lane_id_ = LaneID::CURR_LANE;
    pid_ = control::PID(0.1, fop::Vehicle::max_acceleration(), fop::Vehicle::max_deceleration(), PID_Kp, PID_Ki, PID_Kd);
  };

  void FrenetOptimalPlannerNode::laneInfoCallback(const nav_msgs::Path::ConstPtr &global_path)
  {

    lane_ = fop::Lane(global_path, LANE_WIDTH / 2, LANE_WIDTH / 2, LANE_WIDTH / 2 + LEFT_LANE_WIDTH, LANE_WIDTH / 2 + RIGHT_LANE_WIDTH);
    // ROS_INFO("Local Planner: Lane Info Received, with %d points, filtered to %d points", int(lane_.points.size()), int(lane_.points.size()));
  }

  void FrenetOptimalPlannerNode::CollisionCallback(const std_msgs::Float64 &msg)
  {
    intrusion_ = msg.data;
  }

  void FrenetOptimalPlannerNode::visualizeObstaclePrediction(const mpc_msgs::obstacle_array &obstacles_prediction)
  {
    // ROS_INFO("FOP: Visualizing Obstacles predictions");

    ROSPointMarker &prediction_points = ros_markers_->getNewPointMarker("CYLINDER");
    prediction_points.setScale(0.15, 0.15, 0.1e-3);

    ROSPointMarker &prediction_circles = ros_markers_->getNewPointMarker("CYLINDER");
    ROSLine &prediction_trajectories = ros_markers_->getNewLine();

    double visualization_scale = 1.0;
    prediction_trajectories.setScale(0.07 * visualization_scale, 0.07 * visualization_scale);

    Eigen::Vector3d current_location, prev_location;
    double vehicle_radius = 0.325;
    double obstacle_radius = SETTINGS.obstacle_radius;
    // double obstacle_x_pos;
    // double obstacle_y_pos;

    // indices to draw
    // std::vector<int> indices_to_draw{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19 };
    std::vector<int> indices_to_draw{0, 5, 10, 15, 19};

    for (int i = 0; i < obstacles_prediction.obstacles.size(); i++)
    {
      for (size_t mode = 0; mode < obstacles_prediction.obstacles[i].gaussians.size(); mode++)
      {

        std::vector<double> obstacle_x_pos;
        std::vector<double> obstacle_y_pos;

        for (int j = 1; j < obstacles_prediction.obstacles[i].gaussians[mode].mean.poses.size(); j++)
        {
          obstacle_x_pos.push_back(obstacles_prediction.obstacles[i].gaussians[mode].mean.poses[j].pose.position.x);
          obstacle_y_pos.push_back(obstacles_prediction.obstacles[i].gaussians[mode].mean.poses[j].pose.position.y);
        }

        for (size_t k = 0; k < indices_to_draw.size(); k++)
        {
          const int &index = indices_to_draw[k];
          prediction_trajectories.setColorInt(k, (int)indices_to_draw.size());
          current_location = Eigen::Vector3d(obstacle_x_pos[index], obstacle_y_pos[index], -((double)k) * 0.1e-3);

          // Draw a connecting line
          if (k > 0)
          {
            prediction_trajectories.addLine(prev_location, current_location);
          }

          prev_location = current_location;
          prediction_circles.setScale(2 * (obstacle_radius), 2 * (obstacle_radius), 0.1);

          prediction_circles.setColorInt(k, indices_to_draw.size(), 0.4);
          prediction_points.setColorInt(k, indices_to_draw.size(), 1.0);

          prediction_circles.addPointMarker(current_location);
          prediction_points.addPointMarker(current_location);
        }
      }
    }
  }

  // Update vehicle current state from the tf transform
  void FrenetOptimalPlannerNode::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
  {
    // // std::cout << "OdomCallback is here" << std::endl;
    current_state_.v = magnitude(odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y, 0.);
    current_angular_velocity_ = odom_msg->twist.twist.angular.z;
    // geometry_msgs::TransformStamped transform_stamped;
    // try
    // {
    //   transform_stamped = tf_buffer.lookupTransform("map", odom_msg->header.frame_id, ros::Time(0));
    // }
    // catch (tf2::TransformException &ex)
    // {
    //   ROS_WARN("%s", ex.what());
    //   return;
    // }

    // geometry_msgs::Pose pose_in_map;
    // tf2::doTransform(odom_msg->pose.pose, pose_in_map, transform_stamped);
    // Current XY of robot (map frame)
    current_state_.x = odom_msg->pose.pose.position.x;
    current_state_.y = odom_msg->pose.pose.position.y;
    current_state_.yaw = odom_msg->pose.pose.orientation.z;
    state_received_ = true;
    // For the Jackal, this parameter can be ignored
    // map_height_ = odom_msg->pose.pose.position.z - 0.3; // minus the tire radius

    // tf2::Quaternion q_tf2(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
    //                       odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
    // tf2::Matrix3x3 m(q_tf2.normalize());
    // double roll, pitch;
    // m.getRPY(roll, pitch, current_state_.yaw);

    // Plots a Axis to display the path variable location
    // Oscar
    if (t_last_tf_ + ros::Duration(0.01) < ros::Time::now())
    {
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = ros::Time::now();
      t_last_tf_ = transformStamped.header.stamp;
      transformStamped.header.frame_id = "map";
      transformStamped.child_frame_id = "robot_state";

      transformStamped.transform.translation.x = current_state_.x;
      transformStamped.transform.translation.y = 0.; // solver_interface_ptr_->State().y();
      transformStamped.transform.translation.z = 0.0;
      transformStamped.transform.rotation.x = 0;
      transformStamped.transform.rotation.y = 0;
      transformStamped.transform.rotation.z = 0;
      transformStamped.transform.rotation.w = 1;

      path_pose_pub_.sendTransform(transformStamped);
    }

    // // std::cout << "robot's x-position" << current_state_.x << std::endl;
    // // std::cout << "robot's y-position" << current_state_.y << std::endl;
    // // std::cout << "robot's orientation" << current_state_.yaw << std::endl;
  }

  // this is the main function, from which all other required functions are called
  void FrenetOptimalPlannerNode::processreferencepath()
  {

    // Start a timing for the main algorithm
    const auto start_time = std::chrono::high_resolution_clock::now();

    // ROS_INFO("Local Planner: Planning Start");
    //  Update Waypoints
    if (!feedWaypoints())
    {
      ROS_WARN("Local Planner: Waiting for Waypoints");
      publishEmptyTrajsAndStop();

      return;
    }

    if (SETTINGS.enable_debug == true)
    {
      // std::cout << "size of local lane is: " << local_lane_.points.size() << std::endl;
      // std::cout << "x-position of first local lane point: " << local_lane_.points[0].point.x << std::endl;
    }

    // Update start state, transofrm current robot's state to Frenet coordinates
    updateStartState();
    // Get the reference lane's centerline as a spline
    auto ref_path_and_curve = frenet_planner_.generateReferenceCurve(local_lane_);
    // Store the results into reference spline
    ref_spline_ = std::move(ref_path_and_curve.first);

    if (SETTINGS.enable_debug == true)
    {
      // std::cout << "first x-position of reference path is: " << ref_spline_.x[0] << std::endl;
    }
    if (ref_spline_.x.empty())
    {
      ROS_ERROR("Local Planner: Reference Curve could not be be generated, No path generated");
      publishEmptyTrajsAndStop();

      return;
    }
    // ROS_INFO("Local Planner: Reference Curve Generated");

    // Define ROI width for path sampling
    roi_boundaries_ = getSamplingWidthFromTargetLane(target_lane_id_, SETTINGS.vehicle_width, LANE_WIDTH, LEFT_LANE_WIDTH, RIGHT_LANE_WIDTH);

    // visualize reference path
    ROSLine &line = ros_markers_reference_path_->getNewLine();
    line.setColorInt(10, 10);
    line.setScale(0.05, 0.05);
    line.setOrientation(0.0);
    geometry_msgs::Point prev_point;

    for (size_t i = 0; i < lane_.points.size() - 2; i++)
    {
      geometry_msgs::Point p;
      p.x = lane_.points[i].point.x;
      p.y = lane_.points[i].point.y;
      p.z = -0.5e-3;

      if (i > 0)
        line.addLine(prev_point, p);

      prev_point = p;
    }

    ros_markers_reference_path_->publish();
    publishRefSpline(ref_spline_);

    // // std::cout << "the defined boundaries are" << roi_boundaries_[0] << std::endl;

    // debug start_state
    if (SETTINGS.enable_debug == true)
    {
      // std::cout << "start longitudinal position" << start_state_.s << std::endl;
      // std::cout << "start longitudinal speed" << start_state_.s_d << std::endl;
      // std::cout << "start lateral position" << start_state_.d << std::endl;
      // std::cout << "start lateral speed" << start_state_.d_d << std::endl;
      // std::cout << "start lateral acceleration" << start_state_.d_dd << std::endl;
    }

    visualizeObstaclePrediction(prediction_msg_);
    ros_markers_->publish();

    // Get the planning result
    std::vector<fop::FrenetPath> best_traj_list = frenet_planner_.frenetOptimalPlanning(ref_path_and_curve.second, start_state_, target_lane_id_,
                                                                                        roi_boundaries_[0], roi_boundaries_[1], current_state_.v, CHECK_COLLISION, USE_ASYNC,
                                                                                        prediction_msg_, USE_HEURISTIC, curr_trajectory_, r_x_, risk_planned_traj_client_);

    if (best_traj_list.empty())
    {
      ROS_ERROR("Local Planner: Frenet Optimal Planning Could Not Find a Safe Trajectory");
      // ToDo
      // publishEmptyTrajsAndStop();
      ActuateBrake(3.0);
    }

    // find the best trajectory from all candidates
    fop::FrenetPath best_traj = selectLane(best_traj_list, current_lane_id_);
    // ROS_INFO("Local Planner: Best trajs Selected");

    // Concatenate the best path into output_path
    concatPath(best_traj, TRAJ_MAX_SIZE, TRAJ_MIN_SIZE, WP_MAX_SEP, WP_MIN_SEP);
    // fop::FrenetPath best_traj = selectLane(best_traj_list, current_lane_id_);

    // // std::cout << "the length of the best trajectory is: " << best_traj.x.size() << std::endl;

    // Publish the best trajs
    publishRefSpline(ref_spline_);
    publishCandidateTrajs(*frenet_planner_.all_trajs_fop_);
    publishCurrTraj(curr_trajectory_);
    publishNextTraj(best_traj);
    visualizeTraj(best_traj);

    const auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_time = end_time - start_time;
    data_saver_.AddData("runtime_control_loop", elapsed_time.count() / 1000.);
    // ROS_INFO("Local Planner: Planning took %f ms, (or %f Hz)", elapsed_time.count(), 1000 / elapsed_time.count());
    //  add current vehicle velocity to the associated vector
    //  just for debugging purposes
    vehicle_vel_.push_back(current_state_.v);
    float averaged_velocity = std::accumulate(vehicle_vel_.begin(), vehicle_vel_.end(), 0.0) / vehicle_vel_.size();
    if (SETTINGS.enable_debug)
    {
      // std::cout << "current robot's velocity is: " << current_state_.v << std::endl;
      // std::cout << "averaged velocity is: " << averaged_velocity << std::endl;
    }
  }

  // This function updates with T=1/clock_frequency
  void FrenetOptimalPlannerNode::runNode(const ros::TimerEvent &event)
  {

    processreferencepath();
    ExportData();
  }

  void FrenetOptimalPlannerNode::visualizeTraj(const fop::FrenetPath &next_traj)
  {

    ROSPointMarker &plan_points = collision_space_markers_->getNewPointMarker("CYLINDER");
    ROSPointMarker &ellipse = collision_space_markers_->getNewPointMarker("CYLINDER");
    ROSLine &line = collision_space_markers_->getNewLine();
    int marker_id = 0;

    double alpha = 0.3;
    plan_points.setScale(0.2, 0.2, 0.1e-3);
    plan_points.setColor(0, 0, 1, 1.0);
    double vehicle_radius = 0.325;
    ellipse.setScale(2 * vehicle_radius, 2 * vehicle_radius);
    ellipse.setColor(0, 0, 1, alpha);
    line.setScale(0.15, 0.15);
    line.setColor(0, 0, 1);

    Eigen::Vector2d pose, previous_pose;

    fop::Path vis_traj = curr_trajectory_;
    if (next_traj.x.size() != 0)
    {
      if (SETTINGS.enable_debug)
      {
        // std::cout << "The replan flag is: " << replan_ << std::endl;
      }

      if (replan_ == false)
      {
        for (int i = 0; i < (next_traj.x.size() - curr_trajectory_.x.size()) - 1; i++)
        {
          if (std::isnormal(next_traj.x[i]) && std::isnormal(next_traj.y[i]) && std::isnormal(next_traj.yaw[i]) && next_traj.x[i] >= 0.01)
          {
            vis_traj.x.push_back(next_traj.x[i]);
            vis_traj.y.push_back(next_traj.y[i]);
            vis_traj.yaw.push_back(next_traj.yaw[i]);

            if (next_traj.x[i + 1] < next_traj.x[i])
            {
              return;
            }
          }
        }
      }
      else
      {
        for (int i = curr_trajectory_.x.size(); i < 29; i++)
        {

          if (std::isnormal(next_traj.x[i]) && std::isnormal(next_traj.y[i]) && std::isnormal(next_traj.yaw[i]) && next_traj.x[i] >= 0.01)
          {
            vis_traj.x.push_back(next_traj.x[i]);
            vis_traj.y.push_back(next_traj.y[i]);
            vis_traj.yaw.push_back(next_traj.yaw[i]);

            if (next_traj.x[i + 1] < next_traj.x[i])
            {
              return;
            }
          }
        }
      }
    }

    replan_ = false;
    if (SETTINGS.enable_debug)
    {
      // std::cout << "visualized trajectory size is: " << vis_traj.x.size() << std::endl;
    }

    for (size_t k = 0; k < vis_traj.x.size(); k++)
    {
      // // std::cout << "visualized trajectory x-position is: " << vis_traj.x[k] << std::endl;
      Eigen::Vector2d pose(vis_traj.x[k], vis_traj.y[k]);
      plan_points.addPointMarker(Eigen::Vector3d(pose(0), pose(1), 0.6e-1));
      ellipse.addPointMarker(Eigen::Vector3d(pose(0), pose(1), 0.5e-3));
      if (k >= 1)
        line.addLine(Eigen::Vector3d(previous_pose(0), previous_pose(1), 0.5e-1), Eigen::Vector3d(pose(0), pose(1), 0.5e-1));

      previous_pose = pose;
    }

    collision_space_markers_->publish();
  }

  void FrenetOptimalPlannerNode::Reset()
  {
    ROS_WARN("Resetting...");
    t_last_reset_ = ros::Time::now();

    data_saver_.AddData("reset", control_iteration_); // Add a reset flag to the last data point

    data_saver_.AddData("metric_duration", (control_iteration_ - iteration_at_last_reset_) * (1.0 / CONTROL_FREQUENCY));
    iteration_at_last_reset_ = control_iteration_;

    // Figure out what the name of our recording should be
    std::string project_name;
    nh.getParam("recording/project_name", project_name);
    std::string recording_name;
    nh.getParam("pedestrian_simulator/node/scenario", recording_name);
    std::string segment; // If the name is a file name, remove the file extension
    std::getline(std::stringstream(recording_name), segment, '.');
    std::replace(segment.begin(), segment.end(), '/', '-'); // replace all '/' with '-'
    recording_name = segment;
    std::string prepend_name = "interactive_";
    if (project_name != "")
      recording_name = project_name + "/" + prepend_name + recording_name; // Put it in the project folder

    int num_experiments = 210;

    // Save every x experiments
    if (experiment_counter_ % num_experiments == 0 && experiment_counter_ > 0)
    {
      data_saver_.SaveData(recording_name + "_frenet-planner");
      ROS_WARN("Saving data...");
    }

    experiment_counter_++;

    if (experiment_counter_ >= num_experiments + 1)
    {
      ROS_WARN("Completed the given number of experiments. I know it looks like an error, but it is actually a feature ;)"); // Stop when done with the given number of experiments (+1 for first simulation)
      throw std::runtime_error("Stopping the controller!");
    }

    for (int j = 0; j < 5; j++)
    {
      ActuateBrake(3.0);
      ros::Duration(1.0 / CONTROL_FREQUENCY).sleep();
    }

    std_srvs::Empty reset_msg_;
    robot_localization::SetPose reset_pose_msg_;

    reset_simulation_client_.call(reset_msg_);
    reset_ekf_client_.call(reset_pose_msg_);
    reset_simulation_pub_.publish(std_msgs::Empty());

    ros::Duration(1.0 / CONTROL_FREQUENCY).sleep();
    state_received_ = false;
    t_last_reset_ = ros::Time::now();

    ROS_INFO("Reset completed.");
  }

  void FrenetOptimalPlannerNode::obstacleCallback(const derived_object_msgs::ObjectArray &msg)
  {
    // ROS_INFO_STREAM("Obstacle Callback with " << msg.objects.size() << " objects");
    obstacle_msg_ = msg;
    CreateObstacleList();
  }

  void FrenetOptimalPlannerNode::obstacleTrajectoryPredictionsCallback(const mpc_msgs::obstacle_array &msg)
  {
    // ROS_INFO_STREAM("FrenetOptimalPlannerNode::ObstacleTrajectoryPredictionsCallback received " << msg.obstacles.size() << " obstacle predictions");
    //  this msg includes the trajectory prediction for each obstacle's gaussian mode along prediction horizon
    prediction_msg_ = msg;
    // processreferencepath();
  }

  void FrenetOptimalPlannerNode::CreateObstacleList()
  {
    data_.dynamic_obstacles_.clear();

    int disc_id = 0;
    double obstacle_radius = SETTINGS.obstacle_radius;
    for (auto &object : obstacle_msg_.objects)
    {
      data_.dynamic_obstacles_.emplace_back(object.id, disc_id, obstacle_radius); // Radius = according to lmpcc config
      data_.dynamic_obstacles_.back().pose_ = object.pose;
      disc_id++;
    }

    PlotAllObstacles();
  }

  void FrenetOptimalPlannerNode::PlotAllObstacles()
  {

    ROSPointMarker &obstacle_marker = obstacle_markers_->getNewPointMarker("CYLINDER");
    obstacle_marker.setScale(0.25, 0.25, 1.5);
    obstacle_marker.setColor(0.0, 0.0, 0.0, 0.8);
    double plot_height = 1.5;
    double obstacle_radius = SETTINGS.obstacle_radius;

    // Plot all obstacles
    for (auto &object : data_.dynamic_obstacles_) // obstacle_msg_.objects)
    {

      obstacle_marker.setScale(2.0 * obstacle_radius, 2.0 * obstacle_radius, plot_height);

      // This marker is a simple cube at the exact position.
      object.pose_.position.z = 1e-3 + plot_height / 2.;
      obstacle_marker.addPointMarker(object.pose_);
      obstacle_marker.setZ(1e-3 + plot_height / 2.);
    }

    obstacle_markers_->publish();
  }

  void FrenetOptimalPlannerNode::ActuateBrake(double deceleration)
  {
    control_msg_ = geometry_msgs::Twist();

    control_msg_.linear.x = std::max(current_state_.v - deceleration * SETTINGS.tick_t, 0.);
    control_msg_.angular.z = angular_velocity_;
    command_pub_.publish(control_msg_);
  }

  void FrenetOptimalPlannerNode::ExportData()
  {
    // data_saver_.SetAddTimestamp(true);

    // ROS_INFO("ExportData()");

    // Don't export if the obstacles aren't ready
    if (data_.dynamic_obstacles_.size() == 0)
    {
      ROS_INFO("Not exporting data: Obstacles not yet received.");
      return;
    }

    if (!state_received_)
    {
      ROS_INFO("Not exporting data: State not yet received.");
      return;
    }
    // VEHICLE
    Eigen::Vector2d vehicle_pose(current_state_.x, current_state_.y);
    double vehicle_orientation = current_state_.yaw;

    // Dummy saves
    data_saver_.AddData("status", (int)2);
    data_saver_.AddData("objective", 0.);

    data_saver_.AddData("vehicle_pose", vehicle_pose);
    data_saver_.AddData("vehicle_orientation", vehicle_orientation);

    data_saver_.AddData("longitudinal_velocity", current_state_.v);
    data_saver_.AddData("lateral_velocity", current_angular_velocity_);

    // OBSTACLES
    /*int collisions = 0;
    double max_intrusion = 0.;
    for (size_t v = 0; v < data_.dynamic_obstacles_.size(); v++)
    {
      auto &obstacle = data_.dynamic_obstacles_[v];

      // POSE AND ORIENTATION
      Eigen::Vector2d pose(obstacle.pose_.position.x, obstacle.pose_.position.y); // Can be outdated!
      double orientation = quaternionToAngle(obstacle.pose_);

      // CARLA / Real Jackal
      if (obstacle.id_ != -1)
      {
        data_saver_.AddData("obstacle_map_" + std::to_string(v), obstacle.id_);
        data_saver_.AddData("obstacle_" + std::to_string(obstacle.id_) + "_pose", pose);
        // data_saver_.AddData("obstacle_" + std::to_string(obstacle.id_) + "_orientation", orientation);
      }
      // DISCS
      for (auto &disc : obstacle.discs_)
      {
        data_saver_.AddData("disc_" + std::to_string(disc.id_) + "_pose", disc.TranslateToDisc(pose, orientation));
        data_saver_.AddData("disc_" + std::to_string(disc.id_) + "_radius", disc.radius_); // Write with the correct id?
        // data_saver_.AddData("disc_" + std::to_string(disc.id) + "_obstacle", v);
      }

      // Check for collisions (simple distance check for one disc robot, one disc obstacle)
      // // std::cout << (vehicle_pose - pose).norm() - obstacle.discs_[0].radius - vehicle_->discs_[0].radius << std::endl;
      if ((vehicle_pose - pose).norm() < obstacle.discs_[0].radius_ + 0.325 - 1e-2)
      {
        // // std::cout << "dist to collision boundary: " << (vehicle_pose - pose).norm() - obstacle.discs_[0].radius - vehicle_->discs_[0].radius << std::endl;
        ROS_WARN_STREAM("Collision Detected. Intrusion: " << -((vehicle_pose - pose).norm() - (obstacle.discs_[0].radius_ + 0.325)) << "m");
        collisions++;
        max_intrusion = std::max(max_intrusion, -((vehicle_pose - pose).norm() - (obstacle.discs_[0].radius_ + 0.325)));
      }
    }
    data_saver_.AddData("metric_collisions", collisions);
    data_saver_.AddData("max_intrusion", max_intrusion);*/
    // External collision checking
    data_saver_.AddData("metric_collisions", int(intrusion_ > 0.));
    data_saver_.AddData("max_intrusion", intrusion_);

    data_saver_.AddData("iteration", control_iteration_);
    control_iteration_++;

    /*
      data_saver_.AddData("runtime_modules", module_benchmarkers_.getLast());
      data_saver_.AddData("runtime_optimization", optimization_benchmarker_.getLast());*/

    // data_saver_.AddData("collision", system_interface_->collision_detected_); // Collisions from simulation
    // system_interface_->collision_detected_ = false;

    // Reset if time out
    // if ((double)(control_iteration_ - iteration_at_last_reset_) * (1.0 / ((double)config_->clock_frequency_)) > config_->time_out_)
    // {
    //     // std::cout << "RESETTING BECAUSE OF TIME OUT\n";
    //     OnReset();
    //     iteration_at_last_reset_ = control_iteration_;
    // }
  }

  // Publish the reference spline (for Rviz only)
  void FrenetOptimalPlannerNode::publishRefSpline(const fop::Path &path)
  {
    nav_msgs::Path ref_path_msg;
    ref_path_msg.header.stamp = ros::Time::now();
    ref_path_msg.header.frame_id = "map";

    // std::cout << "reference path size in the publisher is: " << path.yaw.size() << std::endl;

    for (size_t i = 0; i < path.yaw.size(); i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.header = ref_path_msg.header;
      pose.pose.position.x = path.x[i];
      pose.pose.position.y = path.y[i];
      pose.pose.position.z = map_height_;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(path.yaw[i]);
      ref_path_msg.poses.emplace_back(pose);
    }

    ref_path_pub.publish(ref_path_msg);
  }

  // Publish the current path (for Rviz and MPC)
  void FrenetOptimalPlannerNode::publishCurrTraj(const fop::Path &path)
  {
    nav_msgs::Path curr_trajectory_msg;
    curr_trajectory_msg.header.stamp = ros::Time::now();
    curr_trajectory_msg.header.frame_id = "map";

    for (size_t i = 0; i < path.yaw.size(); i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.header = curr_trajectory_msg.header;
      pose.pose.position.x = path.x[i];
      pose.pose.position.y = path.y[i];
      pose.pose.position.z = map_height_ + 2.0 * path.v[i] / fop::Vehicle::max_speed();
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(path.yaw[i]);
      curr_trajectory_msg.poses.emplace_back(pose);
    }

    curr_traj_pub.publish(curr_trajectory_msg);
  }

  // Publish the best next path (for Rviz only)
  void FrenetOptimalPlannerNode::publishNextTraj(const fop::FrenetPath &next_traj)
  {
    nav_msgs::Path curr_trajectory_msg;
    curr_trajectory_msg.header.stamp = ros::Time::now();
    curr_trajectory_msg.header.frame_id = "map";

    for (size_t i = 0; i < next_traj.c.size(); i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.header = curr_trajectory_msg.header;
      pose.pose.position.x = next_traj.x[i];
      pose.pose.position.y = next_traj.y[i];
      pose.pose.position.z = map_height_ + 2.0 * std::hypot(next_traj.s_d[i], next_traj.d_d[i]) / fop::Vehicle::max_speed();
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(next_traj.yaw[i]);
      curr_trajectory_msg.poses.emplace_back(pose);
    }

    next_traj_pub.publish(curr_trajectory_msg);
  }

  /**
   * @brief publish candidate trajs for visualization in rviz
   *
   */
  void FrenetOptimalPlannerNode::publishCandidateTrajs(const std::vector<fop::FrenetPath> &candidate_trajs)
  {
    visualization_msgs::MarkerArray candidate_paths_markers = LocalPlannerVisualization::visualizeCandidateTrajs(candidate_trajs, map_height_, fop::Vehicle::max_speed());

    candidate_paths_pub.publish(std::move(candidate_paths_markers));
  }

  // Publish empty trajs (for Rviz only)
  void FrenetOptimalPlannerNode::publishEmptyTrajsAndStop()
  {
    // Publish empty trajs
    publishRefSpline(fop::Path());
    publishCurrTraj(fop::Path());
    publishNextTraj(fop::FrenetPath());
    publishVehicleCmd(-1.0, 0.0);

    // actuate braking since we cannot find a safe trajectory to follow
    /*
    control_msg_ = geometry_msgs::Twist();
    double velocity_after_braking;
    double deceleration = 5.0;
    velocity_after_braking = current_state_.v - deceleration * (1.0 / 20);
    control_msg_.linear.x  = std::max(velocity_after_braking, 0.);    // Don't drive backwards when braking
    control_msg_.angular.z = 0.0;
    command_pub_.publish(control_msg_);
    */
  }

  // Update the vehicle front axle state (used in odomcallback)
  void FrenetOptimalPlannerNode::updateVehicleFrontAxleState()
  {
    // Current XY of robot (map frame)
    // For the Jackal, this transformation is not necessary
    frontaxle_state_.x = current_state_.x; // + (fop::Vehicle::L() * std::cos(current_state_.yaw));
    frontaxle_state_.y = current_state_.y; // + (fop::Vehicle::L() * std::sin(current_state_.yaw));
    frontaxle_state_.yaw = current_state_.yaw;
    frontaxle_state_.v = current_state_.v;
  }

  std::vector<double> FrenetOptimalPlannerNode::calculate_s(const Lane &ref_wps)
  {
    // store the difference along x and y axis in dx and dy variables
    std::vector<double> dx;
    std::vector<double> dy;
    for (int i = 0; i < ref_wps.points.size() - 1; i++)
    {
      dx.push_back(ref_wps.points[i + 1].point.x - ref_wps.points[i].point.x);
      dy.push_back(ref_wps.points[i + 1].point.y - ref_wps.points[i].point.y);
    }

    // compute the euclidean distances between reference waypoints
    std::vector<double> ds;
    for (int i = 0; i < dx.size(); i++)
    {
      ds.push_back(sqrt(dx[i] * dx[i] + dy[i] * dy[i]));
    }

    // compute the cumulative distances from the starting point
    std::vector<double> s;
    s.push_back(0.0);
    for (int i = 0; i < ds.size(); i++)
    {
      s.push_back(s[i] + ds[i]);
    }

    return s;
  };

  // Feed map waypoints into local map
  bool FrenetOptimalPlannerNode::feedWaypoints()
  {

    // Check if all reference vectors are of the same length
    assert(config_->ref_x_.size() == config_->ref_y_.size());
    assert(config_->ref_x_.size() == config_->ref_psi_.size());
    lane_.points.clear();

    lane_.points.reserve(config_->ref_x_.size());
    for (size_t iIndex = 0; iIndex < config_->ref_x_.size(); iIndex++)
    {
      // // std::cout << "index number is: " << iIndex << std::endl;
      fop::LanePoint path_point;
      path_point.point.x = config_->ref_x_.at(iIndex);
      path_point.point.y = config_->ref_y_.at(iIndex);
      path_point.point.yaw = config_->ref_psi_.at(iIndex);

      lane_.points.push_back(path_point);
    }

    std::vector<double> s = calculate_s(lane_);
    for (size_t iIndex = 0; iIndex < s.size(); iIndex++)
    {
      lane_.points[iIndex].point.s = s[iIndex];
    }

    // std::cout << "size of lane points" << lane_.points.size() << std::endl;
    if (lane_.points.empty())
    {
      ROS_WARN("Local Planner: Waiting for Lane Points");
      return false;
    }
    else if (lane_.points.size() < 5)
    {
      ROS_WARN("Local Planner: Global Path has less than 5 points, unable to plan");
      return false;
    }

    int start_id = fop::lastWaypoint(current_state_, lane_);

    if (SETTINGS.enable_debug)
    {
      // std::cout << "start_id: " << start_id << std::endl;
    }

    if (start_id < 0)
    {
      start_id = 0;
    }

    // if reached the end of the lane, stop
    auto time_since_last_reset = (ros::Time::now() - t_last_reset_).sec;
    if ((time_since_last_reset > 1.0 && current_state_.x > 25.) || SETTINGS.reset_world || time_since_last_reset > 45.) //(start_id >= (lane_.points.size() - 2))  // exclude last 2 waypoints for safety, and prevent code crashing
    {
      // If failure, record as such
      if (SETTINGS.reset_world || time_since_last_reset > 45.)
      {
        data_saver_.AddData("metrics_succes", 0); // Add a reset flag to the last data point

        SETTINGS.reset_world = false;
      }
      else
      {
        data_saver_.AddData("metrics_succes", 1); // Add a reset flag to the last data point
      }

      // ROS_WARN("Local Planner: Vehicle is at waypoint no.%d, with %d waypoints in total", start_id, int(lane_.points.size()));
      if (SETTINGS.enable_debug)
      {
        // std::cout << "start_id in the condition: " << start_id << std::endl;
        // std::cout << "size of lane points in the condition: " << lane_.points.size() << std::endl;
      }

      ROS_WARN("Local Planner: Vehicle has reached the destination");
      // reset environment
      Reset();

      return false;
    }

    const double dist = fop::distance(lane_.points[start_id].point.x, lane_.points[start_id].point.y, current_state_.x, current_state_.y);
    const double heading_diff = fop::unifyAngleRange(current_state_.yaw - lane_.points[start_id].point.yaw);

    if (dist > MAX_DIST_FROM_PATH)
    {
      ROS_WARN("Local Planner: Vehicle's Location Is Too Far From The Target Lane");
      return false;
    }
    // else if (std::abs(heading_diff) > HEADING_DIFF_THRESH)
    // {
    //   ROS_WARN("Local Planner: Vehicle's Is Heading In A Different Direction");
    //   return false;
    // }

    // clear the old waypoints
    local_lane_.clear();

    // Make sure there are at least 5 points in the remaining section of the global reference path
    if (start_id > lane_.points.size() - 5)
    {
      start_id = lane_.points.size() - 5;
    }

    // Check if the global waypoints need to be filtered
    const double ref_spline_length = SETTINGS.highest_speed * (SETTINGS.max_t);

    if (SETTINGS.enable_debug)
    {
      // std::cout << "start_id is: " << start_id << std::endl;
      // std::cout << "reference_spline_length_is: " << ref_spline_length << std::endl;
    }

    if ((lane_.points.back().point.s - lane_.points[start_id].point.s) >= ref_spline_length)
    {
      // Filter the waypoints to a uniform density
      double s_current = lane_.points[start_id].point.s;
      local_lane_.points.push_back(lane_.points[start_id]);
      for (size_t i = start_id + 1; i < lane_.points.size(); i++)
      {
        if (local_lane_.points.size() >= 5)
        {
          break;
        }
        else if ((lane_.points[i].point.s - s_current) >= ref_spline_length / 5.0)
        {
          s_current = lane_.points[i].point.s;
          local_lane_.points.push_back(lane_.points[i]);
        }
      }
      // ROS_INFO("Local Planner: Filtered the global path from %d to %d points", int(lane_.points.size()), int(local_lane_.points.size()));
    }
    else

    {
      // feed the new waypoints
      // ROS_INFO("Local Planner: Global reference path only has %f meters left", lane_.points.back().point.s - lane_.points[start_id].point.s);
      if (SETTINGS.enable_debug)
      {
        // std::cout << "size of lane points: " << lane_.points.size() << std::endl;
      }

      if ((lane_.points.size() - start_id) >= 5)
      {
        const int first_id = start_id;                   // 0
        const int fifth_id = lane_.points.size() - 1;    // 4
        const int third_id = (first_id + fifth_id) / 2;  // 2
        const int second_id = (first_id + third_id) / 2; // 1
        const int fourth_id = (third_id + fifth_id) / 2; // 3

        local_lane_.points.push_back(lane_.points[first_id]);
        local_lane_.points.push_back(lane_.points[second_id]);
        local_lane_.points.push_back(lane_.points[third_id]);
        local_lane_.points.push_back(lane_.points[fourth_id]);
        local_lane_.points.push_back(lane_.points[fifth_id]);
      }
      else
      {
        ROS_WARN("Local Planner: Global reference path only has %d points left, stopped planning!", int(lane_.points.size() - start_id));
        return false;
      }
    }

    // debug
    /*
    for (int index = 0; index < lane_.points.size(); index++)
    {
        // std::cout << "lane points: " << lane_.points[index].point.x << std::endl;
    }

    for (int index = 0; index < local_lane_.points.size(); index++)
    {
        // std::cout << "local_lane points: " << local_lane_.points[index].point.x << std::endl;
    }
    */

    return true;
  }

  // Update the vehicle start state in frenet
  void FrenetOptimalPlannerNode::updateStartState()
  {
    if (local_lane_.points.empty())
    {
      return;
    }

    // if the current path size is too small, regenerate
    if (SETTINGS.enable_debug)
    {
      // std::cout << "current trajectory size is: " << curr_trajectory_.x.size() << std::endl;
    }

    if (curr_trajectory_.x.size() < TRAJ_MIN_SIZE)
    {
      regenerate_flag_ = true;
      if (SETTINGS.enable_debug)
      {
        // std::cout << "replan is:  " << replan_ << std::endl;
      }
    }

    if (SETTINGS.enable_debug)
    {
      // std::cout << "regenerate_flag_ is: " << regenerate_flag_ << std::endl;
    }

    // if need to regenerate the entire path
    if (regenerate_flag_ || SETTINGS.replan_all_time)
    {
      // ROS_INFO("Local Planner: Regenerating The Entire Path...");
      //  Update the starting state in frenet (using ref_spline_ can produce a finer result compared to local_lane_, but
      //  at fringe cases, such as start of code, ref spline might not be available
      //  debug
      //  // std::cout << "ref_spline_ yaw size" << ref_spline_.yaw.size() << std::endl;

      auto return_type = ref_spline_.yaw.empty() ? fop::getFrenet(current_state_, local_lane_) : fop::getFrenet(current_state_, ref_spline_);
      start_state_ = return_type.first;
      r_x_ = return_type.second;
      // Clear the last output path
      curr_trajectory_.clear();
      replan_ = true;
      regenerate_flag_ = false;
    }
    // if not regenerating
    else
    {
      // ROS_INFO("Local Planner: Continuing From The Previous Path...");

      // End of the previous path speed
      const double curr_trajectory_last_speed = hypot(curr_trajectory_.x.back() - curr_trajectory_.x.end()[-2], curr_trajectory_.y.back() - curr_trajectory_.y.end()[-2]) / SETTINGS.tick_t;
      // End of the previous path state
      fop::VehicleState last_state = fop::VehicleState(curr_trajectory_.x.back(), curr_trajectory_.y.back(), curr_trajectory_.yaw.back(), curr_trajectory_.v.back());

      auto return_type = ref_spline_.yaw.empty() ? fop::getFrenet(last_state, local_lane_) : fop::getFrenet(last_state, ref_spline_);
      start_state_ = return_type.first;
      r_x_ = return_type.second;
    }

    // Ensure the speed is above the minimum planning speed
    // start_state_.s_d = std::max(start_state_.s_d, 1.0);

    // Update current lane
    if (std::abs(start_state_.d) <= LANE_WIDTH / 2)
    {
      current_lane_id_ = LaneID::CURR_LANE;
    }
    else if (start_state_.d > LANE_WIDTH / 2)
    {
      current_lane_id_ = LaneID::LEFT_LANE;
    }
    else if (start_state_.d < -LANE_WIDTH / 2)
    {
      current_lane_id_ = LaneID::RIGHT_LANE;
    }
    else
    {
      current_lane_id_ = -1;
      ROS_WARN("Vehicle's lateral position is %f, too far off", start_state_.d);
    }
  }

  // Calculate the sampling width for the planner
  std::vector<double> FrenetOptimalPlannerNode::getSamplingWidthFromTargetLane(const int lane_id, const double vehicle_width, const double current_lane_width,
                                                                               const double left_lane_width, const double right_lane_width)
  {
    double left_bound, right_bound;

    switch (lane_id)
    {
    // all lanes
    case LaneID::ALL_LANES:
      left_bound = current_lane_width / 2 + left_lane_width;
      right_bound = -current_lane_width / 2 - right_lane_width;
      // ROS_INFO("Local Planner: Sampling On ALL Lanes");
      break;

    // stay within the current lane
    case LaneID::CURR_LANE:
      left_bound = current_lane_width / 2;
      right_bound = -current_lane_width / 2;
      // ROS_INFO("Local Planner: Sampling On The Current Lane");
      break;

    // change to left lane
    case LaneID::LEFT_LANE:
      left_bound = current_lane_width / 2 + left_lane_width;
      right_bound = current_lane_width / 2;
      // ROS_INFO("Local Planner: Sampling On The Left Lane");
      break;
    // change to right lane
    case LaneID::RIGHT_LANE:
      left_bound = -current_lane_width / 2;
      right_bound = -current_lane_width / 2 - right_lane_width;
      // ROS_INFO("Local Planner: Sampling On The Right Lane");
      break;
    }

    return {left_bound - vehicle_width / 2, right_bound + vehicle_width / 2};
  }

  // Select the ideal lane to proceed
  fop::FrenetPath FrenetOptimalPlannerNode::selectLane(const std::vector<fop::FrenetPath> &best_traj_list, const int current_lane)
  {
    fop::FrenetPath best_traj;
    bool change_lane_flag;
    int keep_lane_id = -1;
    int change_lane_id = -1;
    double keep_lane_cost = std::numeric_limits<double>::max();
    double change_lane_cost = std::numeric_limits<double>::max();
    /*
    // std::cout << "the number of generated trajectories is: " << best_traj_list.size() << std::endl;
    if (best_traj_list.size() > 2)
    {
        // std::cout << "lane ID of 1st trajectory is: " << best_traj_list[0].lane_id;
        // std::cout << "lane ID of 2nd trajectory is: " << best_traj_list[1].lane_id;
    }
    */

    for (size_t i = 0; i < best_traj_list.size(); i++)
    {
      if (!best_traj_list[i].x.empty())
      {
        // keep lane option
        if (best_traj_list[i].lane_id == current_lane || best_traj_list[i].lane_id == 0)
        {
          if (best_traj_list[i].final_cost < keep_lane_cost)
          {
            keep_lane_id = i;
            keep_lane_cost = best_traj_list[i].final_cost;
          }
        }
        // change lane option
        else
        {
          change_lane_id = i;
          change_lane_cost = best_traj_list[i].final_cost;
        }
      }
    }

    // if both lanes available
    if (keep_lane_id != -1 && change_lane_id != -1)
    {
      if (keep_lane_cost <= change_lane_cost)
      {
        // ROS_INFO("Local Planner: Keeping Lane");
        change_lane_flag = false;
        best_traj = best_traj_list[keep_lane_id];
      }
      else
      {
        // ROS_INFO("Local Planner: Changing Lane");
        change_lane_flag = true;
        best_traj = best_traj_list[change_lane_id];
      }
    }
    // if only keep lane available
    else if (keep_lane_id != -1 && change_lane_id == -1)
    {
      // ROS_INFO("Local Planner: Keeping Lane");
      change_lane_flag = false;
      best_traj = best_traj_list[keep_lane_id];
    }
    // if only change lane available
    else if (keep_lane_id == -1 && change_lane_id != -1)
    {
      // ROS_INFO("Local Planner: Changing Lane");
      change_lane_flag = true;
      best_traj = best_traj_list[change_lane_id];
    }
    // if none available
    else
    {
      // ROS_INFO("Local Planner: No Path Available");
      change_lane_flag = false;
      // dummy path
      best_traj = fop::FrenetPath();
    }

    return best_traj;
  }

  // Concatenate the best next path to the current path
  void FrenetOptimalPlannerNode::concatPath(const fop::FrenetPath &next_traj, const int traj_max_size, const int traj_min_size, const double wp_max_seperation, const double wp_min_seperation)
  {
    size_t diff = 0;
    // std::cout << "current trajectory size is: " << curr_trajectory_.x.size() << std::endl;
    if (curr_trajectory_.x.size() <= traj_min_size)
    {
      diff = std::min(traj_max_size - curr_trajectory_.x.size(), next_traj.x.size());
      // std::cout << "Output Path Size: " << curr_trajectory_.x.size() << " Current Size: " << traj_max_size << " Diff: " << diff
      // << " Next Path Size: " << next_traj.x.size() << std::endl;

      // Concatenate the best path to the output path
      for (size_t i = 0; i < diff; i++)
      {
        // Check if the separation between adjacent waypoint are permitted
        double wp_seperation;
        // // std::cout << "size of current trajectory is: " << curr_trajectory_.x.size() << std::endl;

        if (!curr_trajectory_.x.empty() && !curr_trajectory_.y.empty())
        {
          wp_seperation = fop::distance(curr_trajectory_.x.back(), curr_trajectory_.y.back(), next_traj.x[i], next_traj.y[i]);
        }
        else
        {
          wp_seperation = fop::distance(next_traj.x[i], next_traj.y[i], next_traj.x[i + 1], next_traj.y[i + 1]);
        }

        // If the separation is too big/small, reject point onward
        if (wp_seperation >= wp_max_seperation || wp_seperation <= wp_min_seperation)
        {
          // std::cout << "the way point seperation is: " << wp_seperation << std::endl;
          ROS_WARN("Local Planner: waypoint out of bound, rejected");
          regenerate_flag_ = true;
          // replan_ = true;
          break;
        }

        curr_trajectory_.x.push_back(next_traj.x[i]);
        curr_trajectory_.y.push_back(next_traj.y[i]);
        curr_trajectory_.yaw.push_back(next_traj.yaw[i]);
        curr_trajectory_.v.push_back(std::hypot(next_traj.s_d[i], next_traj.d_d[i]));

        // // std::cout << "Concatenate round " << i << ": Output Path Size: " << curr_trajectory_.x.size() << std::endl;
      }
    }

    // Calculate control outputs and Erase the point that have been executed
    if (!curr_trajectory_.x.empty() && !curr_trajectory_.y.empty())
    {
      // Calculate steering angle
      updateVehicleFrontAxleState();
      const int next_frontlink_wp_id = fop::nextWaypoint(frontaxle_state_, curr_trajectory_);
      // Calculate Control Outputs
      if (calculateControlOutput(next_frontlink_wp_id, frontaxle_state_))
      {
        // Publish steering angle
        publishVehicleCmd(acceleration_, steering_angle_);
      }
      else
      {
        ROS_ERROR("Local Planner: No output steering angle");
        publishVehicleCmd(-1.0, 0.0); // Publish empty control output
      }

      const int next_wp_id = fop::nextWaypoint(current_state_, curr_trajectory_);
      if (SETTINGS.enable_debug)
      {
        // std::cout << "next_wp_id is: " << next_wp_id << std::endl;
      }

      for (size_t i = 0; i < next_wp_id; i++)
      {
        curr_trajectory_.x.erase(curr_trajectory_.x.begin());
        curr_trajectory_.y.erase(curr_trajectory_.y.begin());
        curr_trajectory_.yaw.erase(curr_trajectory_.yaw.begin());
        curr_trajectory_.v.erase(curr_trajectory_.v.begin());
      }
    }
    else
    {
      ROS_ERROR("Local Planner: Output Path is Empty, No Steering Angle");
    }

    // debug output trajectory
    if (SETTINGS.enable_debug)
    {
      // std::cout << "next trajectory size is: " << next_traj.x.size() << std::endl;
      // std::cout << "current x-state is: " << current_state_.x << std::endl;
      // std::cout << "current y-state is: " << current_state_.y << std::endl;
    }

    // debug
    /*
    for (size_t i = 0; i < next_traj.x.size(); i++)
    {
      // std::cout << "x-position of the next trajectory is: " << next_traj.x[i] << std::endl;
    }

    for (size_t i = 0; i < curr_trajectory_.x.size(); i++)
    {
      // std::cout << "x-position of current trajectory is: " << curr_trajectory_.x[i] << std::endl;
    }
    */
  }

  // Steering Help Function
  bool FrenetOptimalPlannerNode::calculateControlOutput(const int next_wp_id, const fop::VehicleState &frontaxle_state)
  {
    const double wp_id = next_wp_id + NUM_WP_LOOK_AHEAD;

    // If the current path is too short, return error value
    if (curr_trajectory_.x.size() < wp_id + 2)
    {
      if (SETTINGS.enable_debug)
      {
        // std::cout << "size of output path is: " << curr_trajectory_.x.size() << std::endl;
      }

      ROS_ERROR("Local Planner: Output Path Too Short! No output steering angle");
      // // std::cout << "Output Path Size: " << curr_trajectory_.x.size() << " Required Size: " << wp_id + 2 << std::endl;
      regenerate_flag_ = true;
      // replan_ = true;

      return false;
    }
    else
    {
      // First Term
      const double delta_yaw = fop::unifyAngleRange(curr_trajectory_.yaw[wp_id] - current_state_.yaw);

      // Second Term
      const double c = fop::distance(curr_trajectory_.x[wp_id], curr_trajectory_.y[wp_id], curr_trajectory_.x[wp_id + 1], curr_trajectory_.y[wp_id + 1]);
      // if two waypoints overlapped, return error value
      if (c <= WP_MIN_SEP)
      {
        ROS_WARN("Local Planner: two points overlapped, Regenerate");
        regenerate_flag_ = true;
        // replan_ = true;

        return false;
      }
      const double a = fop::distance(frontaxle_state.x, frontaxle_state.y, curr_trajectory_.x[wp_id], curr_trajectory_.y[wp_id]);
      const double b = fop::distance(frontaxle_state.x, frontaxle_state.y, curr_trajectory_.x[wp_id + 1], curr_trajectory_.y[wp_id + 1]);
      // if the vehicle is too far from the waypoint, return error value
      if (a >= MAX_DIST_FROM_PATH || b >= MAX_DIST_FROM_PATH)
      {
        ROS_WARN("Local Planner: Vehicle is too far from the path, Regenerate");
        regenerate_flag_ = true;
        // replan_ = true;

        return false;
      }

      const double p = (a + b + c) / 2.0;
      const double triangle_area = sqrt(p * (p - a) * (p - b) * (p - c));
      const double x = triangle_area * 2.0 / c;
      const double u = std::max(1.0, current_state_.v);

      // Angle of vector vehicle -> waypoint
      const double vectors_angle_diff = atan2(frontaxle_state.y - curr_trajectory_.y[wp_id], frontaxle_state.x - curr_trajectory_.x[wp_id]) - curr_trajectory_.yaw[wp_id];
      const double vectors_angle_diff_unified = fop::unifyAngleRange(vectors_angle_diff);
      const int direction = vectors_angle_diff_unified < 0 ? 1 : -1;

      // Final Angle
      steering_angle_ = STANLEY_OVERALL_GAIN * (delta_yaw + direction * atan(TRACK_ERROR_GAIN * x / u));

      // Check if exceeding max steering angle
      steering_angle_ = fop::limitWithinRange(steering_angle_, -fop::Vehicle::max_steering_angle(), fop::Vehicle::max_steering_angle());
      acceleration_ = pid_.calculate(curr_trajectory_.v[wp_id], current_state_.v);

      /** @note: steering_angle_ and acceleration are for vehicle control and are not applicable for controlling the Jackal. */

      // angular_velocity_ = (2*(curr_trajectory_.yaw[wp_id] - current_state_.yaw) / (wp_id * SETTINGS.tick_t)) - current_angular_velocity_;
      double theta_0 = current_state_.yaw;
      double theta_target = curr_trajectory_.yaw[wp_id];
      angular_velocity_ = (theta_target - theta_0) / ((double)wp_id * SETTINGS.tick_t);
      // angular_velocity_ = current_angular_velocity_ - 2. * (theta_0 - theta_target) / (SETTINGS.tick_t * (double)wp_id * (double)wp_id);

      // actuate jackal
      control_msg_ = geometry_msgs::Twist();
      control_msg_.linear.x = curr_trajectory_.v[wp_id];
      control_msg_.angular.z = angular_velocity_;
      command_pub_.publish(control_msg_);

      signal_publishers_[0].Publish(acceleration_);
      signal_publishers_[1].Publish(control_msg_.linear.x);
      signal_publishers_[2].Publish(control_msg_.angular.z);

      // ROS_INFO("Controller: Traget Speed: %2f, Current Speed: %2f, Acceleration: %.2f ", curr_trajectory_.v[wp_id], current_state_.v, acceleration_);
      // ROS_INFO("Controller: Cross Track Error: %2f, Yaw Diff: %2f, SteeringAngle: %.2f ", direction * x, fop::rad2deg(delta_yaw), fop::rad2deg(steering_angle_));
      return true;
    }
  }

  // Publish the resulted steering angle (Stanley)
  void FrenetOptimalPlannerNode::publishVehicleCmd(const double accel, const double angle)
  {
    /*
    autoware_msgs::VehicleCmd vehicle_cmd;
    vehicle_cmd.twist_cmd.twist.linear.x = accel/fop::Vehicle::max_acceleration();  // [pct]
    vehicle_cmd.twist_cmd.twist.angular.z = angle;                                  // [rad]
    vehicle_cmd.gear_cmd.gear = autoware_msgs::Gear::DRIVE;
    vehicle_cmd_pub.publish(vehicle_cmd);
    */
  }

} // namespace fop

int main(int argc, char **argv)
{
  ros::init(argc, argv, "frenet_optimal_planner_node");
  fop::FrenetOptimalPlannerNode frenet_optimal_planner_node;
  ros::spin(); // spin the ros node.
  return 0;
}