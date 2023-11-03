/** frenet_optimal_planner_node.h
 *
 * Copyright (C) 2022 Shuo SUN & Advanced Robotics Center, National University of Singapore
 *
 * Apache License 2.0 https://www.apache.org/licenses/LICENSE-2.0
 *
 * Local Planner ROS Node
 * Using the algorithm described in this paper, https://ieeexplore.ieee.org/document/5509799
 */

#include <cmath>
#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

//#include <autoware_msgs/VehicleCmd.h>
//#include <autoware_msgs/DetectedObjectArray.h>

#include <dynamic_reconfigure/server.h>
#include <frenet_optimal_planner/frenet_optimal_planner_Config.h>
#include <derived_object_msgs/ObjectArray.h>
#include <derived_object_msgs/Object.h>

#include "frenet_optimal_planner/pid.hpp"
#include "frenet_optimal_planner/visualization.hpp"
#include "frenet_optimal_planner/frenet_optimal_trajectory_planner.h"
#include "frenet_optimal_planner/frenet_planner_configuration.h"

#include <frenet_optimal_planner/ros_visuals.h>
#include <frenet_optimal_planner/data_saver.h>
#include <frenet_optimal_planner/types.h>
#include <frenet_optimal_planner/dynamic_obstacle.h>

#include <mpc_msgs/obstacle_array.h>
#include <mpc_msgs/obstacle_gmm.h>
#include <mpc_msgs/gaussian.h>

#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <robot_localization/SetPose.h>
#include <numeric>
#include <frenet_optimal_planner/ObservedRisk.h>

#define CONTROL_FREQUENCY 20.0

namespace fop
{
  // Oscar
  inline double quaternionToAngle(const geometry_msgs::Pose &pose)
  {
    double ysqr = pose.orientation.y * pose.orientation.y;
    double t3 = +2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y);
    double t4 = +1.0 - 2.0 * (ysqr + pose.orientation.z * pose.orientation.z);

    return atan2(t3, t4);
  }

  class FrenetOptimalPlannerNode
  {
  public:
    // Constructor
    FrenetOptimalPlannerNode();
    // Destructor
    virtual ~FrenetOptimalPlannerNode(){};

    // Planning algorithm instance
    FrenetOptimalTrajectoryPlanner frenet_planner_;

  private:
    // Regnerate path flag
    bool regenerate_flag_;
    bool replan_ = false;
    // Oscar
    bool state_received_ = false;

    // Lane related variables
    int current_lane_id_;
    int target_lane_id_;
    double map_height_;

    // Control outputs
    double acceleration_;
    double steering_angle_;

    double angular_velocity_;
    double current_angular_velocity_;

    // Oscar
    ros::Time t_last_reset_;
    int control_iteration_ = 0, iteration_at_last_reset_ = 0, experiment_counter_ = 0;

    // Vehicle's current state
    fop::VehicleState current_state_;   // State of the vehicle baselink
    fop::VehicleState frontaxle_state_; // State of the vehicle frontaxle

    // Trajectory Sampling States
    fop::FrenetState start_state_; // Starting States for sampling

    // Maps and Trajs
    fop::Lane lane_;                     // Maps (All the waypoints)
    fop::Lane local_lane_;               // Selected Waypoints
    fop::Path ref_spline_;               // Reference Spline
    fop::Path curr_trajectory_;          // Output Trajectory
    std::vector<double> roi_boundaries_; //[0] = left boundary length in metre, [1] = right boundary length in metre.

    std::vector<float> vehicle_vel_;
    // Oscar
    double total_exp_time_;
    std::vector<float> duration_per_iteration_;

    // Controllers
    control::PID pid_;

    double r_x_;

    // subscriber and publishers
    ros::Subscriber odom_sub;
    ros::Subscriber lane_info_sub;
    ros::Subscriber obstacles_sub, obstacle_prediction_sub;
    ros::Subscriber collision_sub_;

    ros::Publisher curr_traj_pub;
    ros::Publisher next_traj_pub;
    ros::Publisher ref_path_pub;
    ros::Publisher vehicle_cmd_pub;
    ros::Publisher obstacles_pub;
    ros::Publisher sample_space_pub;
    ros::Publisher final_traj_pub;
    ros::Publisher candidate_paths_pub;

    // Oscar: Additions. Comment it for now.
    std::vector<SignalPublisher> signal_publishers_; /* Mainly intended for publishing control signals to be visualized by jsk_rviz_plugins */
    tf2_ros::TransformBroadcaster path_pose_pub_;

    // To reset the environment after each experiment
    ros::Publisher reset_simulation_pub_;
    ros::ServiceClient reset_simulation_client_, reset_ekf_client_;

    frenet_optimal_planner::ObservedRisk risk_srv_;
    ros::ServiceClient risk_planned_traj_client_;

    ros::Publisher command_pub_;
    geometry_msgs::Twist control_msg_;

    // ROS
    ros::NodeHandle nh;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    dynamic_reconfigure::Server<frenet_optimal_planner::frenet_optimal_planner_Config> server;
    dynamic_reconfigure::Server<frenet_optimal_planner::frenet_optimal_planner_Config>::CallbackType f;
    derived_object_msgs::ObjectArray obstacle_msg_;
    mpc_msgs::obstacle_array prediction_msg_;

    ros::Timer timer_;
    RealTimeData data_;
    ros::Time t_last_tf_;
    std::unique_ptr<ROSMarkerPublisher> ros_markers_reference_path_;
    std::unique_ptr<ROSMarkerPublisher> collision_space_markers_;
    std::unique_ptr<ROSMarkerPublisher> obstacle_markers_;
    std::unique_ptr<ROSMarkerPublisher> ros_markers_;

    boost::shared_ptr<frenet_planner_configuration> config_;

    // ###################################### Private Functions ######################################

    // Functions for subscribing
    void laneInfoCallback(const nav_msgs::Path::ConstPtr &global_path);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
    // void stateCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void CollisionCallback(const std_msgs::Float64 &msg);
    double intrusion_;

    void processreferencepath();
    void obstacleCallback(const derived_object_msgs::ObjectArray &msg);
    void obstacleTrajectoryPredictionsCallback(const mpc_msgs::obstacle_array &msg);

    void PlotAllObstacles();
    void CreateObstacleList();

    // void obstaclesCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& input_obstacles);

    // Functions fo publishing results
    void publishEmptyTrajsAndStop();
    void publishRefSpline(const fop::Path &path);
    void publishCurrTraj(const fop::Path &path);
    void publishNextTraj(const fop::FrenetPath &next_traj);
    void publishVehicleCmd(const double accel, const double angle);
    void publishCandidateTrajs(const std::vector<fop::FrenetPath> &candidate_trajs);

    // Planner Helper Functions
    bool feedWaypoints();
    std::vector<double> calculate_s(const Lane &ref_wps);

    void updateStartState();
    // void transformObjects(autoware_msgs::DetectedObjectArray& output_objects, const autoware_msgs::DetectedObjectArray& input_objects);

    std::vector<double> getSamplingWidthFromTargetLane(const int lane_id, const double vehicle_width, const double current_lane_width,
                                                       const double left_lane_width, const double right_lane_width);
    fop::FrenetPath selectLane(const std::vector<fop::FrenetPath> &best_path_list, const int current_lane);
    void concatPath(const fop::FrenetPath &next_traj, const int traj_max_size, const int traj_min_size, const double wp_max_seperation, const double wp_min_seperation);

    // Stanley Steeing Functions
    void updateVehicleFrontAxleState();
    bool calculateControlOutput(const int next_wp_id, const fop::VehicleState &frontaxle_state);

    void runNode(const ros::TimerEvent &event);
    void visualizeTraj(const fop::FrenetPath &best_traj);

    void visualizeObstaclePrediction(const mpc_msgs::obstacle_array &obstacles_prediction);

    // Oscar: Added functionality below
    /** @brief Reset when the end of the path is reached */
    void Reset();

    void ActuateBrake(double deceleration);

    DataSaver data_saver_;
    void ExportData();
  };

} // namespace fop