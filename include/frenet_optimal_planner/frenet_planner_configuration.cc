#include <frenet_optimal_planner/frenet_planner_configuration.h>

frenet_planner_configuration::frenet_planner_configuration()
{
    initialize_success_ = false;
}

frenet_planner_configuration::~frenet_planner_configuration()
{
}

bool frenet_planner_configuration::initialize()
{
    ros::NodeHandle nh_config;
    ros::NodeHandle nh;

    // Path parameters
    retrieveParameter(nh, "global_path/x", ref_x_);
    retrieveParameter(nh, "global_path/y", ref_y_);
    retrieveParameter(nh, "global_path/psi", ref_psi_);

    initialize_success_ = true;


    ROS_WARN("[LMPCC]: Parameter Configuration Initialized");
    return true;
}
