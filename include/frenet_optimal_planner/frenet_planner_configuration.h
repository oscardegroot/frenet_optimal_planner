#ifndef FRENET_PLANNER_CONFIGURATION_H
#define FRENET_PLANNER_CONFIGURATION_H

#include <ros/ros.h>

class frenet_planner_configuration
{

public:
  /**
   * @brief frenet_planner_configuration: default constructor of this class
   */
   frenet_planner_configuration();
   
  /**
   * @brief frenet_planner_configuration: default distructor of this class
   */
   ~frenet_planner_configuration();
   
  /**
   * @brief intialize: Read all parameters from the ros parameter server
   * @return TRUE iff all parameter initialize successfully
   */
  bool initialize();

  /************ CONFIGURATION VARIABLES **************/
  bool initialize_success_;

  // Initialize vectors for reference path points
  std::vector<double> ref_x_;
  std::vector<double> ref_y_;
  std::vector<double> ref_psi_;
  double road_width_right_, road_width_left_;

private:
  /**
   * @brief Retrieve a parameter from the ROS parameter server, return false if it failed
   *
   * @tparam T Variable type
   * @param nh nodehandle
   * @param name Name of the parameter on the server
   * @param value Variable to store the read value in
   * @return true If variable exists
   * @return false If variable does not exist
   */
  template <class T>
  bool retrieveParameter(const ros::NodeHandle &nh, const std::string &name, T &value)
  {

    if (!nh.getParam(name, value))
    {
      ROS_WARN_STREAM(" Parameter " << name << " not set on node " << ros::this_node::getName().c_str());
      return false;
    }
    else
    {
      return true;
    }
  }

  /**
   * @brief Retrieve a parameter from the ROS parameter server, otherwise use the default value
   *
   * @tparam T Variable type
   * @param nh nodehandle
   * @param name Name of the parameter on the server
   * @param value Variable to store the read value in
   * @param default_value Default value to use if the variable does not exist
   */
  template <class T>
  void retrieveParameter(const ros::NodeHandle &nh, const std::string &name, T &value, const T &default_value)
  {

    if (!retrieveParameter(nh, name, value))
    {
      ROS_WARN_STREAM("\tUsing default value: " << default_value);
      value = default_value;
    }
  }
};

#endif

