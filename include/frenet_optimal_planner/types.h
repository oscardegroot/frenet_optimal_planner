/**
 * @file types.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Defines basic classes for use in LMPCC
 * @version 0.1
 * @date 2022-05-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __TYPES_H__
#define __TYPES_H__

#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>

#include <frenet_optimal_planner/dynamic_obstacle.h>

// To distinguish custom from regular optimization loops.
#define EXIT_CODE_NOT_OPTIMIZED_YET -999

// MINOR TYPES
struct Path
{
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> psi_;

    /**
     * @brief Construct a new Path object
     *
     * @param length Size to allocate (reserve)
     */
    Path(int length = 10)
    {
        x_.reserve(length);
        y_.reserve(length);
        psi_.reserve(length);
    }

    void AddPose(const geometry_msgs::Pose &pose)
    {
        x_.push_back(pose.position.x);
        y_.push_back(pose.position.y);

        // transform from quaternion to RPY angle
        double ysqr = pose.orientation.y * pose.orientation.y;
		double t3 = +2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y);
		double t4 = +1.0 - 2.0 * (ysqr + pose.orientation.z * pose.orientation.z);
        double angle = atan2(t3, t4);
        psi_.push_back(angle);
        //psi_.push_back(Helpers::quaternionToAngle(pose));
    }

    void Clear()
    {
        x_.clear();
        y_.clear();
        psi_.clear();
    }
};

struct Halfspace
{
    Eigen::Vector2d A_;
    double b_;

    Halfspace(){};

    Halfspace(const Eigen::Vector2d &A, const double b)
        : A_(A), b_(b)
    {
    }

    static Halfspace &Dummy()
    {
        static Halfspace dummy(Eigen::Vector2d(1, 0), 1000);
        return dummy;
    }
};

/**
 * @brief A wrapper class to track if data was used yet and if new data has been received
 *
 * @tparam T The variable type
 */
template <class T>
class Tracked
{
public:
    Tracked()
    {
        data_is_new_ = false;
    };

public:
    void Set(const T &new_value)
    {
        value_ = new_value;
        data_is_new_ = true;
    }

    /**
     * @brief Get the data
     *
     * @param keep_data_flag Set to true if the operation should not change the used status of the data to false
     * @return T&
     */
    T &Get(bool keep_data_flag = false)
    {
        data_is_new_ = false & keep_data_flag;
        return value_;
    };

    bool DataIsNew() const
    {
        return data_is_new_;
    }

private:
    T value_;
    bool data_is_new_;
};

/**
 * @brief A wrapper for std::vector that tracks if new data was inserted. To be used for tracking the arrival of real-time data.
 *
 * @tparam T
 */
template <class T>
class TrackedVector : public std::vector<T>
{
public:
    TrackedVector()
        : std::vector<T>()
    {
        data_is_new_ = false;
    };

    /**
     * @brief Copy constructor from a vector
     *
     * @param other a std::vector
     */
    TrackedVector(const std::vector<T> &other)
        : std::vector<T>(other)
    {
        data_is_new_ = true;
    };

public:
    /**
     * @brief A wrapper for emplace_back, sets new data arrived to true
     * @todo Set data_is_new_ to false when data was used
     *
     * @tparam Args
     * @param args
     */
    template <typename... Args>
    void emplace_back(Args &&...args)
    {
        std::vector<T>::emplace_back(std::forward<Args>(args)...);
        data_is_new_ = true;
    }

    /**
     * @brief Wrapper for push_back that sets new data arrived to true
     *
     * @param new_value
     */
    void push_back(const T &new_value)
    {
        std::vector<T>::push_back(new_value);
        data_is_new_ = true;
    }

    /**
     * @brief True if new data was received and not yet used
     *
     * @return true
     * @return false
     */
    bool DataIsNew() const
    {
        return data_is_new_;
    }

private:
    bool data_is_new_;
};

// MAJOR TYPES

/**
 * @brief Stores relevant real-time data in one place
 *
 */
class RealTimeData
{

public:
    RealTimeData(int n_dynamic_obstacles = 0, int n_static_obstacles = 0)
    {
        dynamic_obstacles_.reserve(n_dynamic_obstacles);
        halfspaces_.reserve(n_static_obstacles);
    }

    virtual ~RealTimeData(){};

    void Print()
    {
        ROS_WARN("========== Real Time Data =========");
        ROS_INFO_STREAM("- DYNAMIC OBSTACLES -");
        for (auto &obs : dynamic_obstacles_)
            ROS_INFO_STREAM("Obs [" << obs.id_ << "]: (" << obs.pose_.position.x << ", " << obs.pose_.position.y << ")");

        ROS_INFO_STREAM("- STATIC OBSTACLES -");
        for (int k = 0; k < 0; k++)
        {
            for (auto &halfspace : halfspaces_[k])
            {
                ROS_INFO_STREAM("Halfspace at stage " << k << " [" << halfspace.A_(0) << ", " << halfspace.A_(1) << "] b = " << halfspace.b_);
            }
        }

        ROS_INFO_STREAM("- PATH -");
        for (size_t i = 0; i < path_.Get().x_.size(); i++)
            ROS_INFO_STREAM("Path Waypoint: " << path_.Get().x_[i] << ", " << path_.Get().y_[i] << ", " << path_.Get().psi_[i]);

        ROS_WARN("===================================");
    }

public:
    // Obstacles
    TrackedVector<DynamicObstacle> dynamic_obstacles_; // Dynamic -> /** @Todo: Make a type for this? */
    TrackedVector<std::vector<Halfspace>> halfspaces_; // Static (N x Nh)

    // Path
    Tracked<Path> path_;

protected:
};

#endif // __TYPES_H__
