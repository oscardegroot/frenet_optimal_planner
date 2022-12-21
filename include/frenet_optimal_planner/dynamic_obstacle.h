/**
 * @file dynamic_obstacle.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Class that holds obstacle information (position and prediction) and their collision regions.
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef DYNAMIC_OBSTACLE_H
#define DYNAMIC_OBSTACLE_H

#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <lmpcc_msgs/gaussian.h>
#include <lmpcc_msgs/obstacle_gmm.h>

#include "frenet_optimal_planner/collision_region.h"

/**
 * @brief Class that holds obstacle information (position and prediction) and their collision regions.
 *
 * @param id_ id as defined by an external module (i.e., a simulation)
 * @param disc_start_id where do this obstacle's disc IDs start
 * @param discs_ the discs spanning the collision region of this obstacle
 * @param pose_ The position of the obstacle currently
 * @param prediction_ the predicted movement as Gaussian distribution
 * @todo Make discs easily iterable
 */
class DynamicObstacle
{

public:
    /**
     * @brief Create an obstacle with circular shape.
     *
     * @param id
     * @param disc_start_id start of obstacle discs as index (so that all discs can have a unique ID)
     * @param radius radius of the collision region
     */
    DynamicObstacle(int id, int disc_start_id, double radius)
        : id_(id)
    {
        discs_.emplace_back(disc_start_id, 0., radius);
    }

    /**
     * @brief Create an obstacle with rectangular shape.
     *
     * @param id
     * @param disc_start_id start of obstacle discs as index (so that all discs can have a unique ID)
     * @param width
     * @param length
     * @param center_offset the state is offset with "center_offset" w.r.t. to the middle of the object
     * @param n_discs number of discs to model the collision region
     */
    DynamicObstacle(int id, int disc_start_id, double width, double length, double center_offset, int n_discs)
        : id_(id)
    {
        // Compute the offsets of the discs
        std::vector<double> offsets;
        for (int i = 0; i < n_discs; i++)
        {
            offsets.push_back(-center_offset - length / 2. + length / (n_discs) * (i + 0.5));
        }

        // The radius is fitted to contain the full obstacle
        double radius = std::sqrt(std::pow(offsets[0], 2.) + std::pow(width / 2., 2.));

        // Create discs for the obstacle
        for (int i = 0; i < n_discs; i++)
        {
            discs_.emplace_back(disc_start_id + i, offsets[i], radius);
        }
    }

    operator std::string() const
    {
        return "obstacle_" + std::to_string(id_);
    }

public:
    int id_;

    std::vector<Disc> discs_;

    geometry_msgs::Pose pose_;
    lmpcc_msgs::obstacle_gmm prediction_; // Deterministic or Gaussian

    /**
     * @brief Predict constant velocity for this obstacle if no prediction is given
     *
     * @param twist current twist of the obstacle
     * @param dt delta time of the prediction
     * @param N horizon of the prediction
     */
    void PredictConstantVelocity(const geometry_msgs::Twist &twist, double dt, int N, double one_sigma_radius = 1.0, double growth = 1.03)
    {
        prediction_.gaussians.clear();
        prediction_.probabilities.clear();

        lmpcc_msgs::gaussian gaussian;
        gaussian.mean.poses.resize(N);
        gaussian.major_semiaxis.resize(N);
        gaussian.minor_semiaxis.resize(N);

        for (int t = 0; t < N; t++)
        {
            gaussian.mean.poses[t].pose.position.x = pose_.position.x + twist.linear.x * dt * t;
            gaussian.mean.poses[t].pose.position.y = pose_.position.y + twist.linear.y * dt * t;

            gaussian.major_semiaxis[t] = one_sigma_radius * std::pow(growth, (double)t); // How should this increase over the horizon
            gaussian.minor_semiaxis[t] = one_sigma_radius * std::pow(growth, (double)t);
        }

        prediction_.gaussians.push_back(gaussian);
        prediction_.probabilities.push_back(1.0);
    }

    /**
     * @brief MoG Predictions for debugging purposes
     *
     * @param twist current twist of the obstacle
     * @param dt delta time of the prediction
     * @param N horizon of the prediction
     */
    void PredictDebugMixtureOfGaussian(const geometry_msgs::Twist &twist, double dt, int N)
    {
        prediction_.gaussians.clear();
        prediction_.probabilities.clear();

        int modes = 3;            // Number of modes
        double range = 3.14 / 6.; // In radians

        for (int mode = 0; mode < modes; mode++)
        {
            lmpcc_msgs::gaussian gaussian;
            gaussian.mean.poses.resize(N);
            gaussian.major_semiaxis.resize(N);
            gaussian.minor_semiaxis.resize(N);

            double heading = -range + (2. * range / ((double)modes)) * ((double)mode);
            // formulate rotation matrix
            Eigen::Matrix2d rotation_matrix;
		    rotation_matrix << std::cos(heading), std::sin(heading),
			                  -std::sin(heading), std::cos(heading);                
            // Eigen::Matrix2d rotation_matrix = Helpers::rotationMatrixFromHeading(heading);
            
            Eigen::Vector2d rotated_twist = rotation_matrix * Eigen::Vector2d(twist.linear.x, twist.linear.y);

            for (int t = 0; t < N; t++)
            {
                gaussian.mean.poses[t].pose.position.x = pose_.position.x + rotated_twist(0) * dt * t;
                gaussian.mean.poses[t].pose.position.y = pose_.position.y + rotated_twist(1) * dt * t;

                gaussian.major_semiaxis[t] = discs_[0].radius_ * 2. / 4. * std::pow(1.03, (double)t); // How should this increase over the horizon
                gaussian.minor_semiaxis[t] = discs_[0].radius_ * 2. / 4. * std::pow(1.03, (double)t);
            }

            prediction_.gaussians.push_back(gaussian);
            prediction_.probabilities.push_back(1.0 / (double)modes);
        }
    }

    /**
     * @brief Dummy obstacle for padding of input data
     *
     * @param x x of the vehicle
     * @param y y of the vehicle
     * @param N horizon of the prediction
     */
    void DummyPrediction(const double x, const double y, int N)
    {
        prediction_.gaussians.clear();
        prediction_.probabilities.clear();

        lmpcc_msgs::gaussian gaussian;
        gaussian.mean.poses.resize(N);
        gaussian.major_semiaxis.resize(N);
        gaussian.minor_semiaxis.resize(N);

        for (int t = 0; t < N; t++)
        {
            gaussian.mean.poses[t].pose.position.x = x + 100.;
            gaussian.mean.poses[t].pose.position.y = y + 100.;

            gaussian.major_semiaxis[t] = 0.01;
            gaussian.minor_semiaxis[t] = 0.01;
        }

        prediction_.gaussians.push_back(gaussian);
        prediction_.probabilities.push_back(1.0);
    }

    /**
     * @brief Load predictions into this obstacle
     *
     * @param prediction predictions to load
     */
    void LoadPredictions(const lmpcc_msgs::obstacle_gmm &prediction)
    {
        prediction_ = prediction; // Simple copy
    }

private:
};

#endif