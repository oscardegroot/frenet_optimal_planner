/**
 * @file collision_disc.h
 * @author Oscar de Groot (o.m.degroot@tudelft.nl)
 * @brief Class for modeling collision regions as discs
 * @version 0.1
 * @date 2022-04-25
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef COLLISION_DISC_H
#define COLLISION_DISC_H

#include <Eigen/Dense>


/**
 * @brief Class for modeling collision regions as discs
 */
class Disc
{

public:
    int id_; // Disc ID for identification

    double offset_; // Offset of a disc
    double radius_; // Radius of a disc

    /**
     * @brief Construct a disc from known parameters
     *
     * @param id ID
     * @param offset Offset
     * @param radius Radius
     */
    Disc(int id, double offset, double radius)
        : id_(id), offset_(offset), radius_(radius)
    {
    }

    /**
     * @brief Translate a pose to a disc pose
     *
     * @param pose the initial pose
     * @param orientation the orientation
     * @return Eigen::Vector2d the translated pose
     */
    Eigen::Vector2d TranslateToDisc(const Eigen::Vector2d &pose, const double &orientation) const
    {
        return Eigen::Vector2d(
            pose(0) + offset_ * std::cos(orientation),
            pose(1) + offset_ * std::sin(orientation));
    }

    /**
     * @brief Translate a disc pose to the vehicle pose
     *
     * @param pose the disc pose
     * @param orientation the orientation
     * @return Eigen::Vector2d the vehicle pose
     */
    Eigen::Vector2d TranslateToPose(const Eigen::Vector2d &disc_pose, const double &orientation) const
    {
        return Eigen::Vector2d(
            disc_pose(0) - offset_ * std::cos(orientation),
            disc_pose(1) - offset_ * std::sin(orientation));
    }

    /**
     * @brief Get the x position of a disc
     *
     * @param pose_x the x of the vehicle
     * @param orientation orientation of the vehicle
     * @return double x of the disc
     */
    double DiscX(const double &pose_x, const double &orientation)
    {
        return pose_x + offset_ * std::cos(orientation);
    }

    /**
     * @brief Get the y position of a disc
     *
     * @param pose_y the y of the vehicle
     * @param orientation orientation of the vehicle
     * @return double y of the disc
     */
    double DiscY(const double &pose_y, const double &orientation)
    {
        return pose_y + offset_ * std::sin(orientation);
    }

    operator std::string() const
    {
        return "disc_" + std::to_string(id_);
    }
};

#endif