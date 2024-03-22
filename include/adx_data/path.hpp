#ifndef ADX_DATA_PATH_HPP
#define ADX_DATA_PATH_HPP

#include <vector>

#include <roscomp/msgs/adx_msgs.hpp>
#include <roscomp/msgs/nav_msgs.hpp>

#include "adx_data/header.hpp"
#include "adx_data/pose.hpp"
#include "adx_data/vector.hpp"

namespace adx {
namespace data {

/**
 * @brief Path for a robot to follow
 *
 * A series of Pose representing a Path for robot navigation
 */
struct Path : virtual public Header
{
    /**
     * The series of poses that are used to represent the path
     */
    // std::vector<Pose3d> poses;
    std::vector<Pose<double, 3, false, false>> poses;

    /**
     * @brief Construct a new Path object
     *
     * Default constructor, this should just zero-initialize the poses
     */
    Path() = default;

    /**
     * @brief Construct a new Path object
     *
     * @param aRosPath a ROS Path message
     */
    Path(const roscomp::nav_msgs::Path& aRosPath);

    /**
     * @brief Construct a new Path object
     *
     * @param aRosPath pointer to a ROS Path message
     */
    Path(roscomp::CallbackPtr<roscomp::nav_msgs::Path> aRosPath);

    /**
     * @brief Construct a new Path object
     *
     *@param aRosPath a ROS Plan message
     */
    Path(const roscomp::adx_msgs::Plan& aRosPlan);

    /**
     * @brief Construct a new Path object
     *
     * @param aRosPath pointer to a ROS Plan message
     */
    Path(roscomp::CallbackPtr<roscomp::adx_msgs::Plan> aRosPlan);

    /**
     * @brief Convert this data to ROS message
     *
     * @return nav_msgs::Path ROS message initialized from this class
     */
    roscomp::nav_msgs::Path toRos() const;
};

/**
 * @brief Plan for a robot to follow
 *
 * This is essentially just an Path that uses Point instead of Pose
 * and stores a reference speed for each point.
 * This is useful in racing environments.
 */
struct Plan : virtual public Header
{
    /**
     * The series of points that are used to represent the path
     */
    std::vector<Point3d> positions;

    /**
     * The reference speeds associated with each of the points
     */
    std::vector<adx::data::Vector3d> speeds;

    /**
     * @brief Construct a new Plan object
     *
     * Default constructor, this should just initialize the empty vectors
     */
    Plan() = default;

    /**
     * @brief Construct a new Plan object
     *
     * @param aRosPlan a ROS Plan message
     */
    Plan(const roscomp::adx_msgs::Plan& aRosPlan);

    /**
     * @brief Construct a new Plan object
     *
     * @param aRosPlan pointer to a ROS Plan message
     */
    Plan(roscomp::CallbackPtr<roscomp::adx_msgs::Plan> aRosPlan);

    /**
     * @brief Convert this data to ROS message
     *
     * @return adx_msgs::Plan ROS message initialized from this class
     */
    roscomp::adx_msgs::Plan toRos() const;

    /**
     * @brief Convert this data to ROS Path message
     *
     * @return nav_msgs::Path ROS message initialized from this class
     */
    roscomp::nav_msgs::Path toRosPath() const;
};

} // namespace data
} // namespace adx

#endif // ADX_DATA_PATH_HPP
