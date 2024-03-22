#ifndef ADX_DATA_ODOMETRY_HPP
#define ADX_DATA_ODOMETRY_HPP

#include <roscomp/msgs/nav_msgs.hpp>

#include "adx_data/covariance.hpp"
#include "adx_data/header.hpp"
#include "adx_data/pose.hpp"
#include "adx_data/twist.hpp"

namespace adx {
namespace data {

/**
 * @brief An estimate of robot movement
 *
 * Odometry is an estimate of a position and velocity in free space.
 * Pose represents the estimated position and orientation of the robot.
 * Twist represents the estimated velocities of the robot.
 */
struct Odometry
  : virtual public Header
  , public Pose3d
  , public Twist
{
    /**
     * @brief Construct a new Odometry object
     *
     * Default constructor, this should just zero-initialize an Odometry
     */
    Odometry() = default;

    /**
     * @brief Construct a new Odometry object
     *
     * @param aRosOdometry a ROS Odometry message
     */
    Odometry(const roscomp::nav_msgs::Odometry& aRosOdometry);

    /**
     * @brief Construct a new Odometry object
     *
     * @param aRosOdometry pointer to a ROS Odometry message
     */
    Odometry(roscomp::CallbackPtr<roscomp::nav_msgs::Odometry> aRosOdometry);

    /**
     * @brief Copy-assignment operator
     *
     * @param aOdometry another instance of Odometry
     * @return Odometry& reference to the initialized Odometry object
     */
    Odometry& operator=(const Odometry& aOdometry);

    /**
     * @brief Move-assignment operator
     *
     * @param aRosOdometry a ROS Odometry message with values to copy
     * @return Odometry& reference to the initialized Odometry object
     */
    Odometry& operator=(Odometry&& aOther);

    /**
     * @brief Copy-assignment operator
     *
     * @param aRosOdometry a ROS Odometry message with values to copy
     * @return Odometry& reference to the initialized Odometry object
     */
    Odometry& operator=(const roscomp::nav_msgs::Odometry& aRosOdometry);

    /**
     * @brief Copy-assignment operator
     *
     * @param aRosOdometry pointer to a ROS Odometry message with values to copy
     * @return Odometry& reference to the initialized Odometry object
     */
    Odometry& operator=(roscomp::CallbackPtr<roscomp::nav_msgs::Odometry> aRosOdometry);

    roscomp::nav_msgs::Odometry toRos() const
    {
        roscomp::nav_msgs::Odometry ros_odom;
        ros_odom.header = Header::toRos();
        ros_odom.pose = Pose3d::toRos().pose;
        ros_odom.twist = Twist::toRos();

        return ros_odom;
    }

};

} // namespace data
} // namespace adx

#endif // ADX_DATA_ODOMETRY_HPP
