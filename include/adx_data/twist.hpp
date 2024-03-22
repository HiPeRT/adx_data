#ifndef ADX_DATA_TWIST_HPP
#define ADX_DATA_TWIST_HPP

#include <Eigen/Geometry>

#include <roscomp/msgs/geometry_msgs.hpp>

#include "adx_data/covariance.hpp"
#include "adx_data/vector.hpp"

namespace adx {
namespace data {

/**
 * @brief Velocity
 *
 * This represents velocity in free space broken into its linear and angular components.
 */
struct Twist
{
    /**
     * The 3D vector representing the linear velocity component
     */
    adx::data::Vector3d linear = {};

    /**
     * The 3D vector representing the angular velocity component
     */
    adx::data::Vector3d angular = {};

    /**
     * The Covariance matrix
     * In order there should be x-linear, y-linear, z-linear, x-angular, y-angular, z-angular.
     */
    Covarianced<6> twist_covariance = {};

    /**
     * @brief Construct a new Twist object
     *
     * Default constructor, this should just zero-initialize everything
     */
    Twist() = default;

    /**
     * @brief Construct a new Twist object
     *
     * @param aRosTwist a ROS Twist message
     */
    Twist(const roscomp::geometry_msgs::Twist& aRosTwist);

    /**
     * @brief Construct a new Twist object
     *
     * @param aRosTwist pointer to a ROS Twist message
     */
    Twist(roscomp::CallbackPtr<roscomp::geometry_msgs::Twist> aRosTwist);

    /**
     * @brief Construct a new Twist object
     *
     * @param aRosTwist a ROS TwistWithCovariance message
     */
    Twist(const roscomp::geometry_msgs::TwistWithCovariance& aRosTwist);

    /**
     * @brief Construct a new Twist object
     *
     * @param aRosTwist pointer to a ROS TwistWithCovariance message
     */
    Twist(roscomp::CallbackPtr<roscomp::geometry_msgs::TwistWithCovariance> aRosTwist);

    /**
     * @brief Copy-assignment operator
     *
     * @param aRosTwist a ROS Twist message with values to copy
     * @return Twist& reference to the initialized Twist object
     */
    Twist& operator=(const roscomp::geometry_msgs::Twist& aRosTwist);

    /**
     * @brief Copy-assignment operator
     *
     * @param aRosTwist pointer to a ROS Twist message with values to copy
     * @return Twist& reference to the initialized Twist object
     */
    Twist& operator=(roscomp::CallbackPtr<roscomp::geometry_msgs::Twist> aRosTwist);

    /**
     * @brief Copy-assignment operator
     *
     * @param aRosTwist a ROS TwistWithCovariance message with values to copy
     * @return Twist& reference to the initialized Twist object
     */
    Twist& operator=(const roscomp::geometry_msgs::TwistWithCovariance& aRosTwist);

    /**
     * @brief Copy-assignment operator
     *
     * @param aRosTwist pointer to a ROS TwistWithCovariance message with values to copy
     * @return Twist& reference to the initialized Twist object
     */
    Twist& operator=(
      roscomp::CallbackPtr<roscomp::geometry_msgs::TwistWithCovariance> aRosTwist);

    /**
     * @brief To ROS message
     *
     * @return roscomp::geometry_msgs::Twist message
     */
    roscomp::geometry_msgs::TwistWithCovariance toRos() const
    {
        roscomp::geometry_msgs::TwistWithCovariance ros_twist;

        ros_twist.twist.linear.x = linear.x();
        ros_twist.twist.linear.y = linear.y();
        ros_twist.twist.linear.z = linear.z();

        ros_twist.twist.angular.x = angular.x();
        ros_twist.twist.angular.y = angular.y();
        ros_twist.twist.angular.z = angular.z();

        ros_twist.covariance = twist_covariance.template toArray<double>();

        return ros_twist;
    }
};

} // namespace data
} // namespace adx

#endif // ADX_DATA_TWIST_HPP
