#ifndef ADX_DATA_OBSTACLE_HPP
#define ADX_DATA_OBSTACLE_HPP

#include <roscomp/msgs/adx_msgs.hpp>

#include "adx_data/covariance.hpp"
#include "adx_data/pose.hpp"
#include "adx_data/twist.hpp"
#include "adx_data/vector.hpp"

namespace adx {
namespace data {

/**
 * @brief Cube obstacle
 *
 * This struct represents a cube obstacle.
 */
struct Obstacle
  : public Pose3d
  , public Twist
{
    /**
     * The cube half-lenghts, the linear combination of position and size gives us the cube
     * vertices
     */
    adx::data::Vector3d size = {};

    /**
     * @brief Construct a new Obstacle object
     *
     * The default constructor should just zero-initialize everything.
     */
    Obstacle() = default;

    /**
     * @brief Construct a new Obstacle object
     *
     * @param aRosObstacle a ROS obstacle message
     */
    Obstacle(const roscomp::adx_msgs::Obstacle& aRosObstacle);

    /**
     * @brief Construct a new Obstacle object
     *
     * @param aRosObstacle a ROS obstacle message
     */
    Obstacle(roscomp::CallbackPtr<roscomp::adx_msgs::Obstacle> aRosObstacle);

    /**
     * @brief Copy-assignment operator
     *
     * @param aRosSize a ROS Size message with values to copy
     * @return Obstacle& reference to the initialized Obstacle object
     */
    Obstacle& operator=(const roscomp::geometry_msgs::Vector3& aRosSize);

    /**
     * @brief Copy-assignment operator
     *
     * @param aRosSize pointer to a ROS Size message with values to copy
     * @return Obstacle& reference to the initialized Obstacle object
     */
    Obstacle& operator=(roscomp::CallbackPtr<roscomp::geometry_msgs::Vector3> aRosSize);

    /**
     * @brief Copy-assignment operator
     *
     * @param aRosObstacle a ROS Obstacle message with values to copy
     * @return Obstacle& reference to the initialized Obstacle object
     */
    Obstacle& operator=(const roscomp::adx_msgs::Obstacle& aRosObstacle);

    /**
     * @brief Copy-assignment operator
     *
     * @param aRosObstacle pointer to a ROS obstacle message with values to copy
     * @return Obstacle& reference to the initialized Obstacle object
     */
    Obstacle& operator=(roscomp::CallbackPtr<roscomp::adx_msgs::Obstacle> aRosObstacle);
};

} // nmespace data
} // namespace adx

#endif // ADX_DATA_OBSTACLE_HPP
