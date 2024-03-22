#ifndef ADX_DATA_POINT_HPP
#define ADX_DATA_POINT_HPP

#include <roscomp/msgs/geometry_msgs.hpp>

#include "adx_data/vector.hpp"

namespace adx {
namespace data {

/**
 * @brief 3D position
 *
 * This contains the position of a point in free space.
 */
template<typename Type, unsigned long Size>
struct Point
{
    /**
     * The 3D vector representing the 3D position
     */
    adx::data::Vector<Type, Size> position = {};

    /**
     * @brief Construct a new Point object
     *
     * Default constructor, this should just zero-initialize the position
     */
    Point() = default;

    /**
     * @brief Construct a new Point object
     *
     * @param aRosPoint a ROS Point message
     */
    Point(const roscomp::geometry_msgs::Point& aRosPoint)
      : position(aRosPoint)
    {}

    /**
     * @brief Construct a new Point object
     *
     * @param aRosPoint pointer to a ROS Point message
     */
    Point(roscomp::CallbackPtr<roscomp::geometry_msgs::Point> aRosPoint)
      : Point(*aRosPoint)
    {}

    /**
     * @brief Copy-assignment operator
     *
     * @param aRosPoint a ROS Point message with values to copy
     * @return Point& reference to the initialized Point object
     */
    Point& operator=(const roscomp::geometry_msgs::Point& aRosPoint)
    {
        position = aRosPoint;

        return *this;
    }

    /**
     * @brief Copy-assignment operator
     *
     * @param aRosPoint pointer to a ROS Point message with values to copy
     * @return Point& reference to the initialized Point object
     */
    Point& operator=(roscomp::CallbackPtr<roscomp::geometry_msgs::Point> aRosPoint)
    {
        return operator=(*aRosPoint);
    }

    /**
     * @brief Convert this data to ROS message
     *
     * @return geometry_msgs::Point ROS message initialized from this class
     */
    roscomp::geometry_msgs::Point toRos() const
    {
        roscomp::geometry_msgs::Point ros_point;

        ros_point.x = position.x();
        if constexpr (Size >= 2) {
            ros_point.y = position.y();
        } else {
            ros_point.y = 0.0;
        }
        if constexpr (Size >= 3) {
            ros_point.z = position.z();
        } else {
            ros_point.z = 0.0;
        }

        return ros_point;
    }
};

using Point2f = Point<float, 2>;
using Point2d = Point<double, 2>;
using Point3f = Point<float, 3>;
using Point3d = Point<double, 3>;

} // nmespace data
} // namespace adx

#endif // ADX_DATA_POINT_HPP
