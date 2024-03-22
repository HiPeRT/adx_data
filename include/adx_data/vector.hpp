#ifndef ADX_DATA_VECTOR3_HPP
#define ADX_DATA_VECTOR3_HPP

#include <Eigen/Core>

#include <roscomp/msgs/geometry_msgs.hpp>

namespace adx {
namespace data {

template<typename Type, unsigned long Size>
struct Vector : public Eigen::Matrix<Type, Size, 1>
{
    /**
     * @brief Inherit constructor
     */
    using Eigen::Matrix<Type, Size, 1>::Matrix;

    /**
     * @brief Construct a new Vector object
     *
     * @param aRosPoint a ROS Point message
     */
    Vector(const roscomp::geometry_msgs::Point& aRosPoint) { *this = aRosPoint; }

    /**
     * @brief Construct a new Vector object
     *
     * @param aRosPoint pointer to a ROS Point message
     */
    Vector(roscomp::CallbackPtr<roscomp::geometry_msgs::Point> aRosPoint)
      : Vector(*aRosPoint)
    {}

    /**
     * @brief Construct a new Vector object
     *
     * @param aRosVector a ROS Vector3 message with values to copy
     */
    Vector(const roscomp::geometry_msgs::Vector3& aRosVector) { *this = aRosVector; }

    /**
     * @brief Construct a new Vector object
     *
     * @param aRosVector a ROS Vector3 message with values to copy
     */
    Vector(roscomp::CallbackPtr<roscomp::geometry_msgs::Vector3> aRosVector)
      : Vector(*aRosVector)
    {}

    /**
     * @brief Copy-assignment operator
     *
     * @param aRosPoint a ROS Point message with values to copy
     * @return Vector& reference to the initialized Vector object
     */
    Vector& operator=(const roscomp::geometry_msgs::Point& aRosPoint)
    {
        Eigen::Matrix<Type, Size, 1>::x() = aRosPoint.x;
        if constexpr (Size >= 2) {
            Eigen::Matrix<Type, Size, 1>::y() = aRosPoint.y;
        }
        if constexpr (Size >= 3) {
            Eigen::Matrix<Type, Size, 1>::z() = aRosPoint.z;
        }

        return *this;
    }

    /**
     * @brief Copy-assignment operator
     *
     * @param aRosPoint pointer to a ROS Point message with values to copy
     * @return Vector& reference to the initialized Vector object
     */
    Vector& operator=(roscomp::CallbackPtr<roscomp::geometry_msgs::Point> aRosPoint)
    {
        return operator=(*aRosPoint);
    }

    /**
     * @brief Copy-assignment operator
     *
     * This method copies the 3D vector from a ROS message
     *
     * @param aRosVector a ROS Vector3 message
     * @return Vector& reference to the initialized TimeStamp object
     */
    Vector& operator=(const roscomp::geometry_msgs::Vector3& aRosVector)
    {
        Eigen::Matrix<Type, Size, 1>::x() = aRosVector.x;
        if constexpr (Size >= 2) {
            Eigen::Matrix<Type, Size, 1>::y() = aRosVector.y;
        }
        if constexpr (Size >= 3) {
            Eigen::Matrix<Type, Size, 1>::z() = aRosVector.z;
        }

        return *this;
    }

    /**
     * @brief Copy-assignment operator
     *
     * This method copies the 3D vector from a ROS message
     *
     * @param aRosVector a ROS Vector3 message
     * @return Vector& reference to the initialized TimeStamp object
     */
    Vector& operator=(roscomp::CallbackPtr<roscomp::geometry_msgs::Vector3> aRosVector)
    {
        return operator=(*aRosVector);
    }

    /**
     * @brief Convert this data to ROS message
     *
     * @return roscomp::geometry_msgs::Vector3 ROS message initialized from this class
     */
    roscomp::geometry_msgs::Vector3 toRos() const
    {
        roscomp::geometry_msgs::Vector3 ros_vector;

        ros_vector.x = Eigen::Matrix<Type, Size, 1>::x();
        if constexpr (Size >= 2) {
            ros_vector.y = Eigen::Matrix<Type, Size, 1>::y();
        } else {
            ros_vector.y = 0.0;
        }
        if constexpr (Size >= 3) {
            ros_vector.z = Eigen::Matrix<Type, Size, 1>::z();
        } else {
            ros_vector.z = 0.0;
        }

        return ros_vector;
    }
};

using Vector2f = Vector<float, 2>;
using Vector2d = Vector<double, 2>;
using Vector3f = Vector<float, 3>;
using Vector3d = Vector<double, 3>;

} // namespace data
} // namespace adx

#endif // ADX_DATA_VECTOR3_HPP
