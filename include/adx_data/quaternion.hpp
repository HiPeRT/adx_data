#ifndef ADX_DATA_QUATERNION_HPP
#define ADX_DATA_QUATERNION_HPP

#include <Eigen/Geometry>

#include <roscomp/msgs/geometry_msgs.hpp>

namespace adx {
namespace data {

/**
 * @brief Orientation
 *
 * This represents an orientation in free space in quaternion form.
 */
template<typename Type>
struct Quaternion : public Eigen::Quaternion<Type>
{

    /**
     * @brief Inherit constructors
     */
    using Eigen::Quaternion<Type>::Quaternion;

    /**
     * @brief Construct a new Quaternion object
     *
     * @param aRosQuaternion a ROS Quaternion message
     */
    Quaternion(const roscomp::geometry_msgs::Quaternion& aRosQuaternion)
    {
        *this = aRosQuaternion;
    }

    /**
     * @brief Construct a new Quaternion object
     *
     * @param aRosQuaternion pointer to a ROS Quaternion message
     */
    Quaternion(roscomp::CallbackPtr< roscomp::geometry_msgs::Quaternion> aRosQuaternion)
      : Quaternion(*aRosQuaternion)
    {}

    /**
     * @brief Copy-assignment operator
     *
     * @param aRosQuaternion a ROS Quaternion message with values to copy
     * @return Quaternion& reference to the initialized Quaternion object
     */
    Quaternion& operator=(const roscomp::geometry_msgs::Quaternion& aRosQuaternion)
    {
        Eigen::Quaternion<Type>::x() = static_cast<Type>(aRosQuaternion.x);
        Eigen::Quaternion<Type>::y() = static_cast<Type>(aRosQuaternion.y);
        Eigen::Quaternion<Type>::z() = static_cast<Type>(aRosQuaternion.z);
        Eigen::Quaternion<Type>::w() = static_cast<Type>(aRosQuaternion.w);

        return *this;
    }

    /**
     * @brief Copy-assignment operator
     *
     * @param aRosQuaternion pointer to a ROS Quaternion message with values to copy
     * @return Quaternion& reference to the initialized Quaternion object
     */
    Quaternion& operator=(roscomp::CallbackPtr< roscomp::geometry_msgs::Quaternion> aRosQuaternion)
    {
        return operator=(aRosQuaternion);
    }

    /**
     * @brief set orientation to Roll, Pitch, Yaw
     *
     * @param aRoll the rotation around the X axis
     * @param aPitch the rotation around the Y axis
     * @param aYaw the rotation around the Z axis
     */
    void setRPY(const Type& aRoll, const Type& aPitch, const Type& aYaw)
    {
        Eigen::Quaternion<Type>::operator=(
          Eigen::AngleAxis<Type>(aRoll, Eigen::Matrix<Type, 3, 1>::UnitX()) *
          Eigen::AngleAxis<Type>(aPitch, Eigen::Matrix<Type, 3, 1>::UnitY()) *
          Eigen::AngleAxis<Type>(aYaw, Eigen::Matrix<Type, 3, 1>::UnitZ()));
    }

    /**
     * @brief return the Roll, Pitch and Yaw
     *
     * @return Eigen::Matrix<Type, 3, 1> Vector containing roll, pitch and yaw
     */
    Eigen::Matrix<Type, 3, 1> getRPY() const
    {
        return Eigen::Quaternion<Type>::toRotationMatrix().eulerAngles(0, 1, 2);
    }

    /**
     * @brief Convert this data to ROS message
     *
     * @return roscomp::geometry_msgs::Quaternion ROS message initialized from this class
     */
    roscomp::geometry_msgs::Quaternion toRos() const
    {
        roscomp::geometry_msgs::Quaternion ros_quaternion;

        ros_quaternion.x = static_cast<double>(Eigen::Quaternion<Type>::x());
        ros_quaternion.y = static_cast<double>(Eigen::Quaternion<Type>::y());
        ros_quaternion.z = static_cast<double>(Eigen::Quaternion<Type>::z());
        ros_quaternion.w = static_cast<double>(Eigen::Quaternion<Type>::w());

        return ros_quaternion;
    }
};

using Quaternionf = Quaternion<float>;
using Quaterniond = Quaternion<double>;

} // namespace data
} // namespace adx

#endif // ADX_DATA_QUATERNION_HPP
