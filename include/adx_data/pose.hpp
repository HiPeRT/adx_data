#ifndef ADX_DATA_POSE_HPP
#define ADX_DATA_POSE_HPP

#include <roscomp/msgs/geometry_msgs.hpp>

#include "adx_data/header.hpp"
#include "adx_data/covariance.hpp"
#include "adx_data/point.hpp"
#include "adx_data/quaternion.hpp"

namespace adx {
namespace data {
namespace internal {

constexpr unsigned long covariance_dimension[4] = { 0, 1, 3, 6 };

/**
 * @brief 6D PosePosition
 *
 * A representation of pose in free space, composed of position and orientation.
 */
template<typename Type, unsigned long Size>
struct PosePosition : public Point<Type, Size>
{
    static_assert(Size >= 2, "Minimum size for PosePosition is 2");
    static_assert(Size <= 3, "Maximum size for PosePosition is 3");

    /**
     * @brief inherit Point constructors
     */
    using Point<Type, Size>::Point;

    /**
     * @brief Construct a new PosePosition object
     */
    PosePosition() = default;
};

template<typename Type, unsigned long Size>
struct PoseOrientation
{
    static_assert(Size >= 2, "Minimum size for PoseOrientation is 2");
    static_assert(Size <= 3, "Maximum size for PoseOrientation is 3");

    /**
     * @brief The orientation part of the Pose
     */
    adx::data::Quaternion<Type> orientation = { static_cast<Type>(1),
                                                static_cast<Type>(0),
                                                static_cast<Type>(0),
                                                static_cast<Type>(0) };

    /**
     * @brief Construct a new PoseOrientation object
     *
     * Default constructor, this should just zero-initialize everything
     */
    PoseOrientation() = default;

    /**
     * @brief Construct a new PoseOrientation object
     * @param aRosQuaternion ROS Pose message
     */
    PoseOrientation(const roscomp::geometry_msgs::Quaternion& aRosQuaternion)
      : orientation(aRosQuaternion)
    {}

    /**
     * @brief Construct a new PoseOrientation object
     * @param aRosQuaternion pointer to ROS Pose message
     */
    PoseOrientation(roscomp::CallbackPtr<roscomp::geometry_msgs::Quaternion> aRosQuaternion)
      : PoseOrientation(*aRosQuaternion)
    {}

  protected:
    /**
     * @brief Convert this data to ROS Quaternion message
     * @return roscomp::geometry_msgs::Quaternion ROS message initialized from this class
     */
    roscomp::geometry_msgs::Quaternion toRos() const { return orientation.toRos(); }
};

template<typename Type>
struct PoseOrientation<Type, 2>
{
    /**
     * @brief Construct a new PoseOrientation object
     */
    PoseOrientation() = default;

    /**
     * @brief Construct a new PoseOrientation object
     * @param aRosQuaternion ROS Pose message
     */
    PoseOrientation(const roscomp::geometry_msgs::Quaternion& aRosQuaternion)
      : mYaw(adx::data::Quaternion<Type>(aRosQuaternion).getRPY()(2))
    {}

    /**
     * @brief Construct a new PoseOrientation object
     * @param aRosQuaternion pointer to ROS Pose message
     */
    PoseOrientation(roscomp::CallbackPtr<roscomp::geometry_msgs::Quaternion> aRosQuaternion)
      : PoseOrientation(*aRosQuaternion)
    {}

    /**
     * @brief Copy-assignment operator
     * @param aRosQuaternion a ROS Quaternion message
     */
    PoseOrientation& operator=(const roscomp::geometry_msgs::Quaternion& aRosQuaternion)
    {
        mYaw = adx::data::Quaternion<Type>(aRosQuaternion).getRPY()(2);

        return *this;
    }

    /**
     * @brief yaw accessor
     * @return Type& mutable reference to yaw variable
     */
    Type& yaw() { return mYaw; }

    /**
     * @brief yaw accessor
     * @return const Type& immutable reference to yaw variable
     */
    const Type& yaw() const { return mYaw; }

    Quaternion<Type> toQuaternion() const
    {
        Quaternion<Type> adx_quaternion;

        adx_quaternion.setRPY(0.0, 0.0, mYaw);

        return adx_quaternion;
    }

    /**
     * @brief Convert this data to ROS Quaternion message
     * @return roscomp::geometry_msgs::Quaternion ROS message initialized from this class
     */
    roscomp::geometry_msgs::Quaternion toRos() const { return toQuaternion().toRos(); }

  private:
    Type mYaw;
};

template<typename Type, unsigned long Size>
struct PoseBase
  : public PosePosition<Type, Size>
  , public PoseOrientation<Type, Size>
{
    static_assert(Size >= 2, "Minimum size for PoseBase is 2");
    static_assert(Size <= 3, "Maximum size for PoseBase is 3");

    /**
     * @brief inherit Point constructors
     */
    using PosePosition<Type, Size>::PosePosition;
    using PoseOrientation<Type, Size>::PoseOrientation;

    /**
     * @brief Construct a new PoseBase object
     */
    PoseBase() = default;

    /**
     * @brief Copy-construct a new PoseBase object
     * @param aRosPose ROS Pose message
     */
    PoseBase(const roscomp::geometry_msgs::Pose& aRosPose)
      : PosePosition<Type, Size>(aRosPose.position)
      , PoseOrientation<Type, Size>(aRosPose.orientation)
    {}

    /**
     * @brief Copy-construct a new PoseBase object
     * @param aRosPose a pointeer to ROS Pose message
     */
    PoseBase(roscomp::CallbackPtr<roscomp::geometry_msgs::Pose> aRosPose)
      : PoseBase(*aRosPose)
    {}

    /**
     * @brief Copy-construct a new PoseBase object
     * @param aRosPose ROS PoseStamped message
     */
    PoseBase(const roscomp::geometry_msgs::PoseStamped& aRosPose)
      : PosePosition<Type, Size>(aRosPose.pose.position)
      , PoseOrientation<Type, Size>(aRosPose.pose.orientation)
    {}

    /**
     * @brief Copy-construct a new PoseBase object
     * @param aRosPose a pointeer to ROS PoseStamped message
     */
    PoseBase(roscomp::CallbackPtr<roscomp::geometry_msgs::PoseStamped> aRosPose)
      : PoseBase(*aRosPose)
    {}

    /**
     * @brief Copy-construct a new PoseBase object
     * @param aRosPose ROS PoseWithCovariance message
     */
    PoseBase(const roscomp::geometry_msgs::PoseWithCovariance& aRosPose)
      : PosePosition<Type, Size>(aRosPose.pose.position)
      , PoseOrientation<Type, Size>(aRosPose.pose.orientation)
    {}

    /**
     * @brief Copy-construct a new PoseBase object
     * @param aRosPose a pointeer to ROS PoseWithCovariance message
     */
    PoseBase(roscomp::CallbackPtr<roscomp::geometry_msgs::PoseWithCovariance> aRosPose)
      : PoseBase(*aRosPose)
    {}

    /**
     * @brief Copy-assignment operator
     * @param aRosPose a ROS Pose message
     */
    PoseBase& operator=(const roscomp::geometry_msgs::Pose& aRosPose)
    {
        PosePosition<Type, Size>::operator=(aRosPose.position);
        PoseOrientation<Type, Size>::operator=(aRosPose.orientation);

        return *this;
    }

    /**
     * @brief Convert this data to ROS message
     * @return roscomp::geometry_msgs::Pose ROS message initialized from this class
     */
    roscomp::geometry_msgs::Pose toRos() const
    {
        roscomp::geometry_msgs::Pose ros_pose;
        ros_pose.position = PosePosition<Type, Size>::toRos();
        ros_pose.orientation = PoseOrientation<Type, Size>::toRos();

        return ros_pose;
    }
};

template<typename Type, unsigned long Size, bool HasHeader>
struct PoseHeader : virtual public PoseBase<Type, Size>
{
    /**
     * @brief inherit constructors
     */
    using PoseBase<Type, Size>::PoseBase;

    /**
     * @brief Construct a new PoseHeader object
     */
    PoseHeader() = default;

    /**
     * @brief Copy-construct a new PoseHeader object
     * @param aOther a PoseHeader to copy
     */
    // PoseHeader(const PoseHeader& aOther)
    //   : PoseBase<Type, Size>(static_cast<PoseBase<Type, Size>>(aOther))
    // {}

    /**
     * @brief Move-construct a new PoseHeader object
     * @param aOther a PoseHeader to move
     */
    // PoseHeader(PoseHeader&& aOther)
    //   : PoseBase<Type, Size>(static_cast<PoseBase<Type, Size>&&>(aOther))
    // {}

    /**
     * @brief Copy-assignment operator
     * @param aOther a PoseHeader with values to move
     * @return PoseHeader& reference to the initialized Header object
     */
    // PoseHeader& operator=(const PoseHeader& aOther)
    // {
    //     PoseBase<Type, Size>::operator=(static_cast<PoseBase<Type, Size>>(aOther));
    //     return *this;
    // }

    /**
     * @brief Convert this data to ROS message
     * @return roscomp::geometry_msgs::PoseStamped ROS message initialized from this class
     */
    roscomp::geometry_msgs::PoseStamped toRos() const
    {
        roscomp::geometry_msgs::PoseStamped ros_pose;
        ros_pose.pose.position = PosePosition<Type, Size>::toRos();
        ros_pose.pose.orientation = PoseOrientation<Type, Size>::toRos();

        return ros_pose;
    }
};

template<typename Type, unsigned long Size>
struct PoseHeader<Type, Size, true>
  : virtual public Header
  , virtual public PoseBase<Type, Size>
{
    /**
     * @brief inherit constructors
     */
    using PoseBase<Type, Size>::PoseBase;

    /**
     * @brief Construct a new PoseHeader object
     */
    PoseHeader() = default;

    /**
     * @brief Copy-construct a new PoseHeader object
     * @param aOther a PoseHeader to copy
     */
    // PoseHeader(const PoseHeader& aOther)
    //   : Header(static_cast<Header>(aOther))
    //   , PoseBase<Type, Size>(static_cast<PoseBase<Type, Size>>(aOther))
    // {}

    /**
     * @brief Construct a new PoseHeader object
     * @param aRosPose ROS Header message
     */
    PoseHeader(const roscomp::std_msgs::Header& aRosHeader)
      : Header(aRosHeader)
    {}

    /**
     * @brief Copy-construct a new PoseBase object
     * @param aRosPose ROS PoseStamped message
     */
    PoseHeader(const roscomp::geometry_msgs::PoseStamped& aRosPose)
      : Header(aRosPose.header)
      , PoseBase<Type, Size>(aRosPose.pose)
    {}

    /**
     * @brief Copy-construct a new PoseBase object
     * @param aRosPose a pointeer to ROS PoseStamped message
     */
    PoseHeader(roscomp::CallbackPtr<roscomp::geometry_msgs::PoseStamped> aRosPose)
      : Header(aRosPose->header)
      , PoseBase<Type, Size>(aRosPose->pose)
    {}

    /**
     * @brief Construct a new PoseHeader object
     * @param aRosPose ROS PoseWithCovarianceStamped message
     */
    PoseHeader(const roscomp::geometry_msgs::PoseWithCovarianceStamped& aRosPose)
      : Header(aRosPose.header)
      , PoseBase<Type, Size>(aRosPose.pose.pose)
    {}

    /**
     * @brief Construct a new PoseHeader object
     * @param aRosPose ROS PoseWithCovarianceStamped message
     */
    PoseHeader(roscomp::CallbackPtr<roscomp::geometry_msgs::PoseWithCovarianceStamped> aRosPose)
      : PoseHeader(*aRosPose)
    {}

    /**
     * @brief Copy-assignment operator
     *
     * @param aOther a PoseHeader with values to copy
     * @return PoseHeader& reference to the initialized PoseHeader object
     */
    // PoseHeader& operator=(const PoseHeader& aOther)
    // {
    //     Header::operator=(static_cast<Header>(aOther));
    //     PoseBase<Type, Size>::operator=(static_cast<PoseBase<Type, Size>>(aOther));
    //     return *this;
    // }

    /**
     * @brief Convert this data to ROS message
     * @return roscomp::geometry_msgs::PoseStamped ROS message initialized from this class
     */
    roscomp::geometry_msgs::PoseStamped toRos() const
    {
        roscomp::geometry_msgs::PoseStamped ros_pose;
        ros_pose.header = Header::toRos();
        ros_pose.pose.position = PosePosition<Type, Size>::toRos();
        ros_pose.pose.orientation = PoseOrientation<Type, Size>::toRos();

        return ros_pose;
    }
};

template<typename Type, unsigned long Size, bool HasCovariance>
struct PoseCovariance : virtual public PoseBase<Type, Size>
{

    /**
     * @brief inherit constructors
     */
    using PoseBase<Type, Size>::PoseBase;

    /**
     * @brief Construct a new PoseCovariance object
     */
    PoseCovariance() = default;

    /**
     * @brief Copy-construct a new PoseCovariance object
     * @param aOther a PoseCovariance to copy
     */
    // PoseCovariance(const PoseCovariance& aOther)
    //   : PoseBase<Type, Size>(static_cast<PoseBase<Type, Size>>(aOther))
    // {}

    /**
     * @brief Move-construct a new PoseCovariance object
     * @param aOther a PoseCovariance to move
     */
    // PoseCovariance(PoseCovariance&& aOther)
    //   : PoseBase<Type, Size>(static_cast<PoseBase<Type, Size>&&>(aOther))
    // {}

    /**
     * @brief Copy-assignment operator
     * @param aOther a Header with values to move
     * @return Header& reference to the initialized Header object
     */
    // PoseCovariance& operator=(const PoseCovariance& aOther)
    // {
    //     PoseBase<Type, Size>::operator=(static_cast<PoseBase<Type, Size>>(aOther));
    //     return *this;
    // }

    /**
     * @brief Copy-assignment operator
     * @param aRosPose ROS PoseWithCovariance message with values to copy
     * @return PoseCovariance& reference to the initialized PoseCovariance object
     */
    PoseCovariance& operator=(const roscomp::geometry_msgs::PoseWithCovariance& aRosPose)
    {
        PoseBase<Type, Size>::operator=(aRosPose.pose);

        return *this;
    }

    /**
     * @brief Convert this data to ROS message
     * @return roscomp::geometry_msgs::PoseWithCovariance ROS message initialized from this class
     */
    roscomp::geometry_msgs::PoseWithCovariance toRos() const
    {
        roscomp::geometry_msgs::PoseWithCovariance ros_pose;
        ros_pose.pose.position = PosePosition<Type, Size>::toRos();
        ros_pose.pose.orientation = PoseOrientation<Type, Size>::toRos();

        return ros_pose;
    }
};

template<typename Type, unsigned long Size>
struct PoseCovariance<Type, Size, true> : virtual public PoseBase<Type, Size>
{
    /**
     * @brief inherit constructors
     */
    using PoseBase<Type, Size>::PoseBase;

    /**
     * @brief The Covariance matrix
     * For 2D pose: x, y, z-rotation;
     * For 3D pose: x, y, z, x-rotation, y-rotation, z-rotation.
     */
    Covariance<Type, internal::covariance_dimension[Size]> pose_covariance = {};

    /**
     * @brief Construct a new PoseCovariance object
     */
    PoseCovariance() = default;

    /**
     * @brief Copy-construct a new PoseCovariance object
     * @param aOther a PoseCovariance to copy
     */
    // PoseCovariance(const PoseCovariance& aOther)
    //   : PoseBase<Type, Size>(static_cast<PoseBase<Type, Size>>(aOther))
    //   , pose_covariance(aOther.pose_covariance)
    // {}

    /**
     * @brief Construct a new PoseCovariance object
     * @param aRosPose ROS PoseWithCovariance message
     */
    PoseCovariance(const roscomp::geometry_msgs::PoseWithCovariance& aRosPose)
      : PoseBase<Type, Size>(aRosPose.pose)
      , pose_covariance(aRosPose.covariance)
    {}

    /**
     * @brief Construct a new PoseCovariance object
     * @param aRosPose pointer to ROS PoseWithCovariance message
     */
    PoseCovariance(roscomp::CallbackPtr<roscomp::geometry_msgs::PoseWithCovariance> aRosPose)
      : PoseBase<Type, Size>(aRosPose->pose)
      , pose_covariance(aRosPose->covariance)
    {}

    /**
     * @brief Copy-assignment operator
     *
     * @param aOther a PoseCovariance with values to copy
     * @return PoseCovariance& reference to the initialized PoseCovariance object
     */
    // PoseCovariance& operator=(const PoseCovariance& aOther)
    // {
    //     PoseBase<Type, Size>::operator=(static_cast<PoseBase<Type, Size>>(aOther));
    //     pose_covariance = aOther.pose_covariance;
    //     return *this;
    // }

    /**
     * @brief Copy-assignment operator
     * @param aRosPose ROS PoseWithCovariance message with values to copy
     * @return PoseCovariance& reference to the initialized PoseCovariance object
     */
    PoseCovariance& operator=(const roscomp::geometry_msgs::PoseWithCovariance& aRosPose)
    {
        PoseBase<Type, Size>::operator=(aRosPose.pose);
        pose_covariance = aRosPose.covariance;

        return *this;
    }

    /**
     * @brief Convert this data to ROS message
     * @return roscomp::geometry_msgs::PoseWithCovariance ROS message initialized from this class
     */
    roscomp::geometry_msgs::PoseWithCovariance toRos() const
    {
        roscomp::geometry_msgs::PoseWithCovariance ros_pose;
        ros_pose.pose.position = PosePosition<Type, Size>::toRos();
        ros_pose.pose.orientation = PoseOrientation<Type, Size>::toRos();
        ros_pose.covariance = pose_covariance.template toArray<double>();

        return ros_pose;
    }
};

template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
struct PoseHeaderCovariance
  : public PoseHeader<Type, Size, HasHeader>
  , virtual public PoseBase<Type, Size>
  , public PoseCovariance<Type, Size, HasCovariance>
{
    /**
     * @brief inherit constructors
     */
    using PoseHeader<Type, Size, HasHeader>::PoseHeader;
    using PoseBase<Type, Size>::PoseBase;
    using PoseCovariance<Type, Size, HasCovariance>::PoseCovariance;

    /**
     * @brief Construct a new PoseHeaderCovariance object
     */
    PoseHeaderCovariance() = default;

    // PoseHeaderCovariance(const PoseHeaderCovariance& aOther)
    //   : PoseBase<Type, Size>(static_cast<PoseBase<Type, Size>>(aOther))
    //   , PoseHeader<Type, Size, HasHeader>(static_cast<PoseHeader<Type, Size, HasHeader>>(aOther))
    //   , PoseCovariance<Type, Size, HasCovariance>(
    //       static_cast<PoseCovariance<Type, Size, HasCovariance>>(aOther))
    // {}

    /**
     * @brief Construct a new PoseHeaderCovariance object
     * @param aRosPose ROS PoseStamped message
     */
    PoseHeaderCovariance(const roscomp::geometry_msgs::PoseStamped& aRosPose)
      : PoseBase<Type, Size>(aRosPose.pose)
    {}

    /**
     * @brief Construct a new PoseHeaderCovariance object
     * @param aRosPose ROS PoseStamped message
     */
    PoseHeaderCovariance(const roscomp::geometry_msgs::PoseWithCovarianceStamped& aRosPose)
      : PoseBase<Type, Size>(aRosPose.pose.pose)
      , PoseCovariance<Type, Size, HasCovariance>(aRosPose.pose)
    {}

    /**
     * @brief Construct a new PoseHeaderCovariance object
     * @param aRosPose ROS PoseStamped message
     */
    PoseHeaderCovariance(
      roscomp::CallbackPtr<roscomp::geometry_msgs::PoseWithCovarianceStamped> aRosPose)
      : PoseHeaderCovariance(*aRosPose)
    {}

    /**
     * @brief Copy-assignment operator
     * @param aOther a Header with values to move
     * @return Header& reference to the initialized Header object
     */
    // PoseHeaderCovariance& operator=(const PoseHeaderCovariance& aOther)
    // {
    //     PoseHeader<Type, Size, HasHeader>::operator=(
    //       static_cast<PoseHeader<Type, Size, HasHeader>>(aOther));
    //     PoseBase<Type, Size>::operator=(static_cast<PoseBase<Type, Size>>(aOther));
    //     PoseCovariance<Type, Size, HasCovariance>::operator=(
    //       static_cast<PoseCovariance<Type, Size, HasCovariance>>(aOther));
    //     return *this;
    // }

    /**
     * @brief Copy-assignment operator
     * @param aRosPose ROS PoseStamped message with values to copy
     * @return PoseBase& reference to the initialized PoseBase object
     */
    // PoseHeaderCovariance& operator=(const roscomp::geometry_msgs::PoseStamped& aRosPose)
    // {
    //     PoseBase<Type, Size>::operator=(aRosPose);
    //     PoseHeader<Type, Size, HasHeader>::operator=(aRosPose);
    //     // PoseCovariance<Type, Size, HasCovariance>::operator=(aRosPose.pose);
    //     return *this;
    // }

    /**
     * @brief Convert this data to ROS message
     * @return roscomp::geometry_msgs::PoseWithCovarianceStamped ROS message initialized from this
     * class
     */
    roscomp::geometry_msgs::PoseWithCovarianceStamped toRos() const
    {
        roscomp::geometry_msgs::PoseWithCovarianceStamped ros_pose;
        ros_pose.pose.pose.position = PosePosition<Type, Size>::toRos();
        ros_pose.pose.pose.orientation = PoseOrientation<Type, Size>::toRos();
        if constexpr (HasCovariance) {
            ros_pose.pose.covariance =
              PoseCovariance<Type, Size, HasCovariance>::pose_covariance.template toArray<double>();
        }

        return ros_pose;
    }
};

template<typename Type, unsigned long Size>
struct PoseHeaderCovariance<Type, Size, true, false>
  : public PoseHeader<Type, Size, true>
  , virtual public PoseBase<Type, Size>
  , public PoseCovariance<Type, Size, false>
{
    /**
     * @brief inherit constructors
     */
    using PoseBase<Type, Size>::PoseBase;
    using PoseHeader<Type, Size, true>::PoseHeader;
    using PoseCovariance<Type, Size, false>::PoseCovariance;

    /**
     * @brief Construct a new PoseHeaderCovariance object
     */
    PoseHeaderCovariance() = default;

    // PoseHeaderCovariance(const PoseHeaderCovariance& aOther)
    //   : Header(static_cast<Header>(aOther))
    //   , PoseBase<Type, Size>(static_cast<PoseBase<Type, Size>>(aOther))
    //   , PoseHeader<Type, Size, true>(static_cast<PoseHeader<Type, Size, true>>(aOther))
    //   , PoseCovariance<Type, Size, false>(static_cast<PoseCovariance<Type, Size, false>>(aOther))
    // {}

    /**
     * @brief Copy-assignment operator
     * @param aOther a Header with values to move
     * @return Header& reference to the initialized Header object
     */
    // PoseHeaderCovariance& operator=(const PoseHeaderCovariance& aOther)
    // {
    //     PoseHeader<Type, Size, true>::operator=(static_cast<PoseHeader<Type, Size,
    //     true>>(aOther)); PoseBase<Type, Size>::operator=(static_cast<PoseBase<Type,
    //     Size>>(aOther)); PoseCovariance<Type, Size, false>::operator=(
    //       static_cast<PoseCovariance<Type, Size, false>>(aOther));
    //     return *this;
    // }

    /**
     * @brief Convert this data to ROS message
     * @return roscomp::geometry_msgs::PoseWithCovarianceStamped ROS message initialized from this
     * class
     */
    roscomp::geometry_msgs::PoseWithCovarianceStamped toRos() const
    {
        roscomp::geometry_msgs::PoseWithCovarianceStamped ros_pose;
        ros_pose.header = Header::toRos();
        ros_pose.pose.pose.position = PosePosition<Type, Size>::toRos();
        ros_pose.pose.pose.orientation = PoseOrientation<Type, Size>::toRos();

        return ros_pose;
    }
};

template<typename Type, unsigned long Size>
struct PoseHeaderCovariance<Type, Size, true, true>
  : public PoseHeader<Type, Size, true>
  , virtual public PoseBase<Type, Size>
  , public PoseCovariance<Type, Size, true>
{
    /**
     * @brief inherit constructors
     */
    using PoseHeader<Type, Size, true>::PoseHeader;
    using PoseCovariance<Type, Size, true>::PoseCovariance;

    /**
     * @brief Construct a new PoseHeaderCovariance object
     */
    PoseHeaderCovariance() = default;

    /**
     * @brief Copy-construct a new PoseHeaderCovariance object
     * @param aOther a PoseHeaderCovariance to copy
     */
    // PoseHeaderCovariance(const PoseHeaderCovariance& aOther)
    //   : Header(static_cast<Header>(aOther))
    //   , PoseBase<Type, Size>(static_cast<PoseBase<Type, Size>>(aOther))
    //   , PoseHeader<Type, Size, true>(static_cast<PoseHeader<Type, Size, true>>(aOther))
    //   , PoseCovariance<Type, Size, true>(static_cast<PoseCovariance<Type, Size, true>>(aOther))
    // {}

    /**
     * @brief Construct a new PoseCovariance object
     * @param aRosPose ROS PoseWithCovariance message
     */
    PoseHeaderCovariance(const roscomp::geometry_msgs::PoseWithCovariance& aRosPose)
      : PoseHeader<Type, Size, true>(aRosPose.pose)
      , PoseCovariance<Type, Size, true>(aRosPose)
    {}

    /**
     * @brief Construct a new Pose object
     * @param aRosPose ROS PoseWithCovarianceStamped message
     */
    PoseHeaderCovariance(const roscomp::geometry_msgs::PoseWithCovarianceStamped& aRosPose)
      : PoseBase<Type, Size>(aRosPose.pose.pose)
      , PoseHeader<Type, Size, true>(aRosPose.header)
      , PoseCovariance<Type, Size, true>(aRosPose.pose)
    {}

    /**
     * @brief Construct a new Pose object
     * @param aRosPose pointer to ROS PoseWithCovarianceStamped message
     */
    PoseHeaderCovariance(
      roscomp::CallbackPtr<roscomp::geometry_msgs::PoseWithCovarianceStamped> aRosPose)
      : PoseHeaderCovariance(*aRosPose)
    {}

    /**
     * @brief Copy-assignment operator
     * @param aOther a Header with values to move
     * @return Header& reference to the initialized Header object
     */
    // PoseHeaderCovariance& operator=(const PoseHeaderCovariance& aOther)
    // {
    //     PoseHeader<Type, Size, true>::operator=(static_cast<PoseHeader<Type, Size,
    //     true>>(aOther)); PoseBase<Type, Size>::operator=(static_cast<PoseBase<Type,
    //     Size>>(aOther)); PoseCovariance<Type, Size, true>::operator=(
    //       static_cast<PoseCovariance<Type, Size, true>>(aOther));
    //     return *this;
    // }

    /**
     * @brief Convert this data to ROS message
     * @return roscomp::geometry_msgs::PoseWithCovarianceStamped ROS message initialized from this
     * class
     */
    roscomp::geometry_msgs::PoseWithCovarianceStamped toRos() const
    {
        roscomp::geometry_msgs::PoseWithCovarianceStamped ros_pose;
        ros_pose.header = Header::toRos();
        ros_pose.pose.pose.position = PosePosition<Type, Size>::toRos();
        ros_pose.pose.pose.orientation = PoseOrientation<Type, Size>::toRos();
        ros_pose.pose.covariance =
          PoseCovariance<Type, Size, true>::pose_covariance.template toArray<double>();

        return ros_pose;
    }
};

} // namespace internal

template<typename Type, unsigned long Size, bool HasHeader, bool HasCovariance>
struct Pose : public internal::PoseHeaderCovariance<Type, Size, HasHeader, HasCovariance>
{
    static_assert(Size >= 2, "Minimum size for Pose is 2");
    static_assert(Size <= 3, "Maximum size for Pose is 3");

    /**
     * @brief inherit Point constructors
     */
    using internal::PoseHeaderCovariance<Type, Size, HasHeader, HasCovariance>::
      PoseHeaderCovariance;

    /**
     * @brief Construct a new Pose object
     */
    Pose() = default;

    /**
     * @brief Construct a new Pose object
     * @param aRosPose ROS PoseStamped message
     */
    Pose(const roscomp::geometry_msgs::PoseStamped& aRosPose)
      : Header(aRosPose.header)
      , internal::PoseBase<Type, Size>(aRosPose.pose)
      , internal::PoseHeaderCovariance<Type, Size, HasHeader, HasCovariance>(aRosPose)
    {}

    /**
     * @brief Construct a new Pose object
     * @param aRosPose ROS PoseStamped message
     */
    Pose(roscomp::CallbackPtr<roscomp::geometry_msgs::PoseStamped> aRosPose)
      : Pose(*aRosPose)
    {}

    /**
     * @brief Construct a new Pose object
     * @param aRosPose ROS PoseWithCovariance message
     */
    Pose(const roscomp::geometry_msgs::PoseWithCovariance& aRosPose)
      : internal::PoseBase<Type, Size>(aRosPose.pose)
      , internal::PoseHeaderCovariance<Type, Size, HasHeader, HasCovariance>(aRosPose)
    {}

    /**
     * @brief Construct a new Pose object
     * @param aRosPose ROS PoseWithCovariance message
     */
    Pose(roscomp::CallbackPtr<roscomp::geometry_msgs::PoseWithCovariance> aRosPose)
      : Pose(*aRosPose)
    {}

    /**
     * @brief Construct a new Pose object
     * @param aRosPose ROS PoseWithCovarianceStamped message
     */
    Pose(const roscomp::geometry_msgs::PoseWithCovarianceStamped& aRosPose)
      : Header(aRosPose.header)
      , internal::PoseBase<Type, Size>(aRosPose.pose.pose)
      , internal::PoseHeaderCovariance<Type, Size, HasHeader, HasCovariance>(aRosPose)
    {}

    /**
     * @brief Construct a new Pose object
     * @param aRosPose ROS PoseWithCovarianceStamped message
     */
    Pose(roscomp::CallbackPtr<roscomp::geometry_msgs::PoseWithCovarianceStamped> aRosPose)
      : Pose(*aRosPose)
    {}

    /**
     * @brief Copy-assignment operator
     * @param aRosPose ROS PoseWithCovariance message with values to copy
     * @return Pose& reference to the initialized Pose object
     */
    Pose& operator=(const roscomp::geometry_msgs::PoseWithCovariance& aRosPose)
    {
        internal::PoseCovariance<Type, Size, HasCovariance>::operator=(aRosPose);

        return *this;
    }
};

template<typename Type, unsigned long Size, bool HasCovariance>
struct Pose<Type, Size, false, HasCovariance>
  : public internal::PoseHeaderCovariance<Type, Size, false, HasCovariance>
{
    static_assert(Size >= 2, "Minimum size for Pose is 2");
    static_assert(Size <= 3, "Maximum size for Pose is 3");

    /**
     * @brief inherit Point constructors
     */
    using internal::PoseHeaderCovariance<Type, Size, false, HasCovariance>::PoseHeaderCovariance;

    /**
     * @brief Construct a new Pose object
     */
    Pose() = default;

    /**
     * @brief Copy-construct a new Pose object
     * @param aOther a Pose to copy
     */
    // Pose(const Pose& aOther)
    //   : internal::PoseBase<Type, Size>(static_cast<internal::PoseBase<Type, Size>>(aOther))
    //   , internal::PoseHeaderCovariance<Type, Size, false, HasCovariance>(
    //       static_cast<internal::PoseHeaderCovariance<Type, Size, false, HasCovariance>>(aOther))
    // {}

    /**
     * @brief Move-construct a new Pose object
     * @param aOther a Pose to move
     */
    // Pose(Pose&& aOther)
    //   : internal::PoseHeaderCovariance<Type, Size, false, HasCovariance>(
    //       static_cast<internal::PoseHeaderCovariance<Type, Size, false,
    //       HasCovariance>&&>(aOther))
    // {}

    /**
     * @brief Construct a new Pose object
     * @param aRosPose ROS PoseStamped message
     */
    Pose(const roscomp::geometry_msgs::PoseStamped& aRosPose)
      : internal::PoseBase<Type, Size>(aRosPose.pose)
      , internal::PoseHeaderCovariance<Type, Size, false, HasCovariance>(aRosPose)
    {}

    /**
     * @brief Construct a new Pose object
     * @param aRosPose pointer to ROS PoseStamped message
     */
    Pose(roscomp::CallbackPtr<roscomp::geometry_msgs::PoseStamped> aRosPose)
      : Pose(*aRosPose)
    {}

    /**
     * @brief Construct a new Pose object
     * @param aRosPose ROS PoseWithCovariance message
     */
    Pose(const roscomp::geometry_msgs::PoseWithCovariance& aRosPose)
      : internal::PoseBase<Type, Size>(aRosPose.pose)
      , internal::PoseHeaderCovariance<Type, Size, false, HasCovariance>(aRosPose)
    {}

    /**
     * @brief Construct a new Pose object
     * @param aRosPose pointer to ROS PoseWithCovariance message
     */
    Pose(roscomp::CallbackPtr<roscomp::geometry_msgs::PoseWithCovariance> aRosPose)
      : Pose(*aRosPose)
    {}

    /**
     * @brief Construct a new Pose object
     * @param aRosPose ROS PoseWithCovarianceStamped message
     */
    Pose(const roscomp::geometry_msgs::PoseWithCovarianceStamped& aRosPose)
      : internal::PoseBase<Type, Size>(aRosPose.pose.pose)
      , internal::PoseHeaderCovariance<Type, Size, false, HasCovariance>(aRosPose)
    {}

    /**
     * @brief Construct a new Pose object
     * @param aRosPose ROS PoseWithCovarianceStamped message
     */
    Pose(roscomp::CallbackPtr<roscomp::geometry_msgs::PoseWithCovarianceStamped> aRosPose)
      : Pose(*aRosPose)
    {}

    /**
     * @brief Copy-assignment operator
     * @param aRosPose ROS PoseWithCovariance message with values to copy
     * @return Pose& reference to the initialized Pose object
     */
    Pose& operator=(const roscomp::geometry_msgs::PoseWithCovariance& aRosPose)
    {
        internal::PoseCovariance<Type, Size, HasCovariance>::operator=(aRosPose);

        return *this;
    }

    /**
     * @brief Move-assignment operator
     * @param aOther a Pose to move
     */
    // Pose& operator=(Pose&& aOther)
    // {
    //     if (this != &aOther) {
    //         internal::PoseHeaderCovariance<Type, Size, false, HasCovariance>::operator=(
    //           static_cast<internal::PoseHeaderCovariance<Type, Size, false, HasCovariance>&&>(
    //             aOther));
    //     }
    //     return *this;
    // }
};

using Pose2f = Pose<float, 2, true, true>;
using Pose2d = Pose<double, 2, true, true>;
using Pose3f = Pose<float, 3, true, true>;
using Pose3d = Pose<double, 3, true, true>;

} // namespace data
} // namespace adx

#endif // ADX_DATA_POSE_HPP
