#include "adx_data/odometry.hpp"

namespace adx {
namespace data {

Odometry::Odometry(const roscomp::nav_msgs::Odometry& aRosOdometry)
  : Header(aRosOdometry.header)
  , PoseBase(aRosOdometry.pose.pose)
  , Pose3d(aRosOdometry.pose)
  , Twist(aRosOdometry.twist)
{}

Odometry::Odometry(roscomp::CallbackPtr<roscomp::nav_msgs::Odometry> aRosOdometry)
  : Odometry(*aRosOdometry)
{}

Odometry& Odometry::operator=(const Odometry& aOdometry)
{
    Header::operator=(static_cast<Header>(aOdometry));
    Pose3d::operator=(static_cast<Pose3d>(aOdometry));
    Twist::operator=(static_cast<Twist>(aOdometry));

    return *this;
}

Odometry& Odometry::operator=(Odometry&& aOther)
{
    Header::operator=(static_cast<Header&&>(aOther));
    Pose3d::operator=(static_cast<Pose3d&&>(aOther));
    Twist::operator=(static_cast<Twist&&>(aOther));

    return *this;
}

Odometry& Odometry::operator=(const roscomp::nav_msgs::Odometry& aRosOdometry)
{
    Header::operator=(aRosOdometry.header);
    Pose3d::operator=(aRosOdometry.pose);
    Twist::operator=(aRosOdometry.twist);

    return *this;
}

Odometry& Odometry::operator=(roscomp::CallbackPtr<roscomp::nav_msgs::Odometry> aRosOdometry)
{
    return operator=(*aRosOdometry);
}

} // namespace data
} // namespace adx