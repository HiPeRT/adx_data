#include "adx_data/twist.hpp"

namespace adx {
namespace data {

Twist::Twist(const roscomp::geometry_msgs::Twist& aRosTwist)
{
    *this = aRosTwist;
}

Twist::Twist(roscomp::CallbackPtr<roscomp::geometry_msgs::Twist> aRosTwist)
  : Twist(*aRosTwist)
{}

Twist::Twist(const roscomp::geometry_msgs::TwistWithCovariance& aRosTwist)
{
    *this = aRosTwist;
}

Twist::Twist(roscomp::CallbackPtr<roscomp::geometry_msgs::TwistWithCovariance> aRosTwist)
  : Twist(*aRosTwist)
{}

Twist& Twist::operator=(const roscomp::geometry_msgs::Twist& aRosTwist)
{
    linear.x() = aRosTwist.linear.x;
    linear.y() = aRosTwist.linear.y;
    linear.z() = aRosTwist.linear.z;

    angular.x() = aRosTwist.angular.x;
    angular.y() = aRosTwist.angular.y;
    angular.z() = aRosTwist.angular.z;

    return *this;
}

Twist& Twist::operator=(roscomp::CallbackPtr<roscomp::geometry_msgs::Twist> aRosTwist)
{
    return operator=(*aRosTwist);
}

Twist& Twist::operator=(const roscomp::geometry_msgs::TwistWithCovariance& aRosTwist)
{
    operator=(aRosTwist.twist);
    twist_covariance = aRosTwist.covariance;

    return *this;
}

Twist& Twist::operator=(roscomp::CallbackPtr<roscomp::geometry_msgs::TwistWithCovariance> aRosTwist)
{
    return operator=(*aRosTwist);
}

} // namespace data
} // namespace adx