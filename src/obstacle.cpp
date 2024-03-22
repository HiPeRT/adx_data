#include "adx_data/obstacle.hpp"

namespace adx {
namespace data {

Obstacle::Obstacle(const roscomp::adx_msgs::Obstacle& aRosObstacle)
  : Pose3d(aRosObstacle.pose)
{
    *this = aRosObstacle.size;
}

Obstacle::Obstacle(roscomp::CallbackPtr<roscomp::adx_msgs::Obstacle> aRosObstacle)
  : Obstacle(*aRosObstacle)
{}

Obstacle& Obstacle::operator=(const roscomp::geometry_msgs::Vector3& aRosSize)
{
    size.x() = aRosSize.x;
    size.y() = aRosSize.y;
    size.z() = aRosSize.z;

    return *this;
}

Obstacle& Obstacle::operator=(roscomp::CallbackPtr<roscomp::geometry_msgs::Vector3> aRosSize)
{
    return operator=(*aRosSize);
}

Obstacle& Obstacle::operator=(const roscomp::adx_msgs::Obstacle& aRosObstacle)
{
    Pose::operator=(aRosObstacle.pose);

    return operator=(aRosObstacle.size);
}

Obstacle& Obstacle::operator=(roscomp::CallbackPtr<roscomp::adx_msgs::Obstacle> aRosObstacle)
{
    return operator=(*aRosObstacle);
}

} // namespace data
} // namespace adx