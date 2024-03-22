#include "adx_data/path.hpp"

namespace adx {
namespace data {

Path::Path(const roscomp::nav_msgs::Path& aRosPath)
  : Header(aRosPath.header)
{
    for (auto it : aRosPath.poses) {
        const adx::data::Pose<double, 3, false, false> pose(it.pose);
        poses.push_back(pose);
    }
}

Path::Path(roscomp::CallbackPtr<roscomp::nav_msgs::Path> aRosPath)
  : Path(*aRosPath)
{}

Path::Path(const roscomp::adx_msgs::Plan& aRosPlan)
  : Header(aRosPlan.header)
{
    for (auto point : aRosPlan.points) {
        const adx::data::Pose<double, 3, false, false> pose(point.position);
        poses.push_back(pose);
    }
}

Path::Path(roscomp::CallbackPtr<roscomp::adx_msgs::Plan> aRosPlan)
  : Path(*aRosPlan)
{}

roscomp::nav_msgs::Path Path::toRos() const
{
    roscomp::nav_msgs::Path ros_path;

    ros_path.header = Header::toRos();

    roscomp::geometry_msgs::PoseStamped ros_pose;
    for (auto pose : poses) {
        ros_pose.pose.position.x = pose.position.x();
        ros_pose.pose.position.y = pose.position.y();
        ros_pose.pose.position.z = pose.position.z();

        ros_pose.pose.orientation.w = pose.orientation.w();
        ros_pose.pose.orientation.x = pose.orientation.x();
        ros_pose.pose.orientation.y = pose.orientation.y();
        ros_pose.pose.orientation.z = pose.orientation.z();

        ros_path.poses.push_back(ros_pose);
    }

    return ros_path;
}

Plan::Plan(const roscomp::adx_msgs::Plan& aRosPlan)
  : Header(aRosPlan.header)
{
    for (auto point : aRosPlan.points) {
        const adx::data::Point3d pose(point.position);
        const adx::data::Vector3d speed(point.speed);
        positions.push_back(pose);
        speeds.push_back(speed);
    }
}

Plan::Plan(roscomp::CallbackPtr<roscomp::adx_msgs::Plan> aRosPlan)
  : Plan(*aRosPlan)
{}

roscomp::adx_msgs::Plan Plan::toRos() const
{
    roscomp::adx_msgs::Plan ros_plan;

    ros_plan.header = Header::toRos();

    roscomp::adx_msgs::PlanPoint ros_point;

    for (unsigned int i = 0; i < positions.size(); ++i) {
        ros_point.position.x = positions[i].position.x();
        ros_point.position.y = positions[i].position.y();
        ros_point.position.z = positions[i].position.z();

        ros_point.speed.x = speeds[i].x();
        ros_point.speed.y = speeds[i].y();
        ros_point.speed.z = speeds[i].z();

        ros_plan.points.push_back(ros_point);
    }

    return ros_plan;
}

roscomp::nav_msgs::Path Plan::toRosPath() const
{
    return adx::data::Path(toRos()).toRos();
}

} // namespace data
} // namespace adx
