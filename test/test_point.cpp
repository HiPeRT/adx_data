#include <gtest/gtest.h>

#include "adx_data/point.hpp"

namespace adx {
namespace data {

TEST(PointTest, FromRosPoint)
{
    geometry_msgs::msg::Point ros_point;

    ros_point.x = 1.0;
    ros_point.y = 2.0;
    ros_point.z = 3.0;

    Point3d adx_point3d_constructor(ros_point);
    EXPECT_EQ(ros_point.x, adx_point3d_constructor.position.x());
    EXPECT_EQ(ros_point.y, adx_point3d_constructor.position.y());
    EXPECT_EQ(ros_point.z, adx_point3d_constructor.position.z());

    Point3d adx_point3d_assignment;
    adx_point3d_assignment = ros_point;
    EXPECT_EQ(ros_point.x, adx_point3d_assignment.position.x());
    EXPECT_EQ(ros_point.y, adx_point3d_assignment.position.y());
    EXPECT_EQ(ros_point.z, adx_point3d_assignment.position.z());

    Point2d adx_point2d_constructor(ros_point);
    EXPECT_EQ(ros_point.x, adx_point2d_constructor.position.x());
    EXPECT_EQ(ros_point.y, adx_point2d_constructor.position.y());

    Point2d adx_point2d_assignment;
    adx_point2d_assignment = ros_point;
    EXPECT_EQ(ros_point.x, adx_point2d_assignment.position.x());
    EXPECT_EQ(ros_point.y, adx_point2d_assignment.position.y());

    Point3f adx_point3f_constructor(ros_point);
    EXPECT_EQ(static_cast<float>(ros_point.x), adx_point3f_constructor.position.x());
    EXPECT_EQ(static_cast<float>(ros_point.y), adx_point3f_constructor.position.y());
    EXPECT_EQ(static_cast<float>(ros_point.z), adx_point3f_constructor.position.z());

    Point3f adx_point3f_assignment;
    adx_point3f_assignment = ros_point;
    EXPECT_EQ(static_cast<float>(ros_point.x), adx_point3f_assignment.position.x());
    EXPECT_EQ(static_cast<float>(ros_point.y), adx_point3f_assignment.position.y());
    EXPECT_EQ(static_cast<float>(ros_point.z), adx_point3f_assignment.position.z());

    Point2f adx_point2f_constructor(ros_point);
    EXPECT_EQ(static_cast<float>(ros_point.x), adx_point2f_constructor.position.x());
    EXPECT_EQ(static_cast<float>(ros_point.y), adx_point2f_constructor.position.y());

    Point2f adx_point2f_assignment;
    adx_point2f_assignment = ros_point;
    EXPECT_EQ(static_cast<float>(ros_point.x), adx_point2f_assignment.position.x());
    EXPECT_EQ(static_cast<float>(ros_point.y), adx_point2f_assignment.position.y());
}

TEST(PointTest, ToRosPoint)
{
    Point3d adx_point3d;
    adx_point3d.position.x() = 1.0;
    adx_point3d.position.y() = 2.0;
    adx_point3d.position.z() = 3.0;

    geometry_msgs::msg::Point ros_point = adx_point3d.toRos();
    EXPECT_EQ(ros_point.x, adx_point3d.position.x());
    EXPECT_EQ(ros_point.y, adx_point3d.position.y());
    EXPECT_EQ(ros_point.z, adx_point3d.position.z());

    Point2d adx_point2d;
    adx_point2d.position.x() = 1.0;
    adx_point2d.position.y() = 2.0;

    ros_point = adx_point2d.toRos();
    EXPECT_EQ(ros_point.x, adx_point2d.position.x());
    EXPECT_EQ(ros_point.y, adx_point2d.position.y());
    EXPECT_EQ(ros_point.z, 0.0);

    Point3f adx_point3f;
    adx_point3f.position.x() = 1.0;
    adx_point3f.position.y() = 2.0;
    adx_point3f.position.z() = 3.0;

    ros_point = adx_point3f.toRos();
    EXPECT_EQ(static_cast<float>(ros_point.x), adx_point3f.position.x());
    EXPECT_EQ(static_cast<float>(ros_point.y), adx_point3f.position.y());
    EXPECT_EQ(static_cast<float>(ros_point.z), adx_point3f.position.z());

    Point2f adx_point2f;
    adx_point2f.position.x() = 1.0;
    adx_point2f.position.y() = 2.0;

    ros_point = adx_point2f.toRos();
    EXPECT_EQ(static_cast<float>(ros_point.x), adx_point2f.position.x());
    EXPECT_EQ(static_cast<float>(ros_point.y), adx_point2f.position.y());
    EXPECT_EQ(static_cast<float>(ros_point.z), 0.0);
}

} // namespace data
} // namespace adx
