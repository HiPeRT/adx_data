#include <gtest/gtest.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "adx_data/quaternion.hpp"

namespace adx {
namespace data {

TEST(QuaternionTest, FromRosQuaternion)
{
    geometry_msgs::msg::Quaternion ros_quaternion;

    tf2::Quaternion tf2_quaternion;

    const double roll = 0.1, pitch = 0.2, yaw = 0.3;
    tf2_quaternion.setRPY(roll, pitch, yaw);

    ros_quaternion.w = tf2_quaternion.w();
    ros_quaternion.x = tf2_quaternion.x();
    ros_quaternion.y = tf2_quaternion.y();
    ros_quaternion.z = tf2_quaternion.z();

    Quaterniond adx_quaterniond_constructor(ros_quaternion);
    EXPECT_EQ(ros_quaternion.w, adx_quaterniond_constructor.w());
    EXPECT_EQ(ros_quaternion.x, adx_quaterniond_constructor.x());
    EXPECT_EQ(ros_quaternion.y, adx_quaterniond_constructor.y());
    EXPECT_EQ(ros_quaternion.z, adx_quaterniond_constructor.z());

    Quaterniond adx_quaterniond_assignment = ros_quaternion;
    EXPECT_EQ(ros_quaternion.w, adx_quaterniond_assignment.w());
    EXPECT_EQ(ros_quaternion.x, adx_quaterniond_assignment.x());
    EXPECT_EQ(ros_quaternion.y, adx_quaterniond_assignment.y());
    EXPECT_EQ(ros_quaternion.z, adx_quaterniond_assignment.z());

    Quaternionf adx_quaternionf_constructor(ros_quaternion);
    EXPECT_EQ(static_cast<float>(ros_quaternion.w), adx_quaternionf_constructor.w());
    EXPECT_EQ(static_cast<float>(ros_quaternion.x), adx_quaternionf_constructor.x());
    EXPECT_EQ(static_cast<float>(ros_quaternion.y), adx_quaternionf_constructor.y());
    EXPECT_EQ(static_cast<float>(ros_quaternion.z), adx_quaternionf_constructor.z());

    Quaternionf adx_quaternionf_assignment = ros_quaternion;
    EXPECT_EQ(static_cast<float>(ros_quaternion.w), adx_quaternionf_assignment.w());
    EXPECT_EQ(static_cast<float>(ros_quaternion.x), adx_quaternionf_assignment.x());
    EXPECT_EQ(static_cast<float>(ros_quaternion.y), adx_quaternionf_assignment.y());
    EXPECT_EQ(static_cast<float>(ros_quaternion.z), adx_quaternionf_assignment.z());
}

TEST(QuaternionTest, ToRosQuaternion)
{
    const double roll = 0.1, pitch = 0.2, yaw = 0.3;

    Quaterniond adx_quaterniond;
    adx_quaterniond.setRPY(roll, pitch, yaw);

    geometry_msgs::msg::Quaternion ros_quaternion = adx_quaterniond.toRos();
    EXPECT_EQ(ros_quaternion.w, adx_quaterniond.w());
    EXPECT_EQ(ros_quaternion.x, adx_quaterniond.x());
    EXPECT_EQ(ros_quaternion.y, adx_quaterniond.y());
    EXPECT_EQ(ros_quaternion.z, adx_quaterniond.z());

    Quaternionf adx_quaternionf;
    adx_quaternionf.setRPY(roll, pitch, yaw);

    ros_quaternion = adx_quaternionf.toRos();
    EXPECT_EQ(static_cast<float>(ros_quaternion.w), adx_quaternionf.w());
    EXPECT_EQ(static_cast<float>(ros_quaternion.x), adx_quaternionf.x());
    EXPECT_EQ(static_cast<float>(ros_quaternion.y), adx_quaternionf.y());
    EXPECT_EQ(static_cast<float>(ros_quaternion.z), adx_quaternionf.z());
}

} // namespace data
} // namespace adx
