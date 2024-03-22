#include <iostream>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "adx_data/pose.hpp"

#include "pose_helpers.hpp"

namespace adx {
namespace data {

class PoseTest : public ::testing::Test
{
  protected:
    rclcpp::Node::SharedPtr mNode;

    void SetUp() override
    {
        int argc = 0;
        char** argv = 0;
        rclcpp::init(argc, argv);
        mNode = std::make_shared<rclcpp::Node>("pose_test");
    }

    void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(PoseTest, Pose3dStampCovCopy)
{
    test_pose<double, 3, true, true>();
}
TEST_F(PoseTest, Pose3dStampCopy)
{
    test_pose<double, 3, true, false>();
}
TEST_F(PoseTest, Pose3dCovCopy)
{
    test_pose<double, 3, false, true>();
}
TEST_F(PoseTest, Pose3dCopy)
{
    test_pose<double, 3, false, false>();
}

TEST_F(PoseTest, Pose3fStampCovCopy)
{
    test_pose<float, 3, true, true>();
}
TEST_F(PoseTest, Pose3fStampCopy)
{
    test_pose<float, 3, true, false>();
}
TEST_F(PoseTest, Pose3fCovCopy)
{
    test_pose<float, 3, false, true>();
}
TEST_F(PoseTest, Pose3fCopy)
{
    test_pose<float, 3, false, false>();
}

TEST_F(PoseTest, Pose2dStampCovCopy)
{
    test_pose<double, 2, true, true>();
}
TEST_F(PoseTest, Pose2dStampCopy)
{
    test_pose<double, 2, true, false>();
}
TEST_F(PoseTest, Pose2dCovCopy)
{
    test_pose<double, 2, false, true>();
}
TEST_F(PoseTest, Pose2dCopy)
{
    test_pose<double, 2, false, false>();
}

TEST_F(PoseTest, Pose2fStampCovCopy)
{
    test_pose<float, 2, true, true>();
}
TEST_F(PoseTest, Pose2fStampCopy)
{
    test_pose<float, 2, true, false>();
}
TEST_F(PoseTest, Pose2fCovCopy)
{
    test_pose<float, 2, false, true>();
}
TEST_F(PoseTest, Pose2fCopy)
{
    test_pose<float, 2, false, false>();
}

TEST_F(PoseTest, FromRosPoint)
{
    const double x = 1.0, y = 2.0, z = 3.0;
    geometry_msgs::msg::Point ros_point;
    ros_point.x = x;
    ros_point.y = y;
    ros_point.z = z;

    std::cout << "Testing Pose<double, 3, true, true> Point constructor" << std::endl;
    Pose<double, 3, true, true> adx_posed3tt_constructor(ros_point);
    compare_position<double, 3>(ros_point, adx_posed3tt_constructor.position);
    std::cout << "Testing Pose<double, 3, true, true> Point assignment" << std::endl;
    Pose<double, 3, true, true> adx_posed3tt_assignment;
    adx_posed3tt_assignment = ros_point;
    compare_position<double, 3>(ros_point, adx_posed3tt_assignment.position);
    std::cout << "Testing Pose<float, 3, true, true> Point constructor" << std::endl;
    Pose<float, 3, true, true> adx_posef3tt_constructor(ros_point);
    compare_position<float, 3>(ros_point, adx_posef3tt_constructor.position);
    std::cout << "Testing Pose<float, 3, true, true> Point assignment" << std::endl;
    Pose<float, 3, true, true> adx_posef3tt_assignment;
    adx_posef3tt_assignment = ros_point;
    compare_position<float, 3>(ros_point, adx_posef3tt_assignment.position);

    std::cout << "Testing Pose<double, 3, false, true> Point constructor" << std::endl;
    Pose<double, 3, false, true> adx_posed3ft_constructor(ros_point);
    compare_position<double, 3>(ros_point, adx_posed3ft_constructor.position);
    std::cout << "Testing Pose<double, 3, false, true> Point assignment" << std::endl;
    Pose<double, 3, false, true> adx_posed3ft_assignment;
    adx_posed3ft_assignment = ros_point;
    compare_position<double, 3>(ros_point, adx_posed3ft_assignment.position);
    std::cout << "Testing Pose<float, 3, false, true> Point constructor" << std::endl;
    Pose<float, 3, false, true> adx_posef3ft_constructor(ros_point);
    compare_position<float, 3>(ros_point, adx_posef3ft_constructor.position);
    std::cout << "Testing Pose<float, 3, false, true> Point assignment" << std::endl;
    Pose<float, 3, false, true> adx_posef3ft_assignment;
    adx_posef3ft_assignment = ros_point;
    compare_position<float, 3>(ros_point, adx_posef3ft_assignment.position);

    std::cout << "Testing Pose<double, 3, true, false> Point constructor" << std::endl;
    Pose<double, 3, true, false> adx_posed3tf_constructor(ros_point);
    compare_position<double, 3>(ros_point, adx_posed3tf_constructor.position);
    std::cout << "Testing Pose<double, 3, true, false> Point assignment" << std::endl;
    Pose<double, 3, true, false> adx_posed3tf_assignment;
    adx_posed3tf_assignment = ros_point;
    compare_position<double, 3>(ros_point, adx_posed3tf_assignment.position);
    std::cout << "Testing Pose<float, 3, true, false> Point constructor" << std::endl;
    Pose<float, 3, true, false> adx_posef3tf_constructor(ros_point);
    compare_position<float, 3>(ros_point, adx_posef3tf_constructor.position);
    std::cout << "Testing Pose<float, 3, true, false> Point assignment" << std::endl;
    Pose<float, 3, true, false> adx_posef3tf_assignment;
    adx_posef3tf_assignment = ros_point;
    compare_position<float, 3>(ros_point, adx_posef3tf_assignment.position);

    std::cout << "Testing Pose<double, 3, false, false> Point constructor" << std::endl;
    Pose<double, 3, false, false> adx_posed3ff_constructor(ros_point);
    compare_position<double, 3>(ros_point, adx_posed3ff_constructor.position);
    std::cout << "Testing Pose<double, 3, false, false> Point assignment" << std::endl;
    Pose<double, 3, false, false> adx_posed3ff_assignment;
    adx_posed3ff_assignment = ros_point;
    compare_position<double, 3>(ros_point, adx_posed3ff_assignment.position);
    std::cout << "Testing Pose<float, 3, false, false> Point constructor" << std::endl;
    Pose<float, 3, false, false> adx_posef3ff_constructor(ros_point);
    compare_position<float, 3>(ros_point, adx_posef3ff_constructor.position);
    std::cout << "Testing Pose<float, 3, false, false> Point assignment" << std::endl;
    Pose<float, 3, false, false> adx_posef3ff_assignment;
    adx_posef3ff_assignment = ros_point;
    compare_position<float, 3>(ros_point, adx_posef3ff_assignment.position);

    std::cout << "Testing Pose<double, 2, true, true> Point constructor" << std::endl;
    Pose<double, 2, true, true> adx_posed2tt_constructor(ros_point);
    compare_position<double, 2>(ros_point, adx_posed2tt_constructor.position);
    std::cout << "Testing Pose<double, 2, true, true> Point assignment" << std::endl;
    Pose<double, 2, true, true> adx_posed2tt_assignment;
    adx_posed2tt_assignment = ros_point;
    compare_position<double, 2>(ros_point, adx_posed2tt_assignment.position);
    std::cout << "Testing Pose<float, 2, true, true> Point constructor" << std::endl;
    Pose<float, 2, true, true> adx_posef2tt_constructor(ros_point);
    compare_position<float, 2>(ros_point, adx_posef2tt_constructor.position);
    std::cout << "Testing Pose<float, 2, true, true> Point assignment" << std::endl;
    Pose<float, 2, true, true> adx_posef2tt_assignment;
    adx_posef2tt_assignment = ros_point;
    compare_position<float, 2>(ros_point, adx_posef2tt_assignment.position);

    std::cout << "Testing Pose<double, 2, false, true> Point constructor" << std::endl;
    Pose<double, 2, false, true> adx_posed2ft_constructor(ros_point);
    compare_position<double, 2>(ros_point, adx_posed2ft_constructor.position);
    std::cout << "Testing Pose<double, 2, false, true> Point assignment" << std::endl;
    Pose<double, 2, false, true> adx_posed2ft_assignment;
    adx_posed2ft_assignment = ros_point;
    compare_position<double, 2>(ros_point, adx_posed2ft_assignment.position);
    std::cout << "Testing Pose<float, 2, false, true> Point constructor" << std::endl;
    Pose<float, 2, false, true> adx_posef2ft_constructor(ros_point);
    compare_position<float, 2>(ros_point, adx_posef2ft_constructor.position);
    std::cout << "Testing Pose<float, 2, false, true> Point assignment" << std::endl;
    Pose<float, 2, false, true> adx_posef2ft_assignment;
    adx_posef2ft_assignment = ros_point;
    compare_position<float, 2>(ros_point, adx_posef2ft_assignment.position);

    std::cout << "Testing Pose<double, 2, true, false> Point constructor" << std::endl;
    Pose<double, 2, true, false> adx_posed2tf_constructor(ros_point);
    compare_position<double, 2>(ros_point, adx_posed2tf_constructor.position);
    std::cout << "Testing Pose<double, 2, true, false> Point assignment" << std::endl;
    Pose<double, 2, true, false> adx_posed2tf_assignment;
    adx_posed2tf_assignment = ros_point;
    compare_position<double, 2>(ros_point, adx_posed2tf_assignment.position);
    std::cout << "Testing Pose<float, 2, true, false> Point constructor" << std::endl;
    Pose<float, 2, true, false> adx_posef2tf_constructor(ros_point);
    compare_position<float, 2>(ros_point, adx_posef2tf_constructor.position);
    std::cout << "Testing Pose<float, 2, true, false> Point assignment" << std::endl;
    Pose<float, 2, true, false> adx_posef2tf_assignment;
    adx_posef2tf_assignment = ros_point;
    compare_position<float, 2>(ros_point, adx_posef2tf_assignment.position);

    std::cout << "Testing Pose<double, 2, false, false> Point constructor" << std::endl;
    Pose<double, 2, false, false> adx_posed2ff_constructor(ros_point);
    compare_position<double, 2>(ros_point, adx_posed2ff_constructor.position);
    std::cout << "Testing Pose<double, 2, false, false> Point assignment" << std::endl;
    Pose<double, 2, false, false> adx_posed2ff_assignment;
    adx_posed2ff_assignment = ros_point;
    compare_position<double, 2>(ros_point, adx_posed2ff_assignment.position);
    std::cout << "Testing Pose<float, 2, false, false> Point constructor" << std::endl;
    Pose<float, 2, false, false> adx_posef2ff_constructor(ros_point);
    compare_position<float, 2>(ros_point, adx_posef2ff_constructor.position);
    std::cout << "Testing Pose<float, 2, false, false> Point assignment" << std::endl;
    Pose<float, 2, false, false> adx_posef2ff_assignment;
    adx_posef2ff_assignment = ros_point;
    compare_position<float, 2>(ros_point, adx_posef2ff_assignment.position);
}

TEST_F(PoseTest, ToRosPoint)
{
    const double x = 1.0, y = 2.0, z = 3.0;

    std::cout << "Testing Pose<double, 3, true, true> toRos Point" << std::endl;
    Pose<double, 3, true, true> adx_posed3tt;
    adx_posed3tt.position = { x, y, z };
    geometry_msgs::msg::Point ros_point_d3tt;
    ros_point_d3tt = adx_posed3tt.Point::toRos();
    compare_position<double, 3>(ros_point_d3tt, adx_posed3tt.position);
    std::cout << "Testing Pose<float, 3, true, true> toRos Point" << std::endl;
    Pose<float, 3, true, true> adx_posef3tt;
    adx_posef3tt.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    geometry_msgs::msg::Point ros_point_f3tt;
    ros_point_f3tt = adx_posef3tt.Point::toRos();
    compare_position<float, 3>(ros_point_f3tt, adx_posef3tt.position);

    std::cout << "Testing Pose<double, 3, false, true> toRos Point" << std::endl;
    Pose<double, 3, false, true> adx_posed3ft;
    adx_posed3ft.position = { x, y, z };
    geometry_msgs::msg::Point ros_point_d3ft;
    ros_point_d3ft = adx_posed3ft.Point::toRos();
    compare_position<double, 3>(ros_point_d3ft, adx_posed3ft.position);
    std::cout << "Testing Pose<float, 3, false, true> toRos Point" << std::endl;
    Pose<float, 3, false, true> adx_posef3ft;
    adx_posef3ft.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    geometry_msgs::msg::Point ros_point_f3ft;
    ros_point_f3ft = adx_posef3ft.Point::toRos();
    compare_position<float, 3>(ros_point_f3ft, adx_posef3ft.position);

    std::cout << "Testing Pose<double, 3, true, false> toRos Point" << std::endl;
    Pose<double, 3, true, false> adx_posed3tf;
    adx_posed3tf.position = { x, y, z };
    geometry_msgs::msg::Point ros_point_d3tf;
    ros_point_d3tf = adx_posed3tf.Point::toRos();
    compare_position<double, 3>(ros_point_d3tf, adx_posed3tf.position);
    std::cout << "Testing Pose<float, 3, true, false> toRos Point" << std::endl;
    Pose<float, 3, true, false> adx_posef3tf;
    adx_posef3tf.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    geometry_msgs::msg::Point ros_point_f3tf;
    ros_point_f3tf = adx_posef3tf.Point::toRos();
    compare_position<float, 3>(ros_point_f3tf, adx_posef3tf.position);

    std::cout << "Testing Pose<double, 3, false, false> toRos Point" << std::endl;
    Pose<double, 3, false, false> adx_posed3ff;
    adx_posed3ff.position = { x, y, z };
    geometry_msgs::msg::Point ros_point_d3ff;
    ros_point_d3ff = adx_posed3ff.Point::toRos();
    compare_position<double, 3>(ros_point_d3ff, adx_posed3ff.position);
    std::cout << "Testing Pose<float, 3, false, false> toRos Point" << std::endl;
    Pose<float, 3, false, false> adx_posef3ff;
    adx_posef3ff.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    geometry_msgs::msg::Point ros_point_f3ff;
    ros_point_f3ff = adx_posef3ff.Point::toRos();
    compare_position<float, 3>(ros_point_f3ff, adx_posef3ff.position);

    std::cout << "Testing Pose<double, 2, true, true> toRos Point" << std::endl;
    Pose<double, 2, true, true> adx_posed2tt;
    adx_posed2tt.position = { x, y };
    geometry_msgs::msg::Point ros_point_d2tt;
    ros_point_d2tt = adx_posed2tt.Point::toRos();
    compare_position<double, 2>(ros_point_d2tt, adx_posed2tt.position);
    EXPECT_EQ(ros_point_d2tt.z, 0.0);
    std::cout << "Testing Pose<float, 2, true, true> toRos Point" << std::endl;
    Pose<float, 2, true, true> adx_posef2tt;
    adx_posef2tt.position = { static_cast<float>(x), static_cast<float>(y) };
    geometry_msgs::msg::Point ros_point_f2tt;
    ros_point_f2tt = adx_posef2tt.Point::toRos();
    compare_position<float, 2>(ros_point_f2tt, adx_posef2tt.position);
    EXPECT_EQ(static_cast<float>(ros_point_f2tt.z), 0.0);

    std::cout << "Testing Pose<double, 2, false, true> toRos Point" << std::endl;
    Pose<double, 2, false, true> adx_posed2ft;
    adx_posed2ft.position = { x, y };
    geometry_msgs::msg::Point ros_point_d2ft;
    ros_point_d2ft.z = z;
    ros_point_d2ft = adx_posed2ft.Point::toRos();
    compare_position<double, 2>(ros_point_d2ft, adx_posed2ft.position);
    EXPECT_EQ(ros_point_d2ft.z, 0.0);
    std::cout << "Testing Pose<float, 2, false, true> toRos Point" << std::endl;
    Pose<float, 2, false, true> adx_posef2ft;
    adx_posef2ft.position = { static_cast<float>(x), static_cast<float>(y) };
    geometry_msgs::msg::Point ros_point_f2ft;
    ros_point_f2ft.z = z;
    ros_point_f2ft = adx_posef2ft.Point::toRos();
    compare_position<float, 2>(ros_point_f2ft, adx_posef2ft.position);
    EXPECT_EQ(static_cast<float>(ros_point_f2ft.z), 0.0);

    std::cout << "Testing Pose<double, 2, true, false> toRos Point" << std::endl;
    Pose<double, 2, true, false> adx_posed2tf;
    adx_posed2tf.position = { x, y };
    geometry_msgs::msg::Point ros_point_d2tf;
    ros_point_d2tf.z = z;
    ros_point_d2tf = adx_posed2tf.Point::toRos();
    compare_position<double, 2>(ros_point_d2tf, adx_posed2tf.position);
    EXPECT_EQ(ros_point_d2tf.z, 0.0);
    std::cout << "Testing Pose<float, 2, true, false> toRos Point" << std::endl;
    Pose<float, 2, true, false> adx_posef2tf;
    adx_posef2tf.position = { static_cast<float>(x), static_cast<float>(y) };
    geometry_msgs::msg::Point ros_point_f2tf;
    ros_point_f2tf.z = z;
    ros_point_f2tf = adx_posef2tf.Point::toRos();
    compare_position<float, 2>(ros_point_f2tf, adx_posef2tf.position);
    EXPECT_EQ(static_cast<float>(ros_point_f2tf.z), 0.0);

    std::cout << "Testing Pose<double, 2, false, false> toRos Point" << std::endl;
    Pose<double, 2, false, false> adx_posed2ff;
    adx_posed2ff.position = { x, y };
    geometry_msgs::msg::Point ros_point_d2ff;
    ros_point_d2ff.z = z;
    ros_point_d2ff = adx_posed2ff.Point::toRos();
    compare_position<double, 2>(ros_point_d2ff, adx_posed2ff.position);
    EXPECT_EQ(ros_point_d2ff.z, 0.0);
    std::cout << "Testing Pose<float, 2, false, false> toRos Point" << std::endl;
    Pose<float, 2, false, false> adx_posef2ff;
    adx_posef2ff.position = { static_cast<float>(x), static_cast<float>(y) };
    geometry_msgs::msg::Point ros_point_f2ff;
    ros_point_f2ff.z = z;
    ros_point_f2ff = adx_posef2ff.Point::toRos();
    compare_position<float, 2>(ros_point_f2ff, adx_posef2ff.position);
    EXPECT_EQ(static_cast<float>(ros_point_f2ff.z), 0.0);
}

TEST_F(PoseTest, FromRosPose)
{
    const double x = 1.0, y = 2.0, z = 3.0, roll = 0.1, pitch = 0.2, yaw = 0.3;
    geometry_msgs::msg::Pose ros_pose;

    ros_pose.position.x = x;
    ros_pose.position.y = y;
    ros_pose.position.z = z;

    tf2::Quaternion tf2_quaternion;
    tf2_quaternion.setRPY(roll, pitch, yaw);

    ros_pose.orientation.w = tf2_quaternion.w();
    ros_pose.orientation.x = tf2_quaternion.x();
    ros_pose.orientation.y = tf2_quaternion.y();
    ros_pose.orientation.z = tf2_quaternion.z();

    std::cout << "Testing Pose<double, 3, true, true> Pose constructor" << std::endl;
    Pose<double, 3, true, true> adx_posed3tt_constructor(ros_pose);
    compare_position<double, 3>(ros_pose.position, adx_posed3tt_constructor.position);
    compare_orientation<double>(ros_pose.orientation, adx_posed3tt_constructor.orientation);
    std::cout << "Testing Pose<double, 3, true, true> Pose assignment" << std::endl;
    Pose<double, 3, true, true> adx_posed3tt_assignment;
    adx_posed3tt_assignment = ros_pose;
    compare_position<double, 3>(ros_pose.position, adx_posed3tt_assignment.position);
    compare_orientation<double>(ros_pose.orientation, adx_posed3tt_assignment.orientation);
    std::cout << "Testing Pose<float, 3, true, true> Pose constructor" << std::endl;
    Pose<float, 3, true, true> adx_posef3tt_constructor(ros_pose);
    compare_position<float, 3>(ros_pose.position, adx_posef3tt_constructor.position);
    compare_orientation<float>(ros_pose.orientation, adx_posef3tt_constructor.orientation);
    std::cout << "Testing Pose<float, 3, true, true> Pose assignment" << std::endl;
    Pose<float, 3, true, true> adx_posef3tt_assignment;
    adx_posef3tt_assignment = ros_pose;
    compare_position<float, 3>(ros_pose.position, adx_posef3tt_assignment.position);
    compare_orientation<float>(ros_pose.orientation, adx_posef3tt_assignment.orientation);

    std::cout << "Testing Pose<double, 3, false, true> Pose constructor" << std::endl;
    Pose<double, 3, false, true> adx_posed3ft_constructor(ros_pose);
    compare_position<double, 3>(ros_pose.position, adx_posed3ft_constructor.position);
    compare_orientation<double>(ros_pose.orientation, adx_posed3ft_constructor.orientation);
    std::cout << "Testing Pose<double, 3, false, true> Pose assignment" << std::endl;
    Pose<double, 3, false, true> adx_posed3ft_assignment;
    adx_posed3ft_assignment = ros_pose;
    compare_position<double, 3>(ros_pose.position, adx_posed3ft_assignment.position);
    compare_orientation<double>(ros_pose.orientation, adx_posed3ft_assignment.orientation);
    std::cout << "Testing Pose<float, 3, false, true> Pose constructor" << std::endl;
    Pose<float, 3, false, true> adx_posef3ft_constructor(ros_pose);
    compare_position<float, 3>(ros_pose.position, adx_posef3ft_constructor.position);
    compare_orientation<float>(ros_pose.orientation, adx_posef3ft_constructor.orientation);
    std::cout << "Testing Pose<float, 3, false, true> Pose assignment" << std::endl;
    Pose<float, 3, false, true> adx_posef3ft_assignment;
    adx_posef3ft_assignment = ros_pose;
    compare_position<float, 3>(ros_pose.position, adx_posef3ft_assignment.position);
    compare_orientation<float>(ros_pose.orientation, adx_posef3ft_assignment.orientation);

    std::cout << "Testing Pose<double, 3, true, false> Pose constructor" << std::endl;
    Pose<double, 3, true, false> adx_posed3tf_constructor(ros_pose);
    compare_position<double, 3>(ros_pose.position, adx_posed3tf_constructor.position);
    compare_orientation<double>(ros_pose.orientation, adx_posed3tf_constructor.orientation);
    std::cout << "Testing Pose<double, 3, true, false> Pose assignment" << std::endl;
    Pose<double, 3, true, false> adx_posed3tf_assignment;
    adx_posed3tf_assignment = ros_pose;
    compare_position<double, 3>(ros_pose.position, adx_posed3tf_assignment.position);
    compare_orientation<double>(ros_pose.orientation, adx_posed3tf_assignment.orientation);
    std::cout << "Testing Pose<float, 3, true, false> Pose constructor" << std::endl;
    Pose<float, 3, true, false> adx_posef3tf_constructor(ros_pose);
    compare_position<float, 3>(ros_pose.position, adx_posef3tf_constructor.position);
    compare_orientation<float>(ros_pose.orientation, adx_posef3tf_constructor.orientation);
    std::cout << "Testing Pose<float, 3, true, false> Pose assignment" << std::endl;
    Pose<float, 3, true, false> adx_posef3tf_assignment;
    adx_posef3tf_assignment = ros_pose;
    compare_position<float, 3>(ros_pose.position, adx_posef3tf_assignment.position);
    compare_orientation<float>(ros_pose.orientation, adx_posef3tf_assignment.orientation);

    std::cout << "Testing Pose<double, 3, false, false> Pose constructor" << std::endl;
    Pose<double, 3, false, false> adx_posed3ff_constructor(ros_pose);
    compare_position<double, 3>(ros_pose.position, adx_posed3ff_constructor.position);
    compare_orientation<double>(ros_pose.orientation, adx_posed3ff_constructor.orientation);
    std::cout << "Testing Pose<double, 3, false, false> Pose assignment" << std::endl;
    Pose<double, 3, false, false> adx_posed3ff_assignment;
    adx_posed3ff_assignment = ros_pose;
    compare_position<double, 3>(ros_pose.position, adx_posed3ff_assignment.position);
    compare_orientation<double>(ros_pose.orientation, adx_posed3ff_assignment.orientation);
    std::cout << "Testing Pose<float, 3, false, false> Pose constructor" << std::endl;
    Pose<float, 3, false, false> adx_posef3ff_constructor(ros_pose);
    compare_position<float, 3>(ros_pose.position, adx_posef3ff_constructor.position);
    compare_orientation<float>(ros_pose.orientation, adx_posef3ff_constructor.orientation);
    std::cout << "Testing Pose<float, 3, false, false> Pose assignment" << std::endl;
    Pose<float, 3, false, false> adx_posef3ff_assignment;
    adx_posef3ff_assignment = ros_pose;
    compare_position<float, 3>(ros_pose.position, adx_posef3ff_assignment.position);
    compare_orientation<float>(ros_pose.orientation, adx_posef3ff_assignment.orientation);

    tf2_quaternion.setRPY(0.0, 0.0, yaw);

    ros_pose.orientation.w = tf2_quaternion.w();
    ros_pose.orientation.x = tf2_quaternion.x();
    ros_pose.orientation.y = tf2_quaternion.y();
    ros_pose.orientation.z = tf2_quaternion.z();

    std::cout << "Testing Pose<double, 2, true, true> Pose constructor" << std::endl;
    Pose<double, 2, true, true> adx_posed2tt_constructor(ros_pose);
    compare_position<double, 2>(ros_pose.position, adx_posed2tt_constructor.position);
    compare_yaw<double, double>(yaw, adx_posed2tt_constructor.yaw());
    std::cout << "Testing Pose<double, 2, true, true> Pose assignment" << std::endl;
    Pose<double, 2, true, true> adx_posed2tt_assignment;
    adx_posed2tt_assignment = ros_pose;
    compare_position<double, 2>(ros_pose.position, adx_posed2tt_assignment.position);
    compare_yaw<double, double>(yaw, adx_posed2tt_assignment.yaw());
    std::cout << "Testing Pose<float, 2, true, true> Pose constructor" << std::endl;
    Pose<float, 2, true, true> adx_posef2tt_constructor(ros_pose);
    compare_position<float, 2>(ros_pose.position, adx_posef2tt_constructor.position);
    compare_yaw<double, float>(yaw, adx_posef2tt_constructor.yaw());
    std::cout << "Testing Pose<float, 2, true, true> Pose assignment" << std::endl;
    Pose<float, 2, true, true> adx_posef2tt_assignment;
    adx_posef2tt_assignment = ros_pose;
    compare_position<float, 2>(ros_pose.position, adx_posef2tt_assignment.position);
    compare_yaw<double, float>(yaw, adx_posef2tt_assignment.yaw());

    std::cout << "Testing Pose<double, 2, false, true> Pose constructor" << std::endl;
    Pose<double, 2, false, true> adx_posed2ft_constructor(ros_pose);
    compare_position<double, 2>(ros_pose.position, adx_posed2ft_constructor.position);
    compare_yaw<double, double>(yaw, adx_posed2ft_constructor.yaw());
    std::cout << "Testing Pose<double, 2, false, true> Pose assignment" << std::endl;
    Pose<double, 2, false, true> adx_posed2ft_assignment;
    adx_posed2ft_assignment = ros_pose;
    compare_position<double, 2>(ros_pose.position, adx_posed2ft_assignment.position);
    compare_yaw<double, double>(yaw, adx_posed2ft_assignment.yaw());
    std::cout << "Testing Pose<float, 2, false, true> Pose constructor" << std::endl;
    Pose<float, 2, false, true> adx_posef2ft_constructor(ros_pose);
    compare_position<float, 2>(ros_pose.position, adx_posef2ft_constructor.position);
    compare_yaw<double, float>(yaw, adx_posef2ft_constructor.yaw());
    std::cout << "Testing Pose<float, 2, false, true> Pose assignment" << std::endl;
    Pose<float, 2, false, true> adx_posef2ft_assignment;
    adx_posef2ft_assignment = ros_pose;
    compare_position<float, 2>(ros_pose.position, adx_posef2ft_assignment.position);
    compare_yaw<double, float>(yaw, adx_posef2ft_assignment.yaw());

    std::cout << "Testing Pose<double, 2, true, false> Pose constructor" << std::endl;
    Pose<double, 2, true, false> adx_posed2tf_constructor(ros_pose);
    compare_position<double, 2>(ros_pose.position, adx_posed2tf_constructor.position);
    compare_yaw<double, double>(yaw, adx_posed2tf_constructor.yaw());
    std::cout << "Testing Pose<double, 2, true, false> Pose assignment" << std::endl;
    Pose<double, 2, true, false> adx_posed2tf_assignment;
    adx_posed2tf_assignment = ros_pose;
    compare_position<double, 2>(ros_pose.position, adx_posed2tf_assignment.position);
    compare_yaw<double, double>(yaw, adx_posed2tf_assignment.yaw());
    std::cout << "Testing Pose<float, 2, true, false> Pose constructor" << std::endl;
    Pose<float, 2, true, false> adx_posef2tf_constructor(ros_pose);
    compare_position<float, 2>(ros_pose.position, adx_posef2tf_constructor.position);
    compare_yaw<double, float>(yaw, adx_posef2tf_constructor.yaw());
    std::cout << "Testing Pose<float, 2, true, false> Pose assignment" << std::endl;
    Pose<float, 2, true, false> adx_posef2tf_assignment;
    adx_posef2tf_assignment = ros_pose;
    compare_position<float, 2>(ros_pose.position, adx_posef2tf_assignment.position);
    compare_yaw<double, float>(yaw, adx_posef2tf_assignment.yaw());

    std::cout << "Testing Pose<double, 2, false, false> Pose constructor" << std::endl;
    Pose<double, 2, false, false> adx_posed2ff_constructor(ros_pose);
    compare_position<double, 2>(ros_pose.position, adx_posed2ff_constructor.position);
    compare_yaw<double, double>(yaw, adx_posed2ff_constructor.yaw());
    std::cout << "Testing Pose<double, 2, false, false> Pose assignment" << std::endl;
    Pose<double, 2, false, false> adx_posed2ff_assignment;
    adx_posed2ff_assignment = ros_pose;
    compare_position<double, 2>(ros_pose.position, adx_posed2ff_assignment.position);
    compare_yaw<double, double>(yaw, adx_posed2ff_assignment.yaw());
    std::cout << "Testing Pose<float, 2, false, false> Pose constructor" << std::endl;
    Pose<float, 2, false, false> adx_posef2ff_constructor(ros_pose);
    compare_position<float, 2>(ros_pose.position, adx_posef2ff_constructor.position);
    compare_yaw<double, float>(yaw, adx_posef2ff_constructor.yaw());
    std::cout << "Testing Pose<float, 2, false, false> Pose assignment" << std::endl;
    Pose<float, 2, false, false> adx_posef2ff_assignment;
    adx_posef2ff_assignment = ros_pose;
    compare_position<float, 2>(ros_pose.position, adx_posef2ff_assignment.position);
    compare_yaw<double, float>(yaw, adx_posef2ff_assignment.yaw());
}

TEST_F(PoseTest, ToRosPose)
{
    const double x = 1.0, y = 2.0, z = 3.0, roll = 0.1, pitch = 0.2, yaw = 0.3;

    std::cout << "Testing Pose<double, 3, true, true> toRos Pose" << std::endl;
    Pose<double, 3, true, true> adx_posed3tt;
    adx_posed3tt.position = { x, y, z };
    adx_posed3tt.orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Pose ros_pose_d3tt = adx_posed3tt.PoseBase::toRos();
    compare_position<double, 3>(ros_pose_d3tt.position, adx_posed3tt.position);
    compare_orientation<double>(ros_pose_d3tt.orientation, adx_posed3tt.orientation);
    std::cout << "Testing Pose<float, 3, true, true> toRos Pose" << std::endl;
    Pose<float, 3, true, true> adx_posef3tt;
    adx_posef3tt.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    adx_posef3tt.orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Pose ros_pose_f3tt = adx_posef3tt.PoseBase::toRos();
    compare_position<float, 3>(ros_pose_f3tt.position, adx_posef3tt.position);
    compare_orientation<float>(ros_pose_f3tt.orientation, adx_posef3tt.orientation);

    std::cout << "Testing Pose<double, 3, false, true> toRos Pose" << std::endl;
    Pose<double, 3, false, true> adx_posed3ft;
    adx_posed3ft.position = { x, y, z };
    adx_posed3ft.orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Pose ros_pose_d3ft = adx_posed3ft.PoseBase::toRos();
    compare_position<double, 3>(ros_pose_d3ft.position, adx_posed3ft.position);
    compare_orientation<double>(ros_pose_d3ft.orientation, adx_posed3ft.orientation);
    std::cout << "Testing Pose<float, 3, false, true> toRos Pose" << std::endl;
    Pose<float, 3, false, true> adx_posef3ft;
    adx_posef3ft.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    adx_posef3ft.orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Pose ros_pose_f3ft = adx_posef3ft.PoseBase::toRos();
    compare_position<float, 3>(ros_pose_f3ft.position, adx_posef3ft.position);
    compare_orientation<float>(ros_pose_f3ft.orientation, adx_posef3ft.orientation);

    std::cout << "Testing Pose<double, 3, true, false> toRos Pose" << std::endl;
    Pose<double, 3, true, false> adx_posed3tf;
    adx_posed3tf.position = { x, y, z };
    adx_posed3tf.orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Pose ros_pose_d3tf = adx_posed3tf.PoseBase::toRos();
    compare_position<double, 3>(ros_pose_d3tf.position, adx_posed3tf.position);
    compare_orientation<double>(ros_pose_d3tf.orientation, adx_posed3tf.orientation);
    std::cout << "Testing Pose<float, 3, true, false> toRos Pose" << std::endl;
    Pose<float, 3, true, false> adx_posef3tf;
    adx_posef3tf.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    adx_posef3tf.orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Pose ros_pose_f3tf = adx_posef3tf.PoseBase::toRos();
    compare_position<float, 3>(ros_pose_f3tf.position, adx_posef3tf.position);
    compare_orientation<float>(ros_pose_f3tf.orientation, adx_posef3tf.orientation);

    std::cout << "Testing Pose<double, 3, false, false> toRos Pose" << std::endl;
    Pose<double, 3, false, false> adx_posed3ff;
    adx_posed3ff.position = { x, y, z };
    adx_posed3ff.orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Pose ros_pose_d3ff = adx_posed3ff.PoseBase::toRos();
    compare_position<double, 3>(ros_pose_d3ff.position, adx_posed3ff.position);
    compare_orientation<double>(ros_pose_d3ff.orientation, adx_posed3ff.orientation);
    std::cout << "Testing Pose<float, 3, false, false> toRos Pose" << std::endl;
    Pose<float, 3, false, false> adx_posef3ff;
    adx_posef3ff.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    adx_posef3ff.orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::Pose ros_pose_f3ff = adx_posef3ff.PoseBase::toRos();
    compare_position<float, 3>(ros_pose_f3ff.position, adx_posef3ff.position);
    compare_orientation<float>(ros_pose_f3ff.orientation, adx_posef3ff.orientation);

    std::cout << "Testing Pose<double, 2, true, true> toRos Pose" << std::endl;
    Pose<double, 2, true, true> adx_posed2tt;
    adx_posed2tt.position = { x, y };
    adx_posed2tt.yaw() = yaw;
    geometry_msgs::msg::Pose ros_pose_d2tt;
    ros_pose_d2tt.position.z = z;
    ros_pose_d2tt = adx_posed2tt.PoseBase::toRos();
    compare_position<double, 2>(ros_pose_d2tt.position, adx_posed2tt.position);
    compare_yaw<double, double>(yaw_from_quaternion(ros_pose_d2tt.orientation), adx_posed2tt.yaw());
    EXPECT_EQ(ros_pose_d2tt.position.z, 0.0);
    EXPECT_EQ(ros_pose_d2tt.orientation.x, 0.0);
    EXPECT_EQ(ros_pose_d2tt.orientation.y, 0.0);
    std::cout << "Testing Pose<float, 2, true, true> toRos Pose" << std::endl;
    Pose<float, 2, true, true> adx_posef2tt;
    adx_posef2tt.position = { x, y };
    adx_posef2tt.yaw() = yaw;
    geometry_msgs::msg::Pose ros_pose_f2tt;
    ros_pose_f2tt.position.z = z;
    ros_pose_f2tt = adx_posef2tt.PoseBase::toRos();
    compare_position<float, 2>(ros_pose_f2tt.position, adx_posef2tt.position);
    compare_yaw<double, float>(yaw_from_quaternion(ros_pose_f2tt.orientation), adx_posef2tt.yaw());
    EXPECT_EQ(ros_pose_f2tt.position.z, 0.0);
    EXPECT_EQ(ros_pose_f2tt.orientation.x, 0.0);
    EXPECT_EQ(ros_pose_f2tt.orientation.y, 0.0);

    std::cout << "Testing Pose<double, 2, false, true> toRos Pose" << std::endl;
    Pose<double, 2, false, true> adx_posed2ft;
    adx_posed2ft.position = { x, y };
    adx_posed2ft.yaw() = yaw;
    geometry_msgs::msg::Pose ros_pose_d2ft;
    ros_pose_d2ft.position.z = z;
    ros_pose_d2ft = adx_posed2ft.PoseBase::toRos();
    compare_position<double, 2>(ros_pose_d2ft.position, adx_posed2ft.position);
    compare_yaw<double, double>(yaw_from_quaternion(ros_pose_d2ft.orientation), adx_posed2ft.yaw());
    EXPECT_EQ(ros_pose_d2ft.position.z, 0.0);
    EXPECT_EQ(ros_pose_d2ft.orientation.x, 0.0);
    EXPECT_EQ(ros_pose_d2ft.orientation.y, 0.0);
    std::cout << "Testing Pose<float, 2, false, true> toRos Pose" << std::endl;
    Pose<float, 2, false, true> adx_posef2ft;
    adx_posef2ft.position = { x, y };
    adx_posef2ft.yaw() = yaw;
    geometry_msgs::msg::Pose ros_pose_f2ft;
    ros_pose_f2ft.position.z = z;
    ros_pose_f2ft = adx_posef2ft.PoseBase::toRos();
    compare_position<float, 2>(ros_pose_f2ft.position, adx_posef2ft.position);
    compare_yaw<double, float>(yaw_from_quaternion(ros_pose_f2ft.orientation), adx_posef2ft.yaw());
    EXPECT_EQ(ros_pose_f2ft.position.z, 0.0);
    EXPECT_EQ(ros_pose_f2ft.orientation.x, 0.0);
    EXPECT_EQ(ros_pose_f2ft.orientation.y, 0.0);

    std::cout << "Testing Pose<double, 2, true, false> toRos Pose" << std::endl;
    Pose<double, 2, true, false> adx_posed2tf;
    adx_posed2tf.position = { x, y };
    adx_posed2tf.yaw() = yaw;
    geometry_msgs::msg::Pose ros_pose_d2tf;
    ros_pose_d2tf.position.z = z;
    ros_pose_d2tf = adx_posed2tf.PoseBase::toRos();
    compare_position<double, 2>(ros_pose_d2tf.position, adx_posed2tf.position);
    compare_yaw<double, double>(yaw_from_quaternion(ros_pose_d2tf.orientation), adx_posed2tf.yaw());
    EXPECT_EQ(ros_pose_d2tf.position.z, 0.0);
    EXPECT_EQ(ros_pose_d2tf.orientation.x, 0.0);
    EXPECT_EQ(ros_pose_d2tf.orientation.y, 0.0);
    std::cout << "Testing Pose<float, 2, true, false> toRos Pose" << std::endl;
    Pose<float, 2, true, false> adx_posef2tf;
    adx_posef2tf.position = { x, y };
    adx_posef2tf.yaw() = yaw;
    geometry_msgs::msg::Pose ros_pose_f2tf;
    ros_pose_f2tf.position.z = z;
    ros_pose_f2tf = adx_posef2tf.PoseBase::toRos();
    compare_position<float, 2>(ros_pose_f2tf.position, adx_posef2tf.position);
    compare_yaw<double, float>(yaw_from_quaternion(ros_pose_f2tf.orientation), adx_posef2tf.yaw());
    EXPECT_EQ(ros_pose_f2tf.position.z, 0.0);
    EXPECT_EQ(ros_pose_f2tf.orientation.x, 0.0);
    EXPECT_EQ(ros_pose_f2tf.orientation.y, 0.0);

    std::cout << "Testing Pose<double, 2, false, false> toRos Pose" << std::endl;
    Pose<double, 2, false, false> adx_posed2ff;
    adx_posed2ff.position = { x, y };
    adx_posed2ff.yaw() = yaw;
    geometry_msgs::msg::Pose ros_pose_d2ff;
    ros_pose_d2ff.position.z = z;
    ros_pose_d2ff = adx_posed2ff.PoseBase::toRos();
    compare_position<double, 2>(ros_pose_d2ff.position, adx_posed2ff.position);
    compare_yaw<double, double>(yaw_from_quaternion(ros_pose_d2ff.orientation), adx_posed2ff.yaw());
    EXPECT_EQ(ros_pose_d2ff.position.z, 0.0);
    EXPECT_EQ(ros_pose_d2ff.orientation.x, 0.0);
    EXPECT_EQ(ros_pose_d2ff.orientation.y, 0.0);
    std::cout << "Testing Pose<float, 2, false, false> toRos Pose" << std::endl;
    Pose<float, 2, false, false> adx_posef2ff;
    adx_posef2ff.position = { x, y };
    adx_posef2ff.yaw() = yaw;
    geometry_msgs::msg::Pose ros_pose_f2ff;
    ros_pose_f2ff.position.z = z;
    ros_pose_f2ff = adx_posef2ff.PoseBase::toRos();
    compare_position<float, 2>(ros_pose_f2ff.position, adx_posef2ff.position);
    compare_yaw<double, float>(yaw_from_quaternion(ros_pose_f2ff.orientation), adx_posef2ff.yaw());
    EXPECT_EQ(ros_pose_f2ff.position.z, 0.0);
    EXPECT_EQ(ros_pose_f2ff.orientation.x, 0.0);
    EXPECT_EQ(ros_pose_f2ff.orientation.y, 0.0);
}

TEST_F(PoseTest, FromRosPoseStamped)
{
    const double x = 1.0, y = 2.0, z = 3.0, roll = 0.1, pitch = 0.2, yaw = 0.3;
    geometry_msgs::msg::PoseStamped ros_pose_stamped;

    ros_pose_stamped.header.stamp = mNode->get_clock()->now();
    ros_pose_stamped.header.frame_id = "map";

    ros_pose_stamped.pose.position.x = x;
    ros_pose_stamped.pose.position.y = y;
    ros_pose_stamped.pose.position.z = z;

    tf2::Quaternion tf2_quaternion;
    tf2_quaternion.setRPY(roll, pitch, yaw);

    ros_pose_stamped.pose.orientation.w = tf2_quaternion.w();
    ros_pose_stamped.pose.orientation.x = tf2_quaternion.x();
    ros_pose_stamped.pose.orientation.y = tf2_quaternion.y();
    ros_pose_stamped.pose.orientation.z = tf2_quaternion.z();

    std::cout << "Testing Pose<double, 3, true, true> PoseStamped constructor" << std::endl;
    Pose<double, 3, true, true> adx_posed3tt_constructor(ros_pose_stamped);
    compare_header(ros_pose_stamped.header, static_cast<Header>(adx_posed3tt_constructor));
    compare_position<double, 3>(ros_pose_stamped.pose.position, adx_posed3tt_constructor.position);
    compare_orientation<double>(ros_pose_stamped.pose.orientation,
                                adx_posed3tt_constructor.orientation);
    std::cout << "Testing Pose<double, 3, true, true> PoseStamped assignment" << std::endl;
    Pose<double, 3, true, true> adx_posed3tt_assignment;
    adx_posed3tt_assignment = ros_pose_stamped;
    compare_header(ros_pose_stamped.header, static_cast<Header>(adx_posed3tt_assignment));
    compare_position<double, 3>(ros_pose_stamped.pose.position, adx_posed3tt_assignment.position);
    compare_orientation<double>(ros_pose_stamped.pose.orientation,
                                adx_posed3tt_assignment.orientation);
    std::cout << "Testing Pose<float, 3, true, true> PoseStamped constructor" << std::endl;
    Pose<float, 3, true, true> adx_posef3tt_constructor(ros_pose_stamped);
    compare_header(ros_pose_stamped.header, static_cast<Header>(adx_posef3tt_constructor));
    compare_position<float, 3>(ros_pose_stamped.pose.position, adx_posef3tt_constructor.position);
    compare_orientation<float>(ros_pose_stamped.pose.orientation,
                               adx_posef3tt_constructor.orientation);
    std::cout << "Testing Pose<float, 3, true, true> PoseStamped assignment" << std::endl;
    Pose<float, 3, true, true> adx_posef3tt_assignment;
    adx_posef3tt_assignment = ros_pose_stamped;
    compare_header(ros_pose_stamped.header, static_cast<Header>(adx_posef3tt_assignment));
    compare_position<float, 3>(ros_pose_stamped.pose.position, adx_posef3tt_assignment.position);
    compare_orientation<float>(ros_pose_stamped.pose.orientation,
                               adx_posef3tt_assignment.orientation);

    std::cout << "Testing Pose<double, 3, false, true> PoseStamped constructor" << std::endl;
    Pose<double, 3, false, true> adx_posed3ft_constructor(ros_pose_stamped);
    compare_position<double, 3>(ros_pose_stamped.pose.position, adx_posed3ft_constructor.position);
    compare_orientation<double>(ros_pose_stamped.pose.orientation,
                                adx_posed3ft_constructor.orientation);
    std::cout << "Testing Pose<double, 3, false, true> PoseStamped assignment" << std::endl;
    Pose<double, 3, false, true> adx_posed3ft_assignment;
    adx_posed3ft_assignment = ros_pose_stamped;
    compare_position<double, 3>(ros_pose_stamped.pose.position, adx_posed3ft_assignment.position);
    compare_orientation<double>(ros_pose_stamped.pose.orientation,
                                adx_posed3ft_assignment.orientation);
    std::cout << "Testing Pose<float, 3, false, true> PoseStamped constructor" << std::endl;
    Pose<float, 3, false, true> adx_posef3ft_constructor(ros_pose_stamped);
    compare_position<float, 3>(ros_pose_stamped.pose.position, adx_posef3ft_constructor.position);
    compare_orientation<float>(ros_pose_stamped.pose.orientation,
                               adx_posef3ft_constructor.orientation);
    std::cout << "Testing Pose<float, 3, false, true> PoseStamped assignment" << std::endl;
    Pose<float, 3, false, true> adx_posef3ft_assignment;
    adx_posef3ft_assignment = ros_pose_stamped;
    compare_position<float, 3>(ros_pose_stamped.pose.position, adx_posef3ft_assignment.position);
    compare_orientation<float>(ros_pose_stamped.pose.orientation,
                               adx_posef3ft_assignment.orientation);

    std::cout << "Testing Pose<double, 3, true, false> PoseStamped constructor" << std::endl;
    Pose<double, 3, true, false> adx_posed3tf_constructor(ros_pose_stamped);
    compare_header(ros_pose_stamped.header, static_cast<Header>(adx_posed3tf_constructor));
    compare_position<double, 3>(ros_pose_stamped.pose.position, adx_posed3tf_constructor.position);
    compare_orientation<double>(ros_pose_stamped.pose.orientation,
                                adx_posed3tf_constructor.orientation);
    std::cout << "Testing Pose<double, 3, true, false> PoseStamped assignment" << std::endl;
    Pose<double, 3, true, false> adx_posed3tf_assignment;
    adx_posed3tf_assignment = ros_pose_stamped;
    compare_header(ros_pose_stamped.header, static_cast<Header>(adx_posed3tf_assignment));
    compare_position<double, 3>(ros_pose_stamped.pose.position, adx_posed3tf_assignment.position);
    compare_orientation<double>(ros_pose_stamped.pose.orientation,
                                adx_posed3tf_assignment.orientation);
    std::cout << "Testing Pose<float, 3, true, false> PoseStamped constructor" << std::endl;
    Pose<float, 3, true, false> adx_posef3tf_constructor(ros_pose_stamped);
    compare_header(ros_pose_stamped.header, static_cast<Header>(adx_posef3tf_constructor));
    compare_position<float, 3>(ros_pose_stamped.pose.position, adx_posef3tf_constructor.position);
    compare_orientation<float>(ros_pose_stamped.pose.orientation,
                               adx_posef3tf_constructor.orientation);
    std::cout << "Testing Pose<float, 3, true, false> PoseStamped assignment" << std::endl;
    Pose<float, 3, true, false> adx_posef3tf_assignment;
    adx_posef3tf_assignment = ros_pose_stamped;
    compare_header(ros_pose_stamped.header, static_cast<Header>(adx_posef3tf_assignment));
    compare_position<float, 3>(ros_pose_stamped.pose.position, adx_posef3tf_assignment.position);
    compare_orientation<float>(ros_pose_stamped.pose.orientation,
                               adx_posef3tf_assignment.orientation);

    std::cout << "Testing Pose<double, 3, false, false> PoseStamped constructor" << std::endl;
    Pose<double, 3, false, false> adx_posed3ff_constructor(ros_pose_stamped);
    compare_position<double, 3>(ros_pose_stamped.pose.position, adx_posed3ff_constructor.position);
    compare_orientation<double>(ros_pose_stamped.pose.orientation,
                                adx_posed3ff_constructor.orientation);
    std::cout << "Testing Pose<double, 3, false, false> PoseStamped assignment" << std::endl;
    Pose<double, 3, false, false> adx_posed3ff_assignment;
    adx_posed3ff_assignment = ros_pose_stamped;
    compare_position<double, 3>(ros_pose_stamped.pose.position, adx_posed3ff_assignment.position);
    compare_orientation<double>(ros_pose_stamped.pose.orientation,
                                adx_posed3ff_assignment.orientation);
    std::cout << "Testing Pose<float, 3, false, false> PoseStamped constructor" << std::endl;
    Pose<float, 3, false, false> adx_posef3ff_constructor(ros_pose_stamped);
    compare_position<float, 3>(ros_pose_stamped.pose.position, adx_posef3ff_constructor.position);
    compare_orientation<float>(ros_pose_stamped.pose.orientation,
                               adx_posef3ff_constructor.orientation);
    std::cout << "Testing Pose<float, 3, false, false> PoseStamped assignment" << std::endl;
    Pose<float, 3, false, false> adx_posef3ff_assignment;
    adx_posef3ff_assignment = ros_pose_stamped;
    compare_position<float, 3>(ros_pose_stamped.pose.position, adx_posef3ff_assignment.position);
    compare_orientation<float>(ros_pose_stamped.pose.orientation,
                               adx_posef3ff_assignment.orientation);

    tf2_quaternion.setRPY(0.0, 0.0, yaw);

    ros_pose_stamped.pose.orientation.w = tf2_quaternion.w();
    ros_pose_stamped.pose.orientation.x = tf2_quaternion.x();
    ros_pose_stamped.pose.orientation.y = tf2_quaternion.y();
    ros_pose_stamped.pose.orientation.z = tf2_quaternion.z();

    std::cout << "Testing Pose<double, 2, true, true> PoseStamped constructor" << std::endl;
    Pose<double, 2, true, true> adx_posed2tt_constructor(ros_pose_stamped);
    compare_header(ros_pose_stamped.header, static_cast<Header>(adx_posed2tt_constructor));
    compare_position<double, 2>(ros_pose_stamped.pose.position, adx_posed2tt_constructor.position);
    compare_yaw<double, double>(yaw, adx_posed2tt_constructor.yaw());
    std::cout << "Testing Pose<double, 2, true, true> PoseStamped assignment" << std::endl;
    Pose<double, 2, true, true> adx_posed2tt_assignment;
    adx_posed2tt_assignment = ros_pose_stamped;
    compare_header(ros_pose_stamped.header, static_cast<Header>(adx_posed2tt_assignment));
    compare_position<double, 2>(ros_pose_stamped.pose.position, adx_posed2tt_assignment.position);
    compare_yaw<double, double>(yaw, adx_posed2tt_assignment.yaw());
    std::cout << "Testing Pose<float, 2, true, true> PoseStamped constructor" << std::endl;
    Pose<float, 2, true, true> adx_posef2tt_constructor(ros_pose_stamped);
    compare_header(ros_pose_stamped.header, static_cast<Header>(adx_posef2tt_constructor));
    compare_position<float, 2>(ros_pose_stamped.pose.position, adx_posef2tt_constructor.position);
    compare_yaw<double, float>(yaw, adx_posef2tt_constructor.yaw());
    std::cout << "Testing Pose<float, 2, true, true> PoseStamped assignment" << std::endl;
    Pose<float, 2, true, true> adx_posef2tt_assignment;
    adx_posef2tt_assignment = ros_pose_stamped;
    compare_header(ros_pose_stamped.header, static_cast<Header>(adx_posef2tt_assignment));
    compare_position<float, 2>(ros_pose_stamped.pose.position, adx_posef2tt_assignment.position);
    compare_yaw<double, float>(yaw, adx_posef2tt_assignment.yaw());

    std::cout << "Testing Pose<double, 2, false, true> PoseStamped constructor" << std::endl;
    Pose<double, 2, false, true> adx_posed2ft_constructor(ros_pose_stamped);
    compare_position<double, 2>(ros_pose_stamped.pose.position, adx_posed2ft_constructor.position);
    compare_yaw<double, double>(yaw, adx_posed2ft_constructor.yaw());
    std::cout << "Testing Pose<double, 2, false, true> PoseStamped assignment" << std::endl;
    Pose<double, 2, false, true> adx_posed2ft_assignment;
    adx_posed2ft_assignment = ros_pose_stamped;
    compare_position<double, 2>(ros_pose_stamped.pose.position, adx_posed2ft_assignment.position);
    compare_yaw<double, double>(yaw, adx_posed2ft_assignment.yaw());
    std::cout << "Testing Pose<float, 2, false, true> PoseStamped constructor" << std::endl;
    Pose<float, 2, false, true> adx_posef2ft_constructor(ros_pose_stamped);
    compare_position<float, 2>(ros_pose_stamped.pose.position, adx_posef2ft_constructor.position);
    compare_yaw<double, float>(yaw, adx_posef2ft_constructor.yaw());
    std::cout << "Testing Pose<float, 2, false, true> PoseStamped assignment" << std::endl;
    Pose<float, 2, false, true> adx_posef2ft_assignment;
    adx_posef2ft_assignment = ros_pose_stamped;
    compare_position<float, 2>(ros_pose_stamped.pose.position, adx_posef2ft_assignment.position);
    compare_yaw<double, float>(yaw, adx_posef2ft_assignment.yaw());

    std::cout << "Testing Pose<double, 2, true, false> PoseStamped constructor" << std::endl;
    Pose<double, 2, true, false> adx_posed2tf_constructor(ros_pose_stamped);
    compare_header(ros_pose_stamped.header, static_cast<Header>(adx_posed2tf_constructor));
    compare_position<double, 2>(ros_pose_stamped.pose.position, adx_posed2tf_constructor.position);
    compare_yaw<double, double>(yaw, adx_posed2tf_constructor.yaw());
    std::cout << "Testing Pose<double, 2, true, false> PoseStamped assignment" << std::endl;
    Pose<double, 2, true, false> adx_posed2tf_assignment;
    adx_posed2tf_assignment = ros_pose_stamped;
    compare_header(ros_pose_stamped.header, static_cast<Header>(adx_posed2tf_assignment));
    compare_position<double, 2>(ros_pose_stamped.pose.position, adx_posed2tf_assignment.position);
    compare_yaw<double, double>(yaw, adx_posed2tf_assignment.yaw());
    std::cout << "Testing Pose<float, 2, true, false> PoseStamped constructor" << std::endl;
    Pose<float, 2, true, false> adx_posef2tf_constructor(ros_pose_stamped);
    compare_header(ros_pose_stamped.header, static_cast<Header>(adx_posef2tf_constructor));
    compare_position<float, 2>(ros_pose_stamped.pose.position, adx_posef2tf_constructor.position);
    compare_yaw<double, float>(yaw, adx_posef2tf_constructor.yaw());
    std::cout << "Testing Pose<float, 2, true, false> PoseStamped assignment" << std::endl;
    Pose<float, 2, true, false> adx_posef2tf_assignment;
    adx_posef2tf_assignment = ros_pose_stamped;
    compare_header(ros_pose_stamped.header, static_cast<Header>(adx_posef2tf_assignment));
    compare_position<float, 2>(ros_pose_stamped.pose.position, adx_posef2tf_assignment.position);
    compare_yaw<double, float>(yaw, adx_posef2tf_assignment.yaw());

    std::cout << "Testing Pose<double, 2, false, false> PoseStamped constructor" << std::endl;
    Pose<double, 2, false, false> adx_posed2ff_constructor(ros_pose_stamped);
    compare_position<double, 2>(ros_pose_stamped.pose.position, adx_posed2ff_constructor.position);
    compare_yaw<double, double>(yaw, adx_posed2ff_constructor.yaw());
    std::cout << "Testing Pose<double, 2, false, false> PoseStamped assignment" << std::endl;
    Pose<double, 2, false, false> adx_posed2ff_assignment;
    adx_posed2ff_assignment = ros_pose_stamped;
    compare_position<double, 2>(ros_pose_stamped.pose.position, adx_posed2ff_assignment.position);
    compare_yaw<double, double>(yaw, adx_posed2ff_assignment.yaw());
    std::cout << "Testing Pose<float, 2, false, false> PoseStamped constructor" << std::endl;
    Pose<float, 2, false, false> adx_posef2ff_constructor(ros_pose_stamped);
    compare_position<float, 2>(ros_pose_stamped.pose.position, adx_posef2ff_constructor.position);
    compare_yaw<double, float>(yaw, adx_posef2ff_constructor.yaw());
    std::cout << "Testing Pose<float, 2, false, false> PoseStamped assignment" << std::endl;
    Pose<float, 2, false, false> adx_posef2ff_assignment;
    adx_posef2ff_assignment = ros_pose_stamped;
    compare_position<float, 2>(ros_pose_stamped.pose.position, adx_posef2ff_assignment.position);
    compare_yaw<double, float>(yaw, adx_posef2ff_assignment.yaw());
}

TEST_F(PoseTest, ToRosPoseStamped)
{
    const double x = 1.0, y = 2.0, z = 3.0, roll = 0.1, pitch = 0.2, yaw = 0.3;

    std::cout << "Testing Pose<double, 3, true, true> toRos Pose" << std::endl;
    Pose<double, 3, true, true> adx_posed3tt;
    adx_posed3tt.timestamp = std::chrono::steady_clock::now();
    adx_posed3tt.frame_id = "map";
    adx_posed3tt.position = { x, y, z };
    adx_posed3tt.orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::PoseStamped ros_pose_d3tt = adx_posed3tt.PoseHeader::toRos();
    compare_header(ros_pose_d3tt.header, static_cast<Header>(adx_posed3tt));
    compare_position<double, 3>(ros_pose_d3tt.pose.position, adx_posed3tt.position);
    compare_orientation<double>(ros_pose_d3tt.pose.orientation, adx_posed3tt.orientation);
    std::cout << "Testing Pose<float, 3, true, true> toRos Pose" << std::endl;
    Pose<float, 3, true, true> adx_posef3tt;
    adx_posef3tt.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    adx_posef3tt.orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::PoseStamped ros_pose_f3tt = adx_posef3tt.PoseHeader::toRos();
    compare_header(ros_pose_f3tt.header, static_cast<Header>(adx_posef3tt));
    compare_position<float, 3>(ros_pose_f3tt.pose.position, adx_posef3tt.position);
    compare_orientation<float>(ros_pose_f3tt.pose.orientation, adx_posef3tt.orientation);

    std::cout << "Testing Pose<double, 3, false, true> toRos Pose" << std::endl;
    Pose<double, 3, false, true> adx_posed3ft;
    adx_posed3ft.position = { x, y, z };
    adx_posed3ft.orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::PoseStamped ros_pose_d3ft = adx_posed3ft.PoseHeader::toRos();
    compare_position<double, 3>(ros_pose_d3ft.pose.position, adx_posed3ft.position);
    compare_orientation<double>(ros_pose_d3ft.pose.orientation, adx_posed3ft.orientation);
    std::cout << "Testing Pose<float, 3, false, true> toRos Pose" << std::endl;
    Pose<float, 3, false, true> adx_posef3ft;
    adx_posef3ft.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    adx_posef3ft.orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::PoseStamped ros_pose_f3ft = adx_posef3ft.PoseHeader::toRos();
    compare_position<float, 3>(ros_pose_f3ft.pose.position, adx_posef3ft.position);
    compare_orientation<float>(ros_pose_f3ft.pose.orientation, adx_posef3ft.orientation);

    std::cout << "Testing Pose<double, 3, true, false> toRos Pose" << std::endl;
    Pose<double, 3, true, false> adx_posed3tf;
    adx_posed3tf.position = { x, y, z };
    adx_posed3tf.orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::PoseStamped ros_pose_d3tf = adx_posed3tf.PoseHeader::toRos();
    compare_header(ros_pose_d3tf.header, static_cast<Header>(adx_posed3tf));
    compare_position<double, 3>(ros_pose_d3tf.pose.position, adx_posed3tf.position);
    compare_orientation<double>(ros_pose_d3tf.pose.orientation, adx_posed3tf.orientation);
    std::cout << "Testing Pose<float, 3, true, false> toRos Pose" << std::endl;
    Pose<float, 3, true, false> adx_posef3tf;
    adx_posef3tf.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    adx_posef3tf.orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::PoseStamped ros_pose_f3tf = adx_posef3tf.PoseHeader::toRos();
    compare_header(ros_pose_f3tf.header, static_cast<Header>(adx_posef3tf));
    compare_position<float, 3>(ros_pose_f3tf.pose.position, adx_posef3tf.position);
    compare_orientation<float>(ros_pose_f3tf.pose.orientation, adx_posef3tf.orientation);

    std::cout << "Testing Pose<double, 3, false, false> toRos Pose" << std::endl;
    Pose<double, 3, false, false> adx_posed3ff;
    adx_posed3ff.position = { x, y, z };
    adx_posed3ff.orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::PoseStamped ros_pose_d3ff = adx_posed3ff.PoseHeader::toRos();
    compare_position<double, 3>(ros_pose_d3ff.pose.position, adx_posed3ff.position);
    compare_orientation<double>(ros_pose_d3ff.pose.orientation, adx_posed3ff.orientation);
    std::cout << "Testing Pose<float, 3, false, false> toRos Pose" << std::endl;
    Pose<float, 3, false, false> adx_posef3ff;
    adx_posef3ff.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    adx_posef3ff.orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::PoseStamped ros_pose_f3ff = adx_posef3ff.PoseHeader::toRos();
    compare_position<float, 3>(ros_pose_f3ff.pose.position, adx_posef3ff.position);
    compare_orientation<float>(ros_pose_f3ff.pose.orientation, adx_posef3ff.orientation);

    std::cout << "Testing Pose<double, 2, true, true> toRos Pose" << std::endl;
    Pose<double, 2, true, true> adx_posed2tt;
    adx_posed2tt.position = { x, y };
    adx_posed2tt.yaw() = yaw;
    geometry_msgs::msg::PoseStamped ros_pose_d2tt;
    ros_pose_d2tt.pose.position.z = z;
    ros_pose_d2tt = adx_posed2tt.PoseHeader::toRos();
    compare_header(ros_pose_d2tt.header, static_cast<Header>(adx_posed2tt));
    compare_position<double, 2>(ros_pose_d2tt.pose.position, adx_posed2tt.position);
    compare_yaw<double, double>(yaw_from_quaternion(ros_pose_d2tt.pose.orientation),
                                adx_posed2tt.yaw());
    EXPECT_EQ(ros_pose_d2tt.pose.position.z, 0.0);
    EXPECT_EQ(ros_pose_d2tt.pose.orientation.x, 0.0);
    EXPECT_EQ(ros_pose_d2tt.pose.orientation.y, 0.0);
    std::cout << "Testing Pose<float, 2, true, true> toRos Pose" << std::endl;
    Pose<float, 2, true, true> adx_posef2tt;
    adx_posef2tt.position = { x, y };
    adx_posef2tt.yaw() = yaw;
    geometry_msgs::msg::PoseStamped ros_pose_f2tt;
    ros_pose_f2tt.pose.position.z = z;
    ros_pose_f2tt = adx_posef2tt.PoseHeader::toRos();
    compare_header(ros_pose_f2tt.header, static_cast<Header>(adx_posef2tt));
    compare_position<float, 2>(ros_pose_f2tt.pose.position, adx_posef2tt.position);
    compare_yaw<double, float>(yaw_from_quaternion(ros_pose_f2tt.pose.orientation),
                               adx_posef2tt.yaw());
    EXPECT_EQ(ros_pose_f2tt.pose.position.z, 0.0);
    EXPECT_EQ(ros_pose_f2tt.pose.orientation.x, 0.0);
    EXPECT_EQ(ros_pose_f2tt.pose.orientation.y, 0.0);

    std::cout << "Testing Pose<double, 2, false, true> toRos Pose" << std::endl;
    Pose<double, 2, false, true> adx_posed2ft;
    adx_posed2ft.position = { x, y };
    adx_posed2ft.yaw() = yaw;
    geometry_msgs::msg::PoseStamped ros_pose_d2ft;
    ros_pose_d2ft.pose.position.z = z;
    ros_pose_d2ft = adx_posed2ft.PoseHeader::toRos();
    compare_position<double, 2>(ros_pose_d2ft.pose.position, adx_posed2ft.position);
    compare_yaw<double, double>(yaw_from_quaternion(ros_pose_d2ft.pose.orientation),
                                adx_posed2ft.yaw());
    EXPECT_EQ(ros_pose_d2ft.pose.position.z, 0.0);
    EXPECT_EQ(ros_pose_d2ft.pose.orientation.x, 0.0);
    EXPECT_EQ(ros_pose_d2ft.pose.orientation.y, 0.0);
    std::cout << "Testing Pose<float, 2, false, true> toRos Pose" << std::endl;
    Pose<float, 2, false, true> adx_posef2ft;
    adx_posef2ft.position = { x, y };
    adx_posef2ft.yaw() = yaw;
    geometry_msgs::msg::PoseStamped ros_pose_f2ft;
    ros_pose_f2ft.pose.position.z = z;
    ros_pose_f2ft = adx_posef2ft.PoseHeader::toRos();
    compare_position<float, 2>(ros_pose_f2ft.pose.position, adx_posef2ft.position);
    compare_yaw<double, float>(yaw_from_quaternion(ros_pose_f2ft.pose.orientation),
                               adx_posef2ft.yaw());
    EXPECT_EQ(ros_pose_f2ft.pose.position.z, 0.0);
    EXPECT_EQ(ros_pose_f2ft.pose.orientation.x, 0.0);
    EXPECT_EQ(ros_pose_f2ft.pose.orientation.y, 0.0);

    std::cout << "Testing Pose<double, 2, true, false> toRos Pose" << std::endl;
    Pose<double, 2, true, false> adx_posed2tf;
    adx_posed2tf.position = { x, y };
    adx_posed2tf.yaw() = yaw;
    geometry_msgs::msg::PoseStamped ros_pose_d2tf;
    ros_pose_d2tf.pose.position.z = z;
    ros_pose_d2tf = adx_posed2tf.PoseHeader::toRos();
    compare_header(ros_pose_d2tf.header, static_cast<Header>(adx_posed2tf));
    compare_position<double, 2>(ros_pose_d2tf.pose.position, adx_posed2tf.position);
    compare_yaw<double, double>(yaw_from_quaternion(ros_pose_d2tf.pose.orientation),
                                adx_posed2tf.yaw());
    EXPECT_EQ(ros_pose_d2tf.pose.position.z, 0.0);
    EXPECT_EQ(ros_pose_d2tf.pose.orientation.x, 0.0);
    EXPECT_EQ(ros_pose_d2tf.pose.orientation.y, 0.0);
    std::cout << "Testing Pose<float, 2, true, false> toRos Pose" << std::endl;
    Pose<float, 2, true, false> adx_posef2tf;
    adx_posef2tf.position = { x, y };
    adx_posef2tf.yaw() = yaw;
    geometry_msgs::msg::PoseStamped ros_pose_f2tf;
    ros_pose_f2tf.pose.position.z = z;
    ros_pose_f2tf = adx_posef2tf.PoseHeader::toRos();
    compare_header(ros_pose_f2tf.header, static_cast<Header>(adx_posef2tf));
    compare_position<float, 2>(ros_pose_f2tf.pose.position, adx_posef2tf.position);
    compare_yaw<double, float>(yaw_from_quaternion(ros_pose_f2tf.pose.orientation),
                               adx_posef2tf.yaw());
    EXPECT_EQ(ros_pose_f2tf.pose.position.z, 0.0);
    EXPECT_EQ(ros_pose_f2tf.pose.orientation.x, 0.0);
    EXPECT_EQ(ros_pose_f2tf.pose.orientation.y, 0.0);

    std::cout << "Testing Pose<double, 2, false, false> toRos Pose" << std::endl;
    Pose<double, 2, false, false> adx_posed2ff;
    adx_posed2ff.position = { x, y };
    adx_posed2ff.yaw() = yaw;
    geometry_msgs::msg::PoseStamped ros_pose_d2ff;
    ros_pose_d2ff.pose.position.z = z;
    ros_pose_d2ff = adx_posed2ff.PoseHeader::toRos();
    compare_position<double, 2>(ros_pose_d2ff.pose.position, adx_posed2ff.position);
    compare_yaw<double, double>(yaw_from_quaternion(ros_pose_d2ff.pose.orientation),
                                adx_posed2ff.yaw());
    EXPECT_EQ(ros_pose_d2ff.pose.position.z, 0.0);
    EXPECT_EQ(ros_pose_d2ff.pose.orientation.x, 0.0);
    EXPECT_EQ(ros_pose_d2ff.pose.orientation.y, 0.0);
    std::cout << "Testing Pose<float, 2, false, false> toRos Pose" << std::endl;
    Pose<float, 2, false, false> adx_posef2ff;
    adx_posef2ff.position = { x, y };
    adx_posef2ff.yaw() = yaw;
    geometry_msgs::msg::PoseStamped ros_pose_f2ff;
    ros_pose_f2ff.pose.position.z = z;
    ros_pose_f2ff = adx_posef2ff.PoseHeader::toRos();
    compare_position<float, 2>(ros_pose_f2ff.pose.position, adx_posef2ff.position);
    compare_yaw<double, float>(yaw_from_quaternion(ros_pose_f2ff.pose.orientation),
                               adx_posef2ff.yaw());
    EXPECT_EQ(ros_pose_f2ff.pose.position.z, 0.0);
    EXPECT_EQ(ros_pose_f2ff.pose.orientation.x, 0.0);
    EXPECT_EQ(ros_pose_f2ff.pose.orientation.y, 0.0);
}

TEST_F(PoseTest, FromRosPoseWithCovariance)
{
    const double x = 1.0, y = 2.0, z = 3.0, roll = 0.1, pitch = 0.2, yaw = 0.3;

    geometry_msgs::msg::PoseWithCovariance ros_pose_cov;
    ros_pose_cov.pose.position.x = x;
    ros_pose_cov.pose.position.y = y;
    ros_pose_cov.pose.position.z = z;

    tf2::Quaternion tf2_quaternion;
    tf2_quaternion.setRPY(roll, pitch, yaw);

    ros_pose_cov.pose.orientation.w = tf2_quaternion.w();
    ros_pose_cov.pose.orientation.x = tf2_quaternion.x();
    ros_pose_cov.pose.orientation.y = tf2_quaternion.y();
    ros_pose_cov.pose.orientation.z = tf2_quaternion.z();

    for (int i = 0; i < 36; ++i) {
        ros_pose_cov.covariance[i] = static_cast<double>(i);
    }

    std::cout << "Testing Pose<double, 3, true, true> PoseWithCovariance constructor" << std::endl;
    Pose<double, 3, true, true> adx_posed3tt_constructor(ros_pose_cov);
    compare_position<double, 3>(ros_pose_cov.pose.position, adx_posed3tt_constructor.position);
    compare_orientation<double>(ros_pose_cov.pose.orientation,
                                adx_posed3tt_constructor.orientation);
    compare_covariance<double>(ros_pose_cov.covariance, adx_posed3tt_constructor.pose_covariance);
    std::cout << "Testing Pose<double, 3, true, true> PoseWithCovariance assignment" << std::endl;
    Pose<double, 3, true, true> adx_posed3tt_assignment;
    adx_posed3tt_assignment = ros_pose_cov;
    compare_position<double, 3>(ros_pose_cov.pose.position, adx_posed3tt_assignment.position);
    compare_orientation<double>(ros_pose_cov.pose.orientation, adx_posed3tt_assignment.orientation);
    compare_covariance<double>(ros_pose_cov.covariance, adx_posed3tt_assignment.pose_covariance);
    std::cout << "Testing Pose<float, 3, true, true> PoseWithCovariance constructor" << std::endl;
    Pose<float, 3, true, true> adx_posef3tt_constructor(ros_pose_cov);
    compare_position<float, 3>(ros_pose_cov.pose.position, adx_posef3tt_constructor.position);
    compare_orientation<float>(ros_pose_cov.pose.orientation, adx_posef3tt_constructor.orientation);
    compare_covariance<float>(ros_pose_cov.covariance, adx_posef3tt_constructor.pose_covariance);
    std::cout << "Testing Pose<float, 3, true, true> PoseWithCovariance assignment" << std::endl;
    Pose<float, 3, true, true> adx_posef3tt_assignment;
    adx_posef3tt_assignment = ros_pose_cov;
    compare_position<float, 3>(ros_pose_cov.pose.position, adx_posef3tt_assignment.position);
    compare_orientation<float>(ros_pose_cov.pose.orientation, adx_posef3tt_assignment.orientation);
    compare_covariance<float>(ros_pose_cov.covariance, adx_posef3tt_assignment.pose_covariance);

    std::cout << "Testing Pose<double, 3, false, true> PoseWithCovariance constructor" << std::endl;
    Pose<double, 3, false, true> adx_posed3ft_constructor(ros_pose_cov);
    compare_position<double, 3>(ros_pose_cov.pose.position, adx_posed3ft_constructor.position);
    compare_orientation<double>(ros_pose_cov.pose.orientation,
                                adx_posed3ft_constructor.orientation);
    compare_covariance<double>(ros_pose_cov.covariance, adx_posed3ft_constructor.pose_covariance);
    std::cout << "Testing Pose<double, 3, false, true> PoseWithCovariance assignment" << std::endl;
    Pose<double, 3, false, true> adx_posed3ft_assignment;
    adx_posed3ft_assignment = ros_pose_cov;
    compare_position<double, 3>(ros_pose_cov.pose.position, adx_posed3ft_assignment.position);
    compare_orientation<double>(ros_pose_cov.pose.orientation, adx_posed3ft_assignment.orientation);
    compare_covariance<double>(ros_pose_cov.covariance, adx_posed3ft_assignment.pose_covariance);
    std::cout << "Testing Pose<float, 3, false, true> PoseWithCovariance constructor" << std::endl;
    Pose<float, 3, false, true> adx_posef3ft_constructor(ros_pose_cov);
    compare_position<float, 3>(ros_pose_cov.pose.position, adx_posef3ft_constructor.position);
    compare_orientation<float>(ros_pose_cov.pose.orientation, adx_posef3ft_constructor.orientation);
    compare_covariance<float>(ros_pose_cov.covariance, adx_posef3ft_constructor.pose_covariance);
    std::cout << "Testing Pose<float, 3, false, true> PoseWithCovariance assignment" << std::endl;
    Pose<float, 3, false, true> adx_posef3ft_assignment;
    adx_posef3ft_assignment = ros_pose_cov;
    compare_position<float, 3>(ros_pose_cov.pose.position, adx_posef3ft_assignment.position);
    compare_orientation<float>(ros_pose_cov.pose.orientation, adx_posef3ft_assignment.orientation);
    compare_covariance<float>(ros_pose_cov.covariance, adx_posef3ft_assignment.pose_covariance);

    std::cout << "Testing Pose<double, 3, true, false> PoseWithCovariance constructor" << std::endl;
    Pose<double, 3, true, false> adx_posed3tf_constructor(ros_pose_cov);
    compare_position<double, 3>(ros_pose_cov.pose.position, adx_posed3tf_constructor.position);
    compare_orientation<double>(ros_pose_cov.pose.orientation,
                                adx_posed3tf_constructor.orientation);
    std::cout << "Testing Pose<double, 3, true, false> PoseWithCovariance assignment" << std::endl;
    Pose<double, 3, true, false> adx_posed3tf_assignment;
    adx_posed3tf_assignment = ros_pose_cov;
    compare_position<double, 3>(ros_pose_cov.pose.position, adx_posed3tf_assignment.position);
    compare_orientation<double>(ros_pose_cov.pose.orientation, adx_posed3tf_assignment.orientation);
    std::cout << "Testing Pose<float, 3, true, false> PoseWithCovariance constructor" << std::endl;
    Pose<float, 3, true, false> adx_posef3tf_constructor(ros_pose_cov);
    compare_position<float, 3>(ros_pose_cov.pose.position, adx_posef3tf_constructor.position);
    compare_orientation<float>(ros_pose_cov.pose.orientation, adx_posef3tf_constructor.orientation);
    std::cout << "Testing Pose<float, 3, true, false> PoseWithCovariance assignment" << std::endl;
    Pose<float, 3, true, false> adx_posef3tf_assignment;
    adx_posef3tf_assignment = ros_pose_cov;
    compare_position<float, 3>(ros_pose_cov.pose.position, adx_posef3tf_assignment.position);
    compare_orientation<float>(ros_pose_cov.pose.orientation, adx_posef3tf_assignment.orientation);

    std::cout << "Testing Pose<double, 3, false, false> PoseWithCovariance constructor"
              << std::endl;
    Pose<double, 3, false, false> adx_posed3ff_constructor(ros_pose_cov);
    compare_position<double, 3>(ros_pose_cov.pose.position, adx_posed3ff_constructor.position);
    compare_orientation<double>(ros_pose_cov.pose.orientation,
                                adx_posed3ff_constructor.orientation);
    std::cout << "Testing Pose<double, 3, false, false> PoseWithCovariance assignment" << std::endl;
    Pose<double, 3, false, false> adx_posed3ff_assignment;
    adx_posed3ff_assignment = ros_pose_cov;
    compare_position<double, 3>(ros_pose_cov.pose.position, adx_posed3ff_assignment.position);
    compare_orientation<double>(ros_pose_cov.pose.orientation, adx_posed3ff_assignment.orientation);
    std::cout << "Testing Pose<float, 3, false, false> PoseWithCovariance constructor" << std::endl;
    Pose<float, 3, false, false> adx_posef3ff_constructor(ros_pose_cov);
    compare_position<float, 3>(ros_pose_cov.pose.position, adx_posef3ff_constructor.position);
    compare_orientation<float>(ros_pose_cov.pose.orientation, adx_posef3ff_constructor.orientation);
    std::cout << "Testing Pose<float, 3, false, false> PoseWithCovariance assignment" << std::endl;
    Pose<float, 3, false, false> adx_posef3ff_assignment;
    adx_posef3ff_assignment = ros_pose_cov;
    compare_position<float, 3>(ros_pose_cov.pose.position, adx_posef3ff_assignment.position);
    compare_orientation<float>(ros_pose_cov.pose.orientation, adx_posef3ff_assignment.orientation);

    geometry_msgs::msg::PoseWithCovariance::SharedPtr ros_pose_cov_ptr =
      std::make_shared<geometry_msgs::msg::PoseWithCovariance>();
    ros_pose_cov_ptr->pose.position.x = x;
    ros_pose_cov_ptr->pose.position.y = y;
    ros_pose_cov_ptr->pose.position.z = z;

    ros_pose_cov_ptr->pose.orientation.w = tf2_quaternion.w();
    ros_pose_cov_ptr->pose.orientation.x = tf2_quaternion.x();
    ros_pose_cov_ptr->pose.orientation.y = tf2_quaternion.y();
    ros_pose_cov_ptr->pose.orientation.z = tf2_quaternion.z();

    for (int i = 0; i < 36; ++i) {
        ros_pose_cov_ptr->covariance[i] = static_cast<double>(i);
    }

    std::cout << "Testing Pose<double, 3, true, true> PoseWithCovariance pointer constructor"
              << std::endl;
    Pose<double, 3, true, true> adx_posed3tt_ptr_constructor(ros_pose_cov_ptr);
    compare_position<double, 3>(ros_pose_cov_ptr->pose.position,
                                adx_posed3tt_ptr_constructor.position);
    compare_orientation<double>(ros_pose_cov_ptr->pose.orientation,
                                adx_posed3tt_ptr_constructor.orientation);
    compare_covariance<double>(ros_pose_cov_ptr->covariance,
                               adx_posed3tt_ptr_constructor.pose_covariance);
    std::cout << "Testing Pose<double, 3, true, true> PoseWithCovariance pointer assignment"
              << std::endl;
    Pose<double, 3, true, true> adx_posed3tt_ptr_assignment;
    adx_posed3tt_ptr_assignment = ros_pose_cov;
    compare_position<double, 3>(ros_pose_cov_ptr->pose.position,
                                adx_posed3tt_ptr_assignment.position);
    compare_orientation<double>(ros_pose_cov_ptr->pose.orientation,
                                adx_posed3tt_ptr_assignment.orientation);
    compare_covariance<double>(ros_pose_cov_ptr->covariance,
                               adx_posed3tt_ptr_assignment.pose_covariance);
    std::cout << "Testing Pose<float, 3, true, true> PoseWithCovariance pointer constructor"
              << std::endl;
    Pose<float, 3, true, true> adx_posef3tt_ptr_constructor(ros_pose_cov_ptr);
    compare_position<float, 3>(ros_pose_cov_ptr->pose.position,
                               adx_posef3tt_ptr_constructor.position);
    compare_orientation<float>(ros_pose_cov_ptr->pose.orientation,
                               adx_posef3tt_ptr_constructor.orientation);
    compare_covariance<float>(ros_pose_cov_ptr->covariance,
                              adx_posef3tt_ptr_constructor.pose_covariance);
    std::cout << "Testing Pose<float, 3, true, true> PoseWithCovariance pointer assignment"
              << std::endl;
    Pose<float, 3, true, true> adx_posef3tt_ptr_assignment;
    adx_posef3tt_ptr_assignment = ros_pose_cov;
    compare_position<float, 3>(ros_pose_cov_ptr->pose.position,
                               adx_posef3tt_ptr_assignment.position);
    compare_orientation<float>(ros_pose_cov_ptr->pose.orientation,
                               adx_posef3tt_ptr_assignment.orientation);
    compare_covariance<float>(ros_pose_cov_ptr->covariance,
                              adx_posef3tt_ptr_assignment.pose_covariance);

    std::cout << "Testing Pose<double, 3, false, true> PoseWithCovariance pointer constructor"
              << std::endl;
    Pose<double, 3, false, true> adx_posed3ft_ptr_constructor(ros_pose_cov);
    compare_position<double, 3>(ros_pose_cov.pose.position, adx_posed3ft_ptr_constructor.position);
    compare_orientation<double>(ros_pose_cov.pose.orientation,
                                adx_posed3ft_ptr_constructor.orientation);
    compare_covariance<double>(ros_pose_cov.covariance,
                               adx_posed3ft_ptr_constructor.pose_covariance);
    std::cout << "Testing Pose<double, 3, false, true> PoseWithCovariance pointer assignment"
              << std::endl;
    Pose<double, 3, false, true> adx_posed3ft_ptr_assignment;
    adx_posed3ft_ptr_assignment = ros_pose_cov;
    compare_position<double, 3>(ros_pose_cov.pose.position, adx_posed3ft_ptr_assignment.position);
    compare_orientation<double>(ros_pose_cov.pose.orientation,
                                adx_posed3ft_ptr_assignment.orientation);
    compare_covariance<double>(ros_pose_cov.covariance,
                               adx_posed3ft_ptr_assignment.pose_covariance);
    std::cout << "Testing Pose<float, 3, false, true> PoseWithCovariance pointer constructor"
              << std::endl;
    Pose<float, 3, false, true> adx_posef3ft_ptr_constructor(ros_pose_cov);
    compare_position<float, 3>(ros_pose_cov.pose.position, adx_posef3ft_ptr_constructor.position);
    compare_orientation<float>(ros_pose_cov.pose.orientation,
                               adx_posef3ft_ptr_constructor.orientation);
    compare_covariance<float>(ros_pose_cov.covariance,
                              adx_posef3ft_ptr_constructor.pose_covariance);
    std::cout << "Testing Pose<float, 3, false, true> PoseWithCovariance pointer assignment"
              << std::endl;
    Pose<float, 3, false, true> adx_posef3ft_ptr_assignment;
    adx_posef3ft_ptr_assignment = ros_pose_cov;
    compare_position<float, 3>(ros_pose_cov.pose.position, adx_posef3ft_ptr_assignment.position);
    compare_orientation<float>(ros_pose_cov.pose.orientation,
                               adx_posef3ft_ptr_assignment.orientation);
    compare_covariance<float>(ros_pose_cov.covariance, adx_posef3ft_ptr_assignment.pose_covariance);

    std::cout << "Testing Pose<double, 3, true, false> PoseWithCovariance pointer constructor"
              << std::endl;
    Pose<double, 3, true, false> adx_posed3tf_ptr_constructor(ros_pose_cov);
    compare_position<double, 3>(ros_pose_cov.pose.position, adx_posed3tf_ptr_constructor.position);
    compare_orientation<double>(ros_pose_cov.pose.orientation,
                                adx_posed3tf_ptr_constructor.orientation);
    std::cout << "Testing Pose<double, 3, true, false> PoseWithCovariance pointer assignment"
              << std::endl;
    Pose<double, 3, true, false> adx_posed3tf_ptr_assignment;
    adx_posed3tf_ptr_assignment = ros_pose_cov;
    compare_position<double, 3>(ros_pose_cov.pose.position, adx_posed3tf_ptr_assignment.position);
    compare_orientation<double>(ros_pose_cov.pose.orientation,
                                adx_posed3tf_ptr_assignment.orientation);
    std::cout << "Testing Pose<float, 3, true, false> PoseWithCovariance pointer constructor"
              << std::endl;
    Pose<float, 3, true, false> adx_posef3tf_ptr_constructor(ros_pose_cov);
    compare_position<float, 3>(ros_pose_cov.pose.position, adx_posef3tf_ptr_constructor.position);
    compare_orientation<float>(ros_pose_cov.pose.orientation,
                               adx_posef3tf_ptr_constructor.orientation);
    std::cout << "Testing Pose<float, 3, true, false> PoseWithCovariance pointer assignment"
              << std::endl;
    Pose<float, 3, true, false> adx_posef3tf_ptr_assignment;
    adx_posef3tf_ptr_assignment = ros_pose_cov;
    compare_position<float, 3>(ros_pose_cov.pose.position, adx_posef3tf_ptr_assignment.position);
    compare_orientation<float>(ros_pose_cov.pose.orientation,
                               adx_posef3tf_ptr_assignment.orientation);

    std::cout << "Testing Pose<double, 3, false, false> PoseWithCovariance pointer constructor"
              << std::endl;
    Pose<double, 3, false, false> adx_posed3ff_ptr_constructor(ros_pose_cov);
    compare_position<double, 3>(ros_pose_cov.pose.position, adx_posed3ff_ptr_constructor.position);
    compare_orientation<double>(ros_pose_cov.pose.orientation,
                                adx_posed3ff_ptr_constructor.orientation);
    std::cout << "Testing Pose<double, 3, false, false> PoseWithCovariance pointer assignment"
              << std::endl;
    Pose<double, 3, false, false> adx_posed3ff_ptr_assignment;
    adx_posed3ff_ptr_assignment = ros_pose_cov;
    compare_position<double, 3>(ros_pose_cov.pose.position, adx_posed3ff_ptr_assignment.position);
    compare_orientation<double>(ros_pose_cov.pose.orientation,
                                adx_posed3ff_ptr_assignment.orientation);
    std::cout << "Testing Pose<float, 3, false, false> PoseWithCovariance pointer constructor"
              << std::endl;
    Pose<float, 3, false, false> adx_posef3ff_ptr_constructor(ros_pose_cov);
    compare_position<float, 3>(ros_pose_cov.pose.position, adx_posef3ff_ptr_constructor.position);
    compare_orientation<float>(ros_pose_cov.pose.orientation,
                               adx_posef3ff_ptr_constructor.orientation);
    std::cout << "Testing Pose<float, 3, false, false> PoseWithCovariance pointer assignment"
              << std::endl;
    Pose<float, 3, false, false> adx_posef3ff_ptr_assignment;
    adx_posef3ff_ptr_assignment = ros_pose_cov;
    compare_position<float, 3>(ros_pose_cov.pose.position, adx_posef3ff_ptr_assignment.position);
    compare_orientation<float>(ros_pose_cov.pose.orientation,
                               adx_posef3ff_ptr_assignment.orientation);
}

TEST_F(PoseTest, ToRosPoseWithCovariance)
{
    const double x = 1.0, y = 2.0, z = 3.0, roll = 0.1, pitch = 0.2, yaw = 0.3;

    std::cout << "Testing Pose<double, 3, true, true> toRos PoseWithCovariance" << std::endl;
    Pose<double, 3, true, true> adx_posed3tt;
    adx_posed3tt.position = { x, y, z };
    adx_posed3tt.orientation.setRPY(roll, pitch, yaw);
    for (int i = 0; i < 36; ++i) {
        adx_posed3tt.pose_covariance(i % 6, i / 6) = static_cast<double>(i);
    }
    geometry_msgs::msg::PoseWithCovariance ros_pose_cov_d3tt = adx_posed3tt.PoseCovariance::toRos();
    compare_position<double, 3>(ros_pose_cov_d3tt.pose.position, adx_posed3tt.position);
    compare_orientation<double>(ros_pose_cov_d3tt.pose.orientation, adx_posed3tt.orientation);
    compare_covariance<double>(ros_pose_cov_d3tt.covariance, adx_posed3tt.pose_covariance);

    std::cout << "Testing Pose<float, 3, true, true> toRos PoseWithCovariance" << std::endl;
    Pose<float, 3, true, true> adx_posef3tt;
    adx_posef3tt.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    adx_posef3tt.orientation.setRPY(roll, pitch, yaw);
    for (int i = 0; i < 36; ++i) {
        adx_posef3tt.pose_covariance(i % 6, i / 6) = static_cast<float>(i);
    }
    geometry_msgs::msg::PoseWithCovariance ros_pose_cov_f3tt = adx_posef3tt.PoseCovariance::toRos();
    compare_position<float, 3>(ros_pose_cov_f3tt.pose.position, adx_posef3tt.position);
    compare_orientation<float>(ros_pose_cov_f3tt.pose.orientation, adx_posef3tt.orientation);
    compare_covariance<float>(ros_pose_cov_f3tt.covariance, adx_posef3tt.pose_covariance);

    std::cout << "Testing Pose<double, 3, false, true> toRos PoseWithCovariance" << std::endl;
    Pose<double, 3, false, true> adx_posed3ft;
    adx_posed3ft.position = { x, y, z };
    adx_posed3ft.orientation.setRPY(roll, pitch, yaw);
    for (int i = 0; i < 36; ++i) {
        adx_posed3ft.pose_covariance(i % 6, i / 6) = static_cast<double>(i);
    }
    geometry_msgs::msg::PoseWithCovariance ros_pose_cov_d3ft = adx_posed3ft.PoseCovariance::toRos();
    compare_position<double, 3>(ros_pose_cov_d3ft.pose.position, adx_posed3ft.position);
    compare_orientation<double>(ros_pose_cov_d3ft.pose.orientation, adx_posed3ft.orientation);
    compare_covariance<double>(ros_pose_cov_d3ft.covariance, adx_posed3ft.pose_covariance);

    std::cout << "Testing Pose<float, 3, false, true> toRos PoseWithCovariance" << std::endl;
    Pose<float, 3, false, true> adx_posef3ft;
    adx_posef3ft.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    adx_posef3ft.orientation.setRPY(roll, pitch, yaw);
    for (int i = 0; i < 36; ++i) {
        adx_posef3ft.pose_covariance(i % 6, i / 6) = static_cast<float>(i);
    }
    geometry_msgs::msg::PoseWithCovariance ros_pose_cov_f3ft = adx_posef3ft.PoseCovariance::toRos();
    compare_position<float, 3>(ros_pose_cov_f3ft.pose.position, adx_posef3ft.position);
    compare_orientation<float>(ros_pose_cov_f3ft.pose.orientation, adx_posef3ft.orientation);
    compare_covariance<float>(ros_pose_cov_f3ft.covariance, adx_posef3ft.pose_covariance);

    std::cout << "Testing Pose<double, 3, false, false> toRos PoseWithCovariance" << std::endl;
    Pose<double, 3, false, false> adx_posed3ff;
    adx_posed3ff.position = { x, y, z };
    geometry_msgs::msg::PoseWithCovariance ros_pose_cov_d3ff = adx_posed3ff.PoseCovariance::toRos();
    compare_position<double, 3>(ros_pose_cov_d3ff.pose.position, adx_posed3ff.position);
    compare_orientation<double>(ros_pose_cov_d3ff.pose.orientation, adx_posed3ff.orientation);

    std::cout << "Testing Pose<float, 3, false, false> toRos PoseWithCovariance" << std::endl;
    Pose<float, 3, false, false> adx_posef3ff;
    adx_posef3ff.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    geometry_msgs::msg::PoseWithCovariance ros_pose_cov_f3ff = adx_posef3ff.PoseCovariance::toRos();
    compare_position<float, 3>(ros_pose_cov_f3ff.pose.position, adx_posef3ff.position);
    compare_orientation<float>(ros_pose_cov_f3ff.pose.orientation, adx_posef3ff.orientation);
}

TEST_F(PoseTest, FromRosPoseWithCovarianceStamped)
{
    const double x = 1.0, y = 2.0, z = 3.0, roll = 0.1, pitch = 0.2, yaw = 0.3;

    geometry_msgs::msg::PoseWithCovarianceStamped ros_pose_cov_stamp;
    ros_pose_cov_stamp.header.stamp = mNode->get_clock()->now();
    ros_pose_cov_stamp.header.frame_id = "map";

    ros_pose_cov_stamp.pose.pose.position.x = x;
    ros_pose_cov_stamp.pose.pose.position.y = y;
    ros_pose_cov_stamp.pose.pose.position.z = z;

    tf2::Quaternion tf2_quaternion;
    tf2_quaternion.setRPY(roll, pitch, yaw);

    ros_pose_cov_stamp.pose.pose.orientation.w = tf2_quaternion.w();
    ros_pose_cov_stamp.pose.pose.orientation.x = tf2_quaternion.x();
    ros_pose_cov_stamp.pose.pose.orientation.y = tf2_quaternion.y();
    ros_pose_cov_stamp.pose.pose.orientation.z = tf2_quaternion.z();

    for (int i = 0; i < 36; ++i) {
        ros_pose_cov_stamp.pose.covariance[i] = static_cast<double>(i);
    }

    std::cout << "Testing Pose<double, 3, true, true> PoseWithCovarianceStamped constructor"
              << std::endl;
    Pose<double, 3, true, true> adx_posed3tt_constructor(ros_pose_cov_stamp);
    compare_header(ros_pose_cov_stamp.header, static_cast<Header>(adx_posed3tt_constructor));
    compare_position<double, 3>(ros_pose_cov_stamp.pose.pose.position,
                                adx_posed3tt_constructor.position);
    compare_orientation<double>(ros_pose_cov_stamp.pose.pose.orientation,
                                adx_posed3tt_constructor.orientation);
    compare_covariance<double>(ros_pose_cov_stamp.pose.covariance,
                               adx_posed3tt_constructor.pose_covariance);
    std::cout << "Testing Pose<double, 3, true, true> PoseWithCovarianceStamped assignment"
              << std::endl;
    Pose<double, 3, true, true> adx_posed3tt_assignment;
    adx_posed3tt_assignment = ros_pose_cov_stamp;
    compare_header(ros_pose_cov_stamp.header, static_cast<Header>(adx_posed3tt_assignment));
    compare_position<double, 3>(ros_pose_cov_stamp.pose.pose.position,
                                adx_posed3tt_assignment.position);
    compare_orientation<double>(ros_pose_cov_stamp.pose.pose.orientation,
                                adx_posed3tt_assignment.orientation);
    compare_covariance<double>(ros_pose_cov_stamp.pose.covariance,
                               adx_posed3tt_assignment.pose_covariance);
    std::cout << "Testing Pose<float, 3, true, true> PoseWithCovarianceStamped constructor"
              << std::endl;
    Pose<float, 3, true, true> adx_posef3tt_constructor(ros_pose_cov_stamp);
    compare_header(ros_pose_cov_stamp.header, static_cast<Header>(adx_posef3tt_constructor));
    compare_position<float, 3>(ros_pose_cov_stamp.pose.pose.position,
                               adx_posef3tt_constructor.position);
    compare_orientation<float>(ros_pose_cov_stamp.pose.pose.orientation,
                               adx_posef3tt_constructor.orientation);
    compare_covariance<float>(ros_pose_cov_stamp.pose.covariance,
                              adx_posef3tt_constructor.pose_covariance);
    std::cout << "Testing Pose<float, 3, true, true> PoseWithCovarianceStamped assignment"
              << std::endl;
    Pose<float, 3, true, true> adx_posef3tt_assignment;
    adx_posef3tt_assignment = ros_pose_cov_stamp;
    compare_header(ros_pose_cov_stamp.header, static_cast<Header>(adx_posef3tt_assignment));
    compare_position<float, 3>(ros_pose_cov_stamp.pose.pose.position,
                               adx_posef3tt_assignment.position);
    compare_orientation<float>(ros_pose_cov_stamp.pose.pose.orientation,
                               adx_posef3tt_assignment.orientation);
    compare_covariance<float>(ros_pose_cov_stamp.pose.covariance,
                              adx_posef3tt_assignment.pose_covariance);

    std::cout << "Testing Pose<double, 3, false, true> PoseWithCovarianceStamped constructor"
              << std::endl;
    Pose<double, 3, false, true> adx_posed3ft_constructor(ros_pose_cov_stamp);
    compare_position<double, 3>(ros_pose_cov_stamp.pose.pose.position,
                                adx_posed3ft_constructor.position);
    compare_orientation<double>(ros_pose_cov_stamp.pose.pose.orientation,
                                adx_posed3ft_constructor.orientation);
    compare_covariance<double>(ros_pose_cov_stamp.pose.covariance,
                               adx_posed3ft_constructor.pose_covariance);
    std::cout << "Testing Pose<double, 3, false, true> PoseWithCovarianceStamped assignment"
              << std::endl;
    Pose<double, 3, false, true> adx_posed3ft_assignment;
    adx_posed3ft_assignment = ros_pose_cov_stamp;
    compare_position<double, 3>(ros_pose_cov_stamp.pose.pose.position,
                                adx_posed3ft_assignment.position);
    compare_orientation<double>(ros_pose_cov_stamp.pose.pose.orientation,
                                adx_posed3ft_assignment.orientation);
    compare_covariance<double>(ros_pose_cov_stamp.pose.covariance,
                               adx_posed3ft_assignment.pose_covariance);
    std::cout << "Testing Pose<float, 3, false, true> PoseWithCovarianceStamped constructor"
              << std::endl;
    Pose<float, 3, false, true> adx_posef3ft_constructor(ros_pose_cov_stamp);
    compare_position<float, 3>(ros_pose_cov_stamp.pose.pose.position,
                               adx_posef3ft_constructor.position);
    compare_orientation<float>(ros_pose_cov_stamp.pose.pose.orientation,
                               adx_posef3ft_constructor.orientation);
    compare_covariance<float>(ros_pose_cov_stamp.pose.covariance,
                              adx_posef3ft_constructor.pose_covariance);
    std::cout << "Testing Pose<float, 3, false, true> PoseWithCovarianceStamped assignment"
              << std::endl;
    Pose<float, 3, false, true> adx_posef3ft_assignment;
    adx_posef3ft_assignment = ros_pose_cov_stamp;
    compare_position<float, 3>(ros_pose_cov_stamp.pose.pose.position,
                               adx_posef3ft_assignment.position);
    compare_orientation<float>(ros_pose_cov_stamp.pose.pose.orientation,
                               adx_posef3ft_assignment.orientation);
    compare_covariance<float>(ros_pose_cov_stamp.pose.covariance,
                              adx_posef3ft_assignment.pose_covariance);

    std::cout << "Testing Pose<double, 3, true, false> PoseWithCovarianceStamped constructor"
              << std::endl;
    Pose<double, 3, true, false> adx_posed3tf_constructor(ros_pose_cov_stamp);
    compare_header(ros_pose_cov_stamp.header, static_cast<Header>(adx_posed3tf_constructor));
    compare_position<double, 3>(ros_pose_cov_stamp.pose.pose.position,
                                adx_posed3tf_constructor.position);
    compare_orientation<double>(ros_pose_cov_stamp.pose.pose.orientation,
                                adx_posed3tf_constructor.orientation);
    std::cout << "Testing Pose<double, 3, true, false> PoseWithCovarianceStamped assignment"
              << std::endl;
    Pose<double, 3, true, false> adx_posed3tf_assignment;
    adx_posed3tf_assignment = ros_pose_cov_stamp;
    compare_header(ros_pose_cov_stamp.header, static_cast<Header>(adx_posed3tf_assignment));
    compare_position<double, 3>(ros_pose_cov_stamp.pose.pose.position,
                                adx_posed3tf_assignment.position);
    compare_orientation<double>(ros_pose_cov_stamp.pose.pose.orientation,
                                adx_posed3tf_assignment.orientation);
    std::cout << "Testing Pose<float, 3, true, false> PoseWithCovarianceStamped constructor"
              << std::endl;
    Pose<float, 3, true, false> adx_posef3tf_constructor(ros_pose_cov_stamp);
    compare_header(ros_pose_cov_stamp.header, static_cast<Header>(adx_posef3tf_constructor));
    compare_position<float, 3>(ros_pose_cov_stamp.pose.pose.position,
                               adx_posef3tf_constructor.position);
    compare_orientation<float>(ros_pose_cov_stamp.pose.pose.orientation,
                               adx_posef3tf_constructor.orientation);
    std::cout << "Testing Pose<float, 3, true, false> PoseWithCovarianceStamped assignment"
              << std::endl;
    Pose<float, 3, true, false> adx_posef3tf_assignment;
    adx_posef3tf_assignment = ros_pose_cov_stamp;
    compare_header(ros_pose_cov_stamp.header, static_cast<Header>(adx_posef3tf_assignment));
    compare_position<float, 3>(ros_pose_cov_stamp.pose.pose.position,
                               adx_posef3tf_assignment.position);
    compare_orientation<float>(ros_pose_cov_stamp.pose.pose.orientation,
                               adx_posef3tf_assignment.orientation);

    std::cout << "Testing Pose<double, 3, false, false> PoseWithCovarianceStamped constructor"
              << std::endl;
    Pose<double, 3, false, false> adx_posed3ff_constructor(ros_pose_cov_stamp);
    compare_position<double, 3>(ros_pose_cov_stamp.pose.pose.position,
                                adx_posed3ff_constructor.position);
    compare_orientation<double>(ros_pose_cov_stamp.pose.pose.orientation,
                                adx_posed3ff_constructor.orientation);
    std::cout << "Testing Pose<double, 3, false, false> PoseWithCovarianceStamped assignment"
              << std::endl;
    Pose<double, 3, false, false> adx_posed3ff_assignment;
    adx_posed3ff_assignment = ros_pose_cov_stamp;
    compare_position<double, 3>(ros_pose_cov_stamp.pose.pose.position,
                                adx_posed3ff_assignment.position);
    compare_orientation<double>(ros_pose_cov_stamp.pose.pose.orientation,
                                adx_posed3ff_assignment.orientation);
    std::cout << "Testing Pose<float, 3, false, false> PoseWithCovarianceStamped constructor"
              << std::endl;
    Pose<float, 3, false, false> adx_posef3ff_constructor(ros_pose_cov_stamp);
    compare_position<float, 3>(ros_pose_cov_stamp.pose.pose.position,
                               adx_posef3ff_constructor.position);
    compare_orientation<float>(ros_pose_cov_stamp.pose.pose.orientation,
                               adx_posef3ff_constructor.orientation);
    std::cout << "Testing Pose<float, 3, false, false> PoseWithCovarianceStamped assignment"
              << std::endl;
    Pose<float, 3, false, false> adx_posef3ff_assignment;
    adx_posef3ff_assignment = ros_pose_cov_stamp;
    compare_position<float, 3>(ros_pose_cov_stamp.pose.pose.position,
                               adx_posef3ff_assignment.position);
    compare_orientation<float>(ros_pose_cov_stamp.pose.pose.orientation,
                               adx_posef3ff_assignment.orientation);

    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr ros_pose_cov_stamp_ptr =
      std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    ros_pose_cov_stamp_ptr->pose.pose.position.x = x;
    ros_pose_cov_stamp_ptr->pose.pose.position.y = y;
    ros_pose_cov_stamp_ptr->pose.pose.position.z = z;

    ros_pose_cov_stamp_ptr->pose.pose.orientation.w = tf2_quaternion.w();
    ros_pose_cov_stamp_ptr->pose.pose.orientation.x = tf2_quaternion.x();
    ros_pose_cov_stamp_ptr->pose.pose.orientation.y = tf2_quaternion.y();
    ros_pose_cov_stamp_ptr->pose.pose.orientation.z = tf2_quaternion.z();

    for (int i = 0; i < 36; ++i) {
        ros_pose_cov_stamp_ptr->pose.covariance[i] = static_cast<double>(i);
    }

    std::cout << "Testing Pose<double, 3, true, true> PoseWithCovarianceStamped pointer constructor"
              << std::endl;
    Pose<double, 3, true, true> adx_posed3tt_ptr_constructor(ros_pose_cov_stamp_ptr);
    compare_header(ros_pose_cov_stamp_ptr->header,
                   static_cast<Header>(adx_posed3tt_ptr_constructor));
    compare_position<double, 3>(ros_pose_cov_stamp_ptr->pose.pose.position,
                                adx_posed3tt_ptr_constructor.position);
    compare_orientation<double>(ros_pose_cov_stamp_ptr->pose.pose.orientation,
                                adx_posed3tt_ptr_constructor.orientation);
    compare_covariance<double>(ros_pose_cov_stamp_ptr->pose.covariance,
                               adx_posed3tt_ptr_constructor.pose_covariance);
    std::cout << "Testing Pose<double, 3, true, true> PoseWithCovarianceStamped pointer assignment"
              << std::endl;
    Pose<double, 3, true, true> adx_posed3tt_ptr_assignment;
    adx_posed3tt_ptr_assignment = ros_pose_cov_stamp_ptr;
    compare_header(ros_pose_cov_stamp_ptr->header,
                   static_cast<Header>(adx_posed3tt_ptr_assignment));
    compare_position<double, 3>(ros_pose_cov_stamp_ptr->pose.pose.position,
                                adx_posed3tt_ptr_assignment.position);
    compare_orientation<double>(ros_pose_cov_stamp_ptr->pose.pose.orientation,
                                adx_posed3tt_ptr_assignment.orientation);
    compare_covariance<double>(ros_pose_cov_stamp_ptr->pose.covariance,
                               adx_posed3tt_ptr_assignment.pose_covariance);
    std::cout << "Testing Pose<float, 3, true, true> PoseWithCovarianceStamped pointer constructor"
              << std::endl;
    Pose<float, 3, true, true> adx_posef3tt_ptr_constructor(ros_pose_cov_stamp_ptr);
    compare_header(ros_pose_cov_stamp_ptr->header,
                   static_cast<Header>(adx_posef3tt_ptr_constructor));
    compare_position<float, 3>(ros_pose_cov_stamp_ptr->pose.pose.position,
                               adx_posef3tt_ptr_constructor.position);
    compare_orientation<float>(ros_pose_cov_stamp_ptr->pose.pose.orientation,
                               adx_posef3tt_ptr_constructor.orientation);
    compare_covariance<float>(ros_pose_cov_stamp_ptr->pose.covariance,
                              adx_posef3tt_ptr_constructor.pose_covariance);
    std::cout << "Testing Pose<float, 3, true, true> PoseWithCovarianceStamped pointer assignment"
              << std::endl;
    Pose<float, 3, true, true> adx_posef3tt_ptr_assignment;
    adx_posef3tt_ptr_assignment = ros_pose_cov_stamp_ptr;
    compare_header(ros_pose_cov_stamp_ptr->header,
                   static_cast<Header>(adx_posef3tt_ptr_assignment));
    compare_position<float, 3>(ros_pose_cov_stamp_ptr->pose.pose.position,
                               adx_posef3tt_ptr_assignment.position);
    compare_orientation<float>(ros_pose_cov_stamp_ptr->pose.pose.orientation,
                               adx_posef3tt_ptr_assignment.orientation);
    compare_covariance<float>(ros_pose_cov_stamp_ptr->pose.covariance,
                              adx_posef3tt_ptr_assignment.pose_covariance);

    std::cout
      << "Testing Pose<double, 3, false, true> PoseWithCovarianceStamped pointer constructor"
      << std::endl;
    Pose<double, 3, false, true> adx_posed3ft_ptr_constructor(ros_pose_cov_stamp_ptr);
    compare_position<double, 3>(ros_pose_cov_stamp_ptr->pose.pose.position,
                                adx_posed3ft_ptr_constructor.position);
    compare_orientation<double>(ros_pose_cov_stamp_ptr->pose.pose.orientation,
                                adx_posed3ft_ptr_constructor.orientation);
    compare_covariance<double>(ros_pose_cov_stamp_ptr->pose.covariance,
                               adx_posed3ft_ptr_constructor.pose_covariance);
    std::cout << "Testing Pose<double, 3, false, true> PoseWithCovarianceStamped pointer assignment"
              << std::endl;
    Pose<double, 3, false, true> adx_posed3ft_ptr_assignment;
    adx_posed3ft_ptr_assignment = ros_pose_cov_stamp_ptr;
    compare_position<double, 3>(ros_pose_cov_stamp_ptr->pose.pose.position,
                                adx_posed3ft_ptr_assignment.position);
    compare_orientation<double>(ros_pose_cov_stamp_ptr->pose.pose.orientation,
                                adx_posed3ft_ptr_assignment.orientation);
    compare_covariance<double>(ros_pose_cov_stamp_ptr->pose.covariance,
                               adx_posed3ft_ptr_assignment.pose_covariance);
    std::cout << "Testing Pose<float, 3, false, true> PoseWithCovarianceStamped pointer constructor"
              << std::endl;
    Pose<float, 3, false, true> adx_posef3ft_ptr_constructor(ros_pose_cov_stamp_ptr);
    compare_position<float, 3>(ros_pose_cov_stamp_ptr->pose.pose.position,
                               adx_posef3ft_ptr_constructor.position);
    compare_orientation<float>(ros_pose_cov_stamp_ptr->pose.pose.orientation,
                               adx_posef3ft_ptr_constructor.orientation);
    compare_covariance<float>(ros_pose_cov_stamp_ptr->pose.covariance,
                              adx_posef3ft_ptr_constructor.pose_covariance);
    std::cout << "Testing Pose<float, 3, false, true> PoseWithCovarianceStamped pointer assignment"
              << std::endl;
    Pose<float, 3, false, true> adx_posef3ft_ptr_assignment;
    adx_posef3ft_ptr_assignment = ros_pose_cov_stamp_ptr;
    compare_position<float, 3>(ros_pose_cov_stamp_ptr->pose.pose.position,
                               adx_posef3ft_ptr_assignment.position);
    compare_orientation<float>(ros_pose_cov_stamp_ptr->pose.pose.orientation,
                               adx_posef3ft_ptr_assignment.orientation);
    compare_covariance<float>(ros_pose_cov_stamp_ptr->pose.covariance,
                              adx_posef3ft_ptr_assignment.pose_covariance);

    std::cout
      << "Testing Pose<double, 3, true, false> PoseWithCovarianceStamped pointer constructor"
      << std::endl;
    Pose<double, 3, true, false> adx_posed3tf_ptr_constructor(ros_pose_cov_stamp_ptr);
    compare_header(ros_pose_cov_stamp_ptr->header,
                   static_cast<Header>(adx_posed3tf_ptr_constructor));
    compare_position<double, 3>(ros_pose_cov_stamp_ptr->pose.pose.position,
                                adx_posed3tf_ptr_constructor.position);
    compare_orientation<double>(ros_pose_cov_stamp_ptr->pose.pose.orientation,
                                adx_posed3tf_ptr_constructor.orientation);
    std::cout << "Testing Pose<double, 3, true, false> PoseWithCovarianceStamped pointer assignment"
              << std::endl;
    Pose<double, 3, true, false> adx_posed3tf_ptr_assignment;
    adx_posed3tf_ptr_assignment = ros_pose_cov_stamp_ptr;
    compare_header(ros_pose_cov_stamp_ptr->header,
                   static_cast<Header>(adx_posed3tf_ptr_assignment));
    compare_position<double, 3>(ros_pose_cov_stamp_ptr->pose.pose.position,
                                adx_posed3tf_ptr_assignment.position);
    compare_orientation<double>(ros_pose_cov_stamp_ptr->pose.pose.orientation,
                                adx_posed3tf_ptr_assignment.orientation);
    std::cout << "Testing Pose<float, 3, true, false> PoseWithCovarianceStamped pointer constructor"
              << std::endl;
    Pose<float, 3, true, false> adx_posef3tf_ptr_constructor(ros_pose_cov_stamp_ptr);
    compare_header(ros_pose_cov_stamp_ptr->header,
                   static_cast<Header>(adx_posef3tf_ptr_constructor));
    compare_position<float, 3>(ros_pose_cov_stamp_ptr->pose.pose.position,
                               adx_posef3tf_ptr_constructor.position);
    compare_orientation<float>(ros_pose_cov_stamp_ptr->pose.pose.orientation,
                               adx_posef3tf_ptr_constructor.orientation);
    std::cout << "Testing Pose<float, 3, true, false> PoseWithCovarianceStamped pointer assignment"
              << std::endl;
    Pose<float, 3, true, false> adx_posef3tf_ptr_assignment;
    adx_posef3tf_ptr_assignment = ros_pose_cov_stamp_ptr;
    compare_header(ros_pose_cov_stamp_ptr->header,
                   static_cast<Header>(adx_posef3tf_ptr_assignment));
    compare_position<float, 3>(ros_pose_cov_stamp_ptr->pose.pose.position,
                               adx_posef3tf_ptr_assignment.position);
    compare_orientation<float>(ros_pose_cov_stamp_ptr->pose.pose.orientation,
                               adx_posef3tf_ptr_assignment.orientation);

    std::cout
      << "Testing Pose<double, 3, false, false> PoseWithCovarianceStamped pointer constructor"
      << std::endl;
    Pose<double, 3, false, false> adx_posed3ff_ptr_constructor(ros_pose_cov_stamp_ptr);
    compare_position<double, 3>(ros_pose_cov_stamp_ptr->pose.pose.position,
                                adx_posed3ff_ptr_constructor.position);
    compare_orientation<double>(ros_pose_cov_stamp_ptr->pose.pose.orientation,
                                adx_posed3ff_ptr_constructor.orientation);
    std::cout
      << "Testing Pose<double, 3, false, false> PoseWithCovarianceStamped pointer assignment"
      << std::endl;
    Pose<double, 3, false, false> adx_posed3ff_ptr_assignment;
    adx_posed3ff_ptr_assignment = ros_pose_cov_stamp_ptr;
    compare_position<double, 3>(ros_pose_cov_stamp_ptr->pose.pose.position,
                                adx_posed3ff_ptr_assignment.position);
    compare_orientation<double>(ros_pose_cov_stamp_ptr->pose.pose.orientation,
                                adx_posed3ff_ptr_assignment.orientation);
    std::cout
      << "Testing Pose<float, 3, false, false> PoseWithCovarianceStamped pointer constructor"
      << std::endl;
    Pose<float, 3, false, false> adx_posef3ff_ptr_constructor(ros_pose_cov_stamp_ptr);
    compare_position<float, 3>(ros_pose_cov_stamp_ptr->pose.pose.position,
                               adx_posef3ff_ptr_constructor.position);
    compare_orientation<float>(ros_pose_cov_stamp_ptr->pose.pose.orientation,
                               adx_posef3ff_ptr_constructor.orientation);
    std::cout << "Testing Pose<float, 3, false, false> PoseWithCovarianceStamped pointer assignment"
              << std::endl;
    Pose<float, 3, false, false> adx_posef3ff_ptr_assignment;
    adx_posef3ff_ptr_assignment = ros_pose_cov_stamp_ptr;
    compare_position<float, 3>(ros_pose_cov_stamp_ptr->pose.pose.position,
                               adx_posef3ff_ptr_assignment.position);
    compare_orientation<float>(ros_pose_cov_stamp_ptr->pose.pose.orientation,
                               adx_posef3ff_ptr_assignment.orientation);
}

TEST_F(PoseTest, ToRosPoseWithCovarianceStamped)
{
    const double x = 1.0, y = 2.0, z = 3.0, roll = 0.1, pitch = 0.2, yaw = 0.3;

    std::cout << "Testing Pose<double, 3, true, true> toRos PoseWithCovarianceStamped" << std::endl;
    Pose<double, 3, true, true> adx_posed3tt;
    adx_posed3tt.timestamp = std::chrono::steady_clock::now();
    adx_posed3tt.frame_id = "map";
    adx_posed3tt.position = { x, y, z };
    adx_posed3tt.orientation.setRPY(roll, pitch, yaw);
    for (int i = 0; i < 36; ++i) {
        adx_posed3tt.pose_covariance(i % 6, i / 6) = static_cast<double>(i);
    }
    geometry_msgs::msg::PoseWithCovarianceStamped ros_pose_cov_stamp_d3tt =
      adx_posed3tt.PoseHeaderCovariance::toRos();
    compare_header(ros_pose_cov_stamp_d3tt.header, static_cast<Header>(adx_posed3tt));
    compare_position<double, 3>(ros_pose_cov_stamp_d3tt.pose.pose.position, adx_posed3tt.position);
    compare_orientation<double>(ros_pose_cov_stamp_d3tt.pose.pose.orientation,
                                adx_posed3tt.orientation);
    compare_covariance<double>(ros_pose_cov_stamp_d3tt.pose.covariance,
                               adx_posed3tt.pose_covariance);

    std::cout << "Testing Pose<float, 3, true, true> toRos PoseWithCovarianceStamped" << std::endl;
    Pose<float, 3, true, true> adx_posef3tt;
    adx_posed3tt.timestamp = std::chrono::steady_clock::now();
    adx_posed3tt.frame_id = "map";
    adx_posed3tt.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    adx_posef3tt.orientation.setRPY(roll, pitch, yaw);

    for (int i = 0; i < 36; ++i) {
        adx_posef3tt.pose_covariance(i % 6, i / 6) = static_cast<float>(i);
    }
    geometry_msgs::msg::PoseWithCovarianceStamped ros_pose_cov_stamp_f3tt =
      adx_posef3tt.PoseHeaderCovariance::toRos();
    compare_header(ros_pose_cov_stamp_f3tt.header, static_cast<Header>(adx_posef3tt));
    compare_position<float, 3>(ros_pose_cov_stamp_f3tt.pose.pose.position, adx_posef3tt.position);
    compare_orientation<float>(ros_pose_cov_stamp_f3tt.pose.pose.orientation,
                               adx_posef3tt.orientation);
    compare_covariance<float>(ros_pose_cov_stamp_f3tt.pose.covariance,
                              adx_posef3tt.pose_covariance);

    std::cout << "Testing Pose<double, 3, false, true> toRos PoseWithCovarianceStamped"
              << std::endl;
    Pose<double, 3, false, true> adx_posed3ft;
    adx_posed3tt.position = { x, y, z };
    adx_posed3tt.orientation.setRPY(roll, pitch, yaw);

    for (int i = 0; i < 36; ++i) {
        adx_posed3tt.pose_covariance(i % 6, i / 6) = static_cast<double>(i);
    }
    geometry_msgs::msg::PoseWithCovarianceStamped ros_pose_cov_stamp_d3ft =
      adx_posed3ft.PoseHeaderCovariance::toRos();
    compare_position<double, 3>(ros_pose_cov_stamp_d3ft.pose.pose.position, adx_posed3ft.position);
    compare_orientation<double>(ros_pose_cov_stamp_d3ft.pose.pose.orientation,
                                adx_posed3ft.orientation);
    compare_covariance<double>(ros_pose_cov_stamp_d3ft.pose.covariance,
                               adx_posed3ft.pose_covariance);

    std::cout << "Testing Pose<float, 3, false, true> toRos PoseWithCovarianceStamped" << std::endl;
    Pose<float, 3, false, true> adx_posef3ft;
    adx_posed3tt.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    adx_posef3tt.orientation.setRPY(roll, pitch, yaw);

    for (int i = 0; i < 36; ++i) {
        adx_posef3tt.pose_covariance(i % 6, i / 6) = static_cast<float>(i);
    }
    geometry_msgs::msg::PoseWithCovarianceStamped ros_pose_cov_stamp_f3ft =
      adx_posef3ft.PoseHeaderCovariance::toRos();
    compare_position<float, 3>(ros_pose_cov_stamp_f3ft.pose.pose.position, adx_posef3ft.position);
    compare_orientation<float>(ros_pose_cov_stamp_f3ft.pose.pose.orientation,
                               adx_posef3ft.orientation);
    compare_covariance<float>(ros_pose_cov_stamp_f3ft.pose.covariance,
                              adx_posef3ft.pose_covariance);

    std::cout << "Testing Pose<double, 3, true, false> toRos PoseWithCovarianceStamped"
              << std::endl;
    Pose<double, 3, true, false> adx_posed3tf;
    adx_posed3tt.timestamp = std::chrono::steady_clock::now();
    adx_posed3tt.frame_id = "map";
    adx_posed3tt.position = { x, y, z };
    adx_posed3tt.orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::PoseWithCovarianceStamped ros_pose_cov_stamp_d3tf =
      adx_posed3tf.PoseHeaderCovariance::toRos();
    compare_header(ros_pose_cov_stamp_d3tf.header, static_cast<Header>(adx_posed3tf));
    compare_position<double, 3>(ros_pose_cov_stamp_d3tf.pose.pose.position, adx_posed3tf.position);
    compare_orientation<double>(ros_pose_cov_stamp_d3tf.pose.pose.orientation,
                                adx_posed3tf.orientation);

    std::cout << "Testing Pose<float, 3, true, false> toRos PoseWithCovarianceStamped" << std::endl;
    Pose<float, 3, true, false> adx_posef3tf;
    adx_posed3tt.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    adx_posef3tt.orientation.setRPY(roll, pitch, yaw);
    for (int i = 0; i < 36; ++i) {
        adx_posef3tt.pose_covariance(i % 6, i / 6) = static_cast<float>(i);
    }
    geometry_msgs::msg::PoseWithCovarianceStamped ros_pose_cov_stamp_f3tf =
      adx_posef3tf.PoseHeaderCovariance::toRos();
    compare_header(ros_pose_cov_stamp_f3tf.header, static_cast<Header>(adx_posef3tf));
    compare_position<float, 3>(ros_pose_cov_stamp_f3tf.pose.pose.position, adx_posef3tf.position);
    compare_orientation<float>(ros_pose_cov_stamp_f3tf.pose.pose.orientation,
                               adx_posef3tf.orientation);

    std::cout << "Testing Pose<double, 3, false, false> toRos PoseWithCovarianceStamped"
              << std::endl;
    Pose<double, 3, false, false> adx_posed3ff;
    adx_posed3tt.position = { x, y, z };
    adx_posed3tt.orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::PoseWithCovarianceStamped ros_pose_cov_stamp_d3ff =
      adx_posed3ff.PoseHeaderCovariance::toRos();
    compare_position<double, 3>(ros_pose_cov_stamp_d3ff.pose.pose.position, adx_posed3ff.position);
    compare_orientation<double>(ros_pose_cov_stamp_d3ff.pose.pose.orientation,
                                adx_posed3ff.orientation);

    std::cout << "Testing Pose<float, 3, false, false> toRos PoseWithCovarianceStamped"
              << std::endl;
    Pose<float, 3, false, false> adx_posef3ff;
    adx_posed3tt.position = { static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
    adx_posef3tt.orientation.setRPY(roll, pitch, yaw);
    geometry_msgs::msg::PoseWithCovarianceStamped ros_pose_cov_stamp_f3ff =
      adx_posef3ff.PoseHeaderCovariance::toRos();
    compare_position<float, 3>(ros_pose_cov_stamp_f3ff.pose.pose.position, adx_posef3ff.position);
    compare_orientation<float>(ros_pose_cov_stamp_f3ff.pose.pose.orientation,
                               adx_posef3ff.orientation);
}

} // namespace data
} // namespace adx
