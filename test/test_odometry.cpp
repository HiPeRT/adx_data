#include <iostream>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "adx_data/odometry.hpp"

#include "pose_helpers.hpp"

namespace adx::data {

nav_msgs::msg::Odometry init_ros_odometry(roscomp::Node::SharedPtr aNode)
{
    static constexpr double posx = 1.0;
    static constexpr double posy = 2.0;
    static constexpr double posz = 3.0;
    static constexpr double roll = 0.1;
    static constexpr double pitch = 0.2;
    static constexpr double yaw = 0.3;
    static constexpr double linx = 4.0;
    static constexpr double liny = 5.0;
    static constexpr double linz = 6.0;
    static constexpr double angx = 7.0;
    static constexpr double angy = 8.0;
    static constexpr double angz = 9.0;

    nav_msgs::msg::Odometry ros_odometry;
    ros_odometry.header.stamp = aNode->get_clock()->now();
    ros_odometry.header.frame_id = "map";
    ros_odometry.child_frame_id = "base_link";

    ros_odometry.pose.pose.position.x = posx;
    ros_odometry.pose.pose.position.y = posy;
    ros_odometry.pose.pose.position.z = posz;

    tf2::Quaternion tf2_quaternion;
    tf2_quaternion.setRPY(roll, pitch, yaw);

    ros_odometry.pose.pose.orientation.w = tf2_quaternion.w();
    ros_odometry.pose.pose.orientation.x = tf2_quaternion.x();
    ros_odometry.pose.pose.orientation.y = tf2_quaternion.y();
    ros_odometry.pose.pose.orientation.z = tf2_quaternion.z();

    for (int i = 0; i < 36; ++i) {
        ros_odometry.pose.covariance[i] = static_cast<double>(i);
    }

    ros_odometry.twist.twist.linear.x = linx;
    ros_odometry.twist.twist.linear.y = liny;
    ros_odometry.twist.twist.linear.z = linz;

    ros_odometry.twist.twist.angular.x = angx;
    ros_odometry.twist.twist.angular.y = angy;
    ros_odometry.twist.twist.angular.z = angz;

    for (int i = 0; i < 36; ++i) {
        ros_odometry.twist.covariance[i] = static_cast<double>(i + 36);
    }

    return ros_odometry;
}

class OdometryTest : public ::testing::Test
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

TEST_F(OdometryTest, FromRosOdometryConstructor)
{
    nav_msgs::msg::Odometry ros_odometry = init_ros_odometry(mNode);
    Odometry adx_odometryd_constructor(ros_odometry);

    compare_header(ros_odometry.header, adx_odometryd_constructor);
    compare_position<double, 3>(ros_odometry.pose.pose.position,
                                adx_odometryd_constructor.position);
    compare_orientation<double>(ros_odometry.pose.pose.orientation,
                                adx_odometryd_constructor.orientation);
    compare_covariance<double>(ros_odometry.pose.covariance,
                               adx_odometryd_constructor.pose_covariance);

    // TODO: use compare linear
    EXPECT_EQ(ros_odometry.twist.twist.linear.x, adx_odometryd_constructor.linear.x());
    EXPECT_EQ(ros_odometry.twist.twist.linear.y, adx_odometryd_constructor.linear.y());
    EXPECT_EQ(ros_odometry.twist.twist.linear.z, adx_odometryd_constructor.linear.z());

    // TODO: use compare angular
    EXPECT_EQ(ros_odometry.twist.twist.angular.x, adx_odometryd_constructor.angular.x());
    EXPECT_EQ(ros_odometry.twist.twist.angular.y, adx_odometryd_constructor.angular.y());
    EXPECT_EQ(ros_odometry.twist.twist.angular.z, adx_odometryd_constructor.angular.z());

    compare_covariance<double>(ros_odometry.twist.covariance,
                               adx_odometryd_constructor.twist_covariance);
}

TEST_F(OdometryTest, FromRosOdometryAssignment)
{
    Odometry adx_odometryd_assignment;
    roscomp::nav_msgs::Odometry ros_odometry = init_ros_odometry(mNode);
    adx_odometryd_assignment = ros_odometry;

    compare_header(ros_odometry.header, adx_odometryd_assignment);
    compare_position<double, 3>(ros_odometry.pose.pose.position, adx_odometryd_assignment.position);
    compare_orientation<double>(ros_odometry.pose.pose.orientation,
                                adx_odometryd_assignment.orientation);
    compare_covariance<double>(ros_odometry.pose.covariance,
                               adx_odometryd_assignment.pose_covariance);

    // TODO: use compare linear
    EXPECT_EQ(ros_odometry.twist.twist.linear.x, adx_odometryd_assignment.linear.x());
    EXPECT_EQ(ros_odometry.twist.twist.linear.y, adx_odometryd_assignment.linear.y());
    EXPECT_EQ(ros_odometry.twist.twist.linear.z, adx_odometryd_assignment.linear.z());

    // TODO: use compare angular
    EXPECT_EQ(ros_odometry.twist.twist.angular.x, adx_odometryd_assignment.angular.x());
    EXPECT_EQ(ros_odometry.twist.twist.angular.y, adx_odometryd_assignment.angular.y());
    EXPECT_EQ(ros_odometry.twist.twist.angular.z, adx_odometryd_assignment.angular.z());

    compare_covariance<double>(ros_odometry.twist.covariance,
                               adx_odometryd_assignment.twist_covariance);
}

} // namespace adx::data