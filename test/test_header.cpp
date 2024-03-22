#include <chrono>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "adx_data/header.hpp"

namespace adx {
namespace data {

class HeaderTest : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        int argc = 0;
        char** argv = 0;
        rclcpp::init(argc, argv);
        mNode = std::make_shared<rclcpp::Node>("dummy");
    }

    void TearDown() override { rclcpp::shutdown(); }

    rclcpp::Node::SharedPtr mNode;
};

TEST_F(HeaderTest, FromRos)
{
    std_msgs::msg::Header ros_header;

    ros_header.stamp = mNode->get_clock()->now();
    ros_header.frame_id = "map";

    Header adx_header_constructor(ros_header);

    EXPECT_EQ(adx_header_constructor.timestamp, ros_header.stamp);
    EXPECT_EQ(ros_header.frame_id, adx_header_constructor.frame_id);
    // EXPECT_EQ(ros_header.frame_id.substr(0, 64),
    //           std::string(adx_header_constructor.frame_id.begin(),
    //           adx_header_constructor.frame_id.begin()));

    Header adx_header_assignment;
    adx_header_assignment = ros_header;

    EXPECT_EQ(adx_header_assignment.timestamp, ros_header.stamp);
    EXPECT_EQ(ros_header.frame_id, adx_header_assignment.frame_id);
    // EXPECT_EQ(ros_header.frame_id.substr(0, 64),
    //           std::string(adx_header_assignment.frame_id.begin(),
    //           adx_header_assignment.frame_id.begin()));
}

TEST_F(HeaderTest, ToRos)
{
    Header adx_header;

    adx_header.timestamp = std::chrono::steady_clock::now();
    adx_header.frame_id = "map";

    std_msgs::msg::Header ros_header = adx_header.toRos();

    EXPECT_EQ(adx_header.timestamp, ros_header.stamp);
    EXPECT_EQ(ros_header.frame_id, adx_header.frame_id);
    // EXPECT_EQ(ros_header.frame_id,
    //           std::string(adx_header.frame_id.begin(), adx_header.frame_id.begin()));
}

} // namespace data
} // namespace adx

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
