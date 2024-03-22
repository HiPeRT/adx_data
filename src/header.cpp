// #include <rclcpp/time.hpp>

#include "adx_data/header.hpp"

namespace adx {
namespace data {

Header::Header(const roscomp::std_msgs::Header& aRosHeader)
  : timestamp(aRosHeader.stamp)
  , frame_id(aRosHeader.frame_id)
{}

Header::Header(roscomp::CallbackPtr<roscomp::std_msgs::Header> aRosHeader)
  : Header(*aRosHeader)
{}

Header& Header::operator=(Header&& aOther)
{
    if (this != &aOther) {
        timestamp = std::move(aOther.timestamp);
        frame_id = std::move(aOther.frame_id);
    }
    return *this;
}

Header& Header::operator=(const roscomp::std_msgs::Header& aRosHeader)
{
    timestamp = aRosHeader.stamp;
    frame_id = aRosHeader.frame_id;

    // std::copy(aRosHeader.frame_id.begin(),
    //           std::min(aRosHeader.frame_id.begin() + 64, aRosHeader.frame_id.end()),
    //           frame_id.data());

    return *this;
}

Header& Header::operator=(roscomp::CallbackPtr<roscomp::std_msgs::Header> aRosHeader)
{
    return operator=(*aRosHeader);
}

roscomp::std_msgs::Header Header::toRos() const
{
    roscomp::std_msgs::Header ros_header;

    ros_header.stamp = timestamp.toRos();

    ros_header.frame_id = frame_id;
    // ros_header.frame_id = std::string(frame_id.begin(), frame_id.end());

    return ros_header;
}

} // namespace data
} // namespace adx
