#include <iostream>

#include <roscomp/time.hpp>

#include "adx_data/timestamp.hpp"

namespace adx {
namespace data {

// TimeStamp::TimeStamp(const roscomp::std_msgs::Time& aRosTime)
// {
//     *this = operator=(aRosTime);
// }

TimeStamp::TimeStamp(const roscomp::time::Time& aRosTime)
{
    *this = operator=(aRosTime);
}

// TimeStamp::TimeStamp(roscomp::CallbackPtr<roscomp::std_msgs::Time> aRosTime)
//   : TimeStamp(*aRosTime)
// {}

TimeStamp& TimeStamp::operator=(TimeStamp&& aOther)
{
    timestamp = std::move(aOther.timestamp);

    return *this;
}

// TimeStamp& TimeStamp::operator=(const roscomp::std_msgs::Time& aRosTime)
// {
//     timestamp = std::chrono::time_point<std::chrono::steady_clock>(
//       std::chrono::nanoseconds(roscomp::time::to_nanosec(aRosTime));

//     return *this;
// }

TimeStamp& TimeStamp::operator=(const roscomp::time::Time& aRosTime)
{
    timestamp = std::chrono::time_point<std::chrono::steady_clock>(
      std::chrono::nanoseconds(static_cast<uint64_t>(roscomp::time::to_nanosec(aRosTime))));

    return *this;
}

// TimeStamp& TimeStamp::operator=(roscomp::CallbackPtr<roscomp::std_msgs::Time> aRosTime)
// {
//     return operator=(aRosTime);
// }

TimeStamp& TimeStamp::operator=(const std::chrono::steady_clock::time_point& aTimePoint)
{
    timestamp = aTimePoint;

    return *this;
}

bool TimeStamp::operator<(const TimeStamp& rhs) const
{
    return timestamp < rhs.timestamp;
}

bool TimeStamp::operator<=(const TimeStamp& rhs) const
{
    return timestamp <= rhs.timestamp;
}

bool TimeStamp::operator>(const TimeStamp& rhs) const
{
    return timestamp > rhs.timestamp;
}

bool TimeStamp::operator>=(const TimeStamp& rhs) const
{
    return timestamp >= rhs.timestamp;
}

bool TimeStamp::operator==(const TimeStamp& rhs) const
{

    return timestamp == rhs.timestamp;
}

bool operator==(const TimeStamp& lhs, const roscomp::time::Time& rhs)
{
    return (
      static_cast<uint64_t>(std::chrono::time_point_cast<std::chrono::nanoseconds>(lhs.timestamp)
                              .time_since_epoch()
                              .count()) == roscomp::time::to_nanosec(rhs));
}
bool operator==(const roscomp::time::Time& lhs, const TimeStamp& rhs)
{
    return operator==(rhs, lhs);
}

bool TimeStamp::operator!=(const TimeStamp& rhs) const
{
    return timestamp != rhs.timestamp;
}

std::ostream& operator<<(std::ostream& os, const TimeStamp& rhs)
{
    os << std::chrono::time_point_cast<std::chrono::nanoseconds>(rhs.timestamp)
            .time_since_epoch()
            .count();
    return os;
}

std::ofstream& operator<<(std::ofstream& ofs, const TimeStamp& rhs)
{
    ofs << std::chrono::time_point_cast<std::chrono::nanoseconds>(rhs.timestamp)
             .time_since_epoch()
             .count();
    return ofs;
}

roscomp::time::Time TimeStamp::toRos() const
{
    roscomp::time::Time ros_time;

    // static uint64_t sec;
    // sec =
    //   std::chrono::time_point_cast<std::chrono::nanoseconds>(timestamp).time_since_epoch().count()
    //   / static_cast<uint64_t>(1e9);

    // ros_time.sec = static_cast<int32_t>(sec);
    // ros_time.nanosec = static_cast<uint32_t>(
    //   std::chrono::time_point_cast<std::chrono::nanoseconds>(timestamp).time_since_epoch().count()
    //   - static_cast<uint64_t>(sec) * static_cast<uint64_t>(1e9));

    // return ros_time;

    return roscomp::time::from_sec(
      static_cast<double>(std::chrono::time_point_cast<std::chrono::nanoseconds>(timestamp)
                            .time_since_epoch()
                            .count()) /
      1e9);
}

} // namespace data
} // namespace adx
