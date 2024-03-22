#ifndef ADX_DATA_TIMESTAMP_HPP
#define ADX_DATA_TIMESTAMP_HPP

#include <iostream>
#include <fstream>
#include <chrono>

#include <roscomp/time.hpp>
#include <roscomp/msgs/std_msgs.hpp>

namespace adx {
namespace data {

/**
 * @brief Generic data header with timestamp and frame_id
 *
 * This header stores a timestamp of the received data and a frame_id to identify in which set of
 * coordinates a specific message resides
 */
struct TimeStamp
{
    /**
     * @brief Nanoseconds from epoch
     *
     * An exact representation of time, stored as a chrono::time_point
     */
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> timestamp = {};

    /**
     * @brief Construct a new TimeStamp object
     *
     * Default constructor, this should just zero-initialize the matrix.
     */
    TimeStamp() = default;

    /**
     * @brief Copy-constructor
     *
     * @param aTimeStamp a TimeStamp with values to move
     * @return TimeStamp& reference to the initialized TimeStamp object
     */
    constexpr TimeStamp(const TimeStamp& aTimeStamp) { *this = operator=(aTimeStamp); }

    /**
     * @brief Construct a new TimeStamp object
     *
     * @param aRosTime a ROS time message
     *
     * This constructor initializes the timestampg using the values from a ROS time.
     */
    // TimeStamp(const roscomp::std_msgs::Time& aRosTime);

    /**
     * @brief Construct a new TimeStamp object
     *
     * @param aRosTime a ROS time class
     *
     * This constructor initializes the timestampg using the values from a ROS time.
     */
    TimeStamp(const roscomp::time::Time& aRosTime);

    /**
     * @brief Construct a new TimeStamp object
     *
     * @param aRosTime pointer to a ROS time message
     *
     * This constructor initializes the timestampg using the values from a ROS time.
     */
    // TimeStamp(roscomp::CallbackPtr<roscomp::std_msgs::Time> aRosTime);

    /**
     * @brief Copy-assignment operator
     *
     * @param aTimeStamp a TimeStamp with values to copy
     * @return TimeStamp& reference to the initialized TimeStamp object
     */
    constexpr TimeStamp& operator=(const TimeStamp& aTimeStamp)
    {
        timestamp = aTimeStamp.timestamp;

        return *this;
    }

    /**
     * @brief Move-assignment operator
     *
     * @param aOther a TimeStamp with values to move
     * @return TimeStamp& reference to the initialized TimeStamp object
     */
    TimeStamp& operator=(TimeStamp&& aOther);

    /**
     * @brief Copy-assignment operator
     *
     * This method copies the time stamp and frame id from a ROS time.
     *
     * @param aRosTime a ROS time message with values to copy
     * @return TimeStamp& reference to the initialized TimeStamp object
     */
    // TimeStamp& operator=(const roscomp::std_msgs::Time& aRosTime);

    /**
     * @brief Copy-assignment operator
     *
     * This method copies the time stamp and frame id from a ROS time.
     *
     * @param aRosTime a ROS time class with values to copy
     * @return TimeStamp& reference to the initialized TimeStamp object
     */
    TimeStamp& operator=(const roscomp::time::Time& aRosTime);

    /**
     * @brief Copy-assignment operator
     *
     * This method copies the time stamp and frame id from a ROS time.
     *
     * @param aRosTime pointer to a ROS time message with values to copy
     * @return TimeStamp& reference to the initialized TimeStamp object
     */
    // TimeStamp& operator=(roscomp::CallbackPtr<roscomp::std_msgs::Time> aRosTime);

    /**
     * @brief Copy-assignment operator
     *
     * This method copies the time from an std::chrono time_point
     * @param aTimePoint an std::chrono time_point to copy
     * @return TimeStamp& reference to the initialized TimeStamp object
     */
    TimeStamp& operator=(const std::chrono::steady_clock::time_point& aTimePoint);

    /**
     * @brief Less-than operator
     *
     * @param rhs right-hand-side argument
     * @return true if lhs's timestamp is less than rhs's timestamp
     * @return false otherwise
     */
    bool operator<(const TimeStamp& rhs) const;

    /**
     * @brief Less-than or equal operator
     *
     * @param rhs right-hand-side argument
     * @return true if lhs's timestamp is less than or equal to rhs's timestamp
     * @return false otherwise
     */
    bool operator<=(const TimeStamp& rhs) const;

    /**
     * @brief Greater-than operator
     *
     * @param rhs right-hand-side argument
     * @return true if lhs's timestamp is greater than rhs's timestamp
     * @return false otherwise
     */
    bool operator>(const TimeStamp& rhs) const;

    /**
     * @brief Less-than operator
     *
     * @param rhs right-hand-side argument
     * @return true if lhs's timestamp is greater than or equal to rhs's timestamp
     * @return false otherwise
     */
    bool operator>=(const TimeStamp& rhs) const;

    /**
     * @brief Equal-to operator
     *
     * @param rhs right-hand-side argument
     * @return true if lhs's timestamp is equal to rhs's timestamp
     * @return false otherwise
     */
    bool operator==(const TimeStamp& rhs) const;

    /**
     * @brief Equal-to operator
     *
     * @param lhs a TimeStamp
     * @param rhs a ROS Time msg
     * @return true if lhs's timestamp is equal to rhs's timestamp
     * @return false otherwise
     */
    friend bool operator==(const TimeStamp& lhs, const roscomp::time::Time& rhs);

    /**
     * @brief Equal-to operator
     *
     * @param lhs a TimeStamp
     * @param rhs a ROS Time msg
     * @return true if lhs's timestamp is equal to rhs's timestamp
     * @return false otherwise
     */
    friend bool operator==(const roscomp::time::Time& lhs, const TimeStamp& rhs);

    /**
     * @brief Not-equal-to operator
     *
     * @param rhs right-hand-side argument
     * @return true if lhs's timestamp is not equal to rhs's timestamp
     * @return false otherwise
     */
    bool operator!=(const TimeStamp& rhs) const;

    /**
     * @brief std::ostream insertion operator
     *
     * @param os reference to ofstream
     * @param rhs reference to TimeStamp
     * @return reference to inserted std::ostream
     */
    friend std::ostream& operator<<(std::ostream& os, const TimeStamp& rhs);

    /**
     * @brief std::ostream insertion operator
     *
     * @param os reference to ofstream
     * @param rhs reference to TimeStamp
     * @return reference to inserted std::ofstream
     */
    friend std::ofstream& operator<<(std::ofstream& ofs, const TimeStamp& rhs);

    /**
     * @brief Convert this data to ROS message
     *
     * @return roscomp::time::Time ROS message initialized from this class
     */
    roscomp::time::Time toRos() const;
};

} // nmespace data
} // namespace adx

#endif // ADX_DATA_TIMESTAMP_HPP
