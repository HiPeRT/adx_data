cmake_minimum_required(VERSION 3.15)
project(adx_data)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ROS_VERSION $ENV{ROS_VERSION})

if(ROS_VERSION EQUAL 2)
    find_package(ament_cmake REQUIRED)
endif()

find_package(roscomp REQUIRED)

if(ROS_VERSION EQUAL 1)
    find_package(Boost REQUIRED)
endif()

find_package(Eigen3 REQUIRED)

# Covariance
add_library(covariance INTERFACE)
add_library(${PROJECT_NAME}::covariance ALIAS covariance)
target_include_directories(covariance INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(covariance INTERFACE
    Eigen3::Eigen
)
if(Boost_FOUND)
    target_link_libraries(covariance INTERFACE
        Boost::boost
    )
    target_compile_options(covariance INTERFACE -DTKBOOST_ENABLED)
endif()

# TimeStamp
add_library(timestamp STATIC
    src/timestamp.cpp
)
add_library(${PROJECT_NAME}::timestamp ALIAS timestamp)
target_include_directories(timestamp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(timestamp
    PUBLIC
        Eigen3::Eigen
    PRIVATE
        roscomp::roscomp
    INTERFACE
        roscomp::roscompstd_msgs
)

# vector
add_library(vector INTERFACE)
add_library(${PROJECT_NAME}::vector ALIAS vector)
target_include_directories(vector INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(vector INTERFACE
    Eigen3::Eigen
    roscomp::roscompgeometry_msgs
)

# Header
add_library(header STATIC
    src/header.cpp
)
add_library(${PROJECT_NAME}::header ALIAS header)
target_include_directories(header PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(header PUBLIC
    adx_data::timestamp
    Eigen3::Eigen
)

# Point
add_library(point INTERFACE)
add_library(${PROJECT_NAME}::point ALIAS point)
target_include_directories(point INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(point INTERFACE
    adx_data::vector
)

# Quaternion
add_library(quaternion INTERFACE)
add_library(${PROJECT_NAME}::quaternion ALIAS quaternion)
target_include_directories(quaternion INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(quaternion INTERFACE
    Eigen3::Eigen
)

# Pose
add_library(pose INTERFACE)
add_library(${PROJECT_NAME}::pose ALIAS pose)
target_include_directories(pose INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(pose INTERFACE
    adx_data::header
    adx_data::covariance
    adx_data::point
    adx_data::quaternion
    adx_data::vector
)

# Particle
add_library(particle INTERFACE)
add_library(${PROJECT_NAME}::particle ALIAS particle)
target_include_directories(particle INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(particle INTERFACE
    adx_data::pose
)

# Twist
add_library(twist STATIC
    src/twist.cpp
)
add_library(${PROJECT_NAME}::twist ALIAS twist)
target_include_directories(twist PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(twist PUBLIC
    adx_data::covariance
    adx_data::vector
    roscomp::roscompgeometry_msgs
)

# Path
add_library(path STATIC
    src/path.cpp
)
add_library(${PROJECT_NAME}::path ALIAS path)
target_include_directories(path PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(path PUBLIC
    adx_data::header
    adx_data::pose
    adx_data::vector
    roscomp::roscompadx_msgs
)

# Odometry
add_library(odometry STATIC
    src/odometry.cpp
)
add_library(${PROJECT_NAME}::odometry ALIAS odometry)
target_include_directories(odometry PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(odometry PUBLIC
    adx_data::covariance
    adx_data::header
    adx_data::pose
    adx_data::twist
)

# Obstacle
add_library(obstacle STATIC
    src/obstacle.cpp
)
add_library(${PROJECT_NAME}::obstacle ALIAS obstacle)
target_include_directories(obstacle PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(obstacle PUBLIC
    adx_data::covariance
    adx_data::pose
    adx_data::twist
    adx_data::vector
    roscomp::roscompadx_msgs
)

# Install and export
install(TARGETS
    covariance
    header
    obstacle
    odometry
    path
    point
    pose
    particle
    quaternion
    timestamp
    twist
    vector
    EXPORT ${PROJECT_NAME}Targets
)

install(DIRECTORY
    include/
    DESTINATION include
)

install(EXPORT ${PROJECT_NAME}Targets
    NAMESPACE ${PROJECT_NAME}::
    FILE ${PROJECT_NAME}Targets.cmake
    DESTINATION share/${PROJECT_NAME}/cmake
)

include(CMakePackageConfigHelpers)
configure_package_config_file(Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION 1.0.0
    COMPATIBILITY AnyNewerVersion
)

export(EXPORT ${PROJECT_NAME}Targets
    NAMESPACE ${PROJECT_NAME}::
    FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION share/${PROJECT_NAME}/cmake
)

if(ROS_VERSION EQUAL 2)
    #ament_package()
endif()

if(BUILD_TESTING)
    enable_testing()

    find_package(GTest CONFIG REQUIRED)
    find_package(tf2 REQUIRED)

    include(CTest)
    include(GoogleTest)

    add_executable(test_covariance
        test/test_covariance.cpp
    )
    target_link_libraries(test_covariance PRIVATE
        adx_data::covariance
        GTest::gtest_main
    )

    add_executable(test_header
        test/test_header.cpp
    )
    target_link_libraries(test_header PRIVATE
        adx_data::header
        GTest::gtest_main
    )

    add_executable(test_odometry
        test/test_odometry.cpp
    )
    target_link_libraries(test_odometry PRIVATE
        adx_data::odometry
        GTest::gtest_main
    )
    ament_target_dependencies(test_odometry PUBLIC
        tf2
    )


    add_executable(test_point
        test/test_point.cpp
    )
    target_link_libraries(test_point PRIVATE
        adx_data::point
        GTest::gtest_main
    )

    add_executable(test_pose
        test/test_pose.cpp
    )
    target_link_libraries(test_pose PRIVATE
        adx_data::pose
        GTest::gtest_main
    )
    ament_target_dependencies(test_pose PUBLIC
        tf2
    )

    add_executable(test_particle
        test/test_particle.cpp
    )
    target_link_libraries(test_particle PRIVATE
        adx_data::pose
        GTest::gtest_main
    )
    ament_target_dependencies(test_particle PUBLIC
        tf2
    )

    add_executable(test_quaternion
        test/test_quaternion.cpp
    )
    target_link_libraries(test_quaternion PRIVATE
        adx_data::pose
        GTest::gtest_main
    )
    ament_target_dependencies(test_quaternion PUBLIC
        tf2
    )

    add_test(NAME test_covariance
        COMMAND test_covariance
    )
    add_test(NAME test_header
        COMMAND test_header
    )
    add_test(NAME test_odometry
        COMMAND test_odometry
    )
    add_test(NAME test_point
        COMMAND test_point
    )
    add_test(NAME test_pose
        COMMAND test_pose
    )
    add_test(NAME test_particle
        COMMAND test_particle
    )
    add_test(NAME test_quaternion
        COMMAND test_quaternion
    )
endif()
