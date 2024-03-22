#include <gtest/gtest.h>

#include "adx_data/covariance.hpp"

namespace adx {
namespace data {

TEST(CovarianceTest, FromDoubleArray)
{
    std::array<double, 36> ros_covariance;

    for (int i = 0; i < 36; ++i) {
        ros_covariance[i] = static_cast<double>(i);
    }

    Covarianced<6> adx_covarianced_constructor(ros_covariance);

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(adx_covarianced_constructor(i % 6, i / 6), ros_covariance[i]);
    }

    Covarianced<6> adx_covarianced_assignment = ros_covariance;

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(adx_covarianced_assignment(i % 6, i / 6), ros_covariance[i]);
    }

    Covariancef<6> adx_covariancef_constructor(ros_covariance);

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(adx_covariancef_constructor(i % 6, i / 6), static_cast<float>(ros_covariance[i]));
    }

    Covariancef<6> adx_covariancef_assignment = ros_covariance;

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(adx_covariancef_assignment(i % 6, i / 6), static_cast<float>(ros_covariance[i]));
    }
}

TEST(CovarianceTest, ToDoubleArray)
{
    Covarianced<6> adx_covarianced;
    for (int i = 0; i < 36; ++i) {
        adx_covarianced(i % 6, i / 6) = static_cast<double>(i);
    }

    std::array<double, 36> ros_covariance = adx_covarianced.template toArray<double>();

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(adx_covarianced(i % 6, i / 6), ros_covariance[i]);
    }

    Covariancef<6> adx_covariancef;
    for (int i = 0; i < 36; ++i) {
        adx_covariancef(i % 6, i / 6) = static_cast<float>(i);
    }

    ros_covariance = adx_covariancef.template toArray<double>();

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(adx_covariancef(i % 6, i / 6), static_cast<float>(ros_covariance[i]));
    }
}

TEST(CovarianceTest, FromFloatArray)
{
    std::array<float, 36> ros_covariance;

    for (int i = 0; i < 36; ++i) {
        ros_covariance[i] = static_cast<float>(i);
    }

    Covarianced<6> adx_covarianced_constructor(ros_covariance);

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(static_cast<float>(adx_covarianced_constructor(i % 6, i / 6)), ros_covariance[i]);
    }

    Covarianced<6> adx_covarianced_assignment = ros_covariance;

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(static_cast<float>(adx_covarianced_assignment(i % 6, i / 6)), ros_covariance[i]);
    }

    Covariancef<6> adx_covariancef_constructor(ros_covariance);

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(adx_covariancef_constructor(i % 6, i / 6), ros_covariance[i]);
    }

    Covariancef<6> adx_covariancef_assignment = ros_covariance;

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(adx_covariancef_assignment(i % 6, i / 6), ros_covariance[i]);
    }
}

TEST(CovarianceTest, toFloatArray)
{
    Covarianced<6> adx_covarianced;

    for (int i = 0; i < 36; ++i) {
        adx_covarianced(i % 6, i / 6) = static_cast<double>(i);
    }

    std::array<float, 36> ros_covariance = adx_covarianced.template toArray<float>();

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(static_cast<float>(adx_covarianced(i % 6, i / 6)), ros_covariance[i]);
    }

    Covariancef<6> adx_covariancef;
    for (int i = 0; i < 36; ++i) {
        adx_covariancef(i % 6, i / 6) = static_cast<float>(i);
    }

    ros_covariance = adx_covariancef.template toArray<float>();

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(adx_covariancef(i % 6, i / 6), ros_covariance[i]);
    }
}

} // namespace data
} // namespace adx
