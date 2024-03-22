#include <gtest/gtest.h>

#include "adx_data/particle.hpp"

#include "pose_helpers.hpp"

namespace adx {
namespace data {

template<typename Type>
void init_weight(internal::Weight<Type>& adx_particle)
{
    const Type weight = 0.5;
    adx_particle.weight = weight;
    EXPECT_EQ(adx_particle.weight, weight);
}

template<typename Type>
void init_particle(Particle<Type>& adx_particle)
{
    init_pose(adx_particle);
    init_weight(adx_particle);
}

template<typename Type>
void compare_weight(internal::Weight<Type> adx_weight1, internal::Weight<Type> adx_weight2)
{
    EXPECT_EQ(adx_weight1.weight, adx_weight2.weight);
}

template<typename Type>
void compare_particle(const Particle<Type>& adx_particle1, const Particle<Type>& adx_particle2)
{
    compare_pose(adx_particle1, adx_particle2);
    compare_weight(adx_particle1, adx_particle2);
}

template<typename Type>
void test_particle()
{
    Particle<Type> adx_particle;
    init_particle<Type>(adx_particle);

    Particle<Type> adx_particle_constructor(adx_particle);
    compare_particle<Type>(adx_particle, adx_particle_constructor);

    Particle<Type> adx_particle_assignment;
    adx_particle_assignment = adx_particle;
    compare_particle<Type>(adx_particle, adx_particle_assignment);
}

TEST(ParticleTest, Particled)
{
    test_particle<double>();
}
TEST(ParticleTest, Particlef)
{
    test_particle<float>();
}

} // namespace data
} // namespace adx
