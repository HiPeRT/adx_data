#ifndef ADX_DATA_PARTICLE_HPP
#define ADX_DATA_PARTICLE_HPP

#include "adx_data/pose.hpp"

namespace adx {
namespace data {

namespace internal {

/**
 * @brief A generic 1-dimensional weight
 *
 * @tparam Type The type of the weight
 */
template<typename Type>
struct Weight
{
    /**
     * The Type value representing the weight
     */
    Type weight;

    /**
     * @brief Construct a new Weight object
     *
     * Default constructor, this should just zero-initialize a Weight
     */
    Weight() = default;
};

} // namespace internal

/**
 * @brief A weighted estimate of robot pose
 *
 * Pose represents the estimated position and orientation of the robot.
 * Weight represents the estimated weight of this particle
 */
template<typename Type>
struct Particle
  : public Pose<Type, 2, true, false>
  , public internal::Weight<Type>
{
    /**
     * @brief inherit Pose constructors
     */
    using Pose<Type, 2, true, false>::Pose;

    /**
     * @brief inherit Weight constructors
     */
    using internal::Weight<Type>::Weight;

    /**
     * @brief Construct a new Particle object
     *
     * Default constructor, this should just zero-initialize a Particle
     */
    Particle() = default;

    /**
     * @brief Copy-construct a new Particle object
     * @param aOther a Particle to copy
     */
    // Particle(const Particle& aOther)
    //   : Pose<Type, 2, true, false>(static_cast<Pose<Type, 2, true, false>>(aOther))
    //   , internal::Weight<Type>(static_cast<internal::Weight<Type>>(aOther))
    // {}

    /**
     * @brief Move-construct a new Particle object
     * @param aOther a Particle to move
     */
    // Particle(Particle&& aOther)
    //   : Pose<Type, 2, true, false>(static_cast<Pose<Type, 2, true, false>&&>(aOther))
    //   , internal::Weight<Type>(static_cast<internal::Weight<Type>&&>(aOther))
    // {}

    /**
     * @brief Copy-assignment operator
     *
     * @param aParticle another instance of Particle
     * @return Particle& reference to the initialized Particle object
     */
    // Particle& operator=(const Particle& aParticle)
    // {
    //     Pose<Type, 2, true, false>::operator=(static_cast<Pose<Type, 2, true, false>>(aParticle));
    //     internal::Weight<Type>::operator=(static_cast<internal::Weight<Type>>(aParticle));
    // }

    /**
     * @brief Move-assignment operator
     *
     * @param aParticle a Particle message with values to move
     * @return Particle& reference to the initialized Particle object
     */
    // Particle& operator=(Particle&& aParticle)
    // {
    //     Pose<Type, 2, true, false>::operator=(static_cast<Pose<Type, 2, true, false>&&>(aParticle));
    //     internal::Weight<Type>::operator=(static_cast<internal::Weight<Type>&&>(aParticle));
    // }
};

using Particled = Particle<double>;
using Particlef = Particle<float>;

} // namespace data
} // namespace adx

#endif // ADX_DATA_PARTICLE_HPP