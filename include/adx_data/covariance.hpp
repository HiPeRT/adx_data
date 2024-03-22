#ifndef ADX_DATA_COVARIANCE_HPP
#define ADX_DATA_COVARIANCE_HPP

#ifdef TKBOOST_ENABLED
#include <boost/array.hpp>
#endif

#include <Eigen/Core>

namespace adx {
namespace data {

/**
 * @brief Size x Size covariance matrix
 *
 * Size is the number of values that may be correlated, using an Eigen matrix as the underlying
 * data type If Size is 6 this will result in a 6x6 covariance matrix.
 * For a great covariance explanation please read:
 * https://manialabs.wordpress.com/2012/08/06/covariance-matrices-with-a-practical-example/
 */
template<typename Type, unsigned long Size>
struct Covariance : public Eigen::Matrix<Type, Size, Size>
{
    /**
     * @brief Construct a new Covariance object
     *
     * Default constructor, this should just zero-initialize the matrix.
     */
    Covariance() = default;

    /**
     * @brief Construct a new Covariance object
     *
     * @param aRosCovariance a ROS covariance, as can be seen in PoseWithCovariance
     *
     * This constructor initializes the matrix using the values from a ROS covariance,
     * which is a row-major std::array of Size * Size length.
     */
    template<typename ArrayType>
    Covariance(const std::array<ArrayType, Size * Size>& aRosCovariance)
    {
        *this = aRosCovariance;
    }

#if TKBOOST_ENABLED
    /**
     * @brief Construct a new Covariance object
     *
     * @param aRosCovariance a ROS covariance, as can be seen in PoseWithCovariance
     *
     * This constructor initializes the matrix using the values from a ROS covariance,
     * which is a row-major boost::array of Size * Size length.
     */
    template<typename ArrayType>
    Covariance(const boost::array<ArrayType, Size * Size>& aRosCovariance)
    {
        *this = aRosCovariance;
    }
#endif

    /**
     * @brief Copy-assignment operator
     *
     * This method copies the covariance values from a Size * Size array to the
     * Size x Size underlying Eigen matrix.
     *
     * @param aRosCovariance a ROS covariance with values to copy
     * @return Covariance& reference to the initialized Covariance object
     */
    template<typename ArrayType>
    Covariance& operator=(const std::array<ArrayType, Size * Size>& aRosCovariance)
    {
        for (unsigned long col = 0; col < Size; ++col) {
            for (unsigned long row = 0; row < Size; ++row) {
                Eigen::Matrix<Type, Size, Size>::operator()(row, col) =
                  static_cast<Type>(aRosCovariance[col * Size + row]);
            }
        }

        return *this;
    }

    template<typename ArrayType>
    std::array<ArrayType, Size * Size> toArray() const
    {
        std::array<ArrayType, Size * Size> array;

        for (unsigned long col = 0; col < Size; ++col) {
            for (unsigned long row = 0; row < Size; ++row) {
                array[col * Size + row] =
                  static_cast<ArrayType>(Eigen::Matrix<Type, Size, Size>::operator()(row, col));
            }
        }

        return array;
    }

#ifdef TKBOOST_ENABLED
    /**
     * @brief Copy-assignment operator
     *
     * This method copies the covariance values from a Size * Size array to the
     * Size x Size underlying Eigen matrix.
     *
     * @param aRosCovariance a ROS covariance with values to copy
     * @return Covariance& reference to the initialized Covariance object
     */
    template<typename ArrayType>
    Covariance& operator=(const boost::array<ArrayType, Size * Size>& aRosCovariance)
    {
        for (unsigned long col = 0; col < Size; ++col) {
            for (unsigned long row = 0; row < Size; ++row) {
                Eigen::Matrix<Type, Size, Size>::operator()(row, col) =
                  static_cast<Type>(aRosCovariance[col * Size + row]);
            }
        }

        return *this;
    }

    template<typename ArrayType>
    boost::array<ArrayType, Size * Size> toBoostArray() const
    {
        boost::array<ArrayType, Size * Size> array;

        for (unsigned long col = 0; col < Size; ++col) {
            for (unsigned long row = 0; row < Size; ++row) {
                array[col * Size + row] =
                  static_cast<ArrayType>(Eigen::Matrix<Type, Size, Size>::operator()(row, col));
            }
        }

        return array;
    }
#endif
};

template<unsigned long Size>
using Covariancef = Covariance<float, Size>;

template<unsigned long Size>
using Covarianced = Covariance<double, Size>;

} // nmespace data
} // namespace adx

#endif // ADX_DATA_COVARIANCE_HPP
