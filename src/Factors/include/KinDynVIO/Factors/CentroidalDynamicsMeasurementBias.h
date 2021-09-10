/**
 * @file CentroidalDynamicsMeasurementBias.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNVIO_FACTORS_CENTROIDAL_DYNAMICS_MEASUREMENT_BIAS_H
#define KINDYNVIO_FACTORS_CENTROIDAL_DYNAMICS_MEASUREMENT_BIAS_H

#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/base/Testable.h>

namespace gtsam
{
class CentroidalDynamicsMeasurementBias
{
private:
    gtsam::Vector3 biasGyro;
    gtsam::Vector3 biasNetContactForceInBase;
    gtsam::Vector3 biasNetContactTorqueInBase;
    gtsam::Vector3 biasCOMPositionInBase;

public:
    /// dimension of the variable - used to autodetect sizes
    static const std::size_t dimension = 12;

    CentroidalDynamicsMeasurementBias();
    CentroidalDynamicsMeasurementBias(const gtsam::Vector3& bGyro,
                              const gtsam::Vector3& bNetForceInBase,
                              const gtsam::Vector3& bNetTorqueInBase,
                              const gtsam::Vector3& bCOMPositionInBase);
    CentroidalDynamicsMeasurementBias(const gtsam::Vector12& v);

    gtsam::Vector12 vector() const;

    const gtsam::Vector3& gyroscope() const;
    const gtsam::Vector3& netContactForceInBase() const;
    const gtsam::Vector3& netContactTorqueInBase() const;
    const gtsam::Vector3& comPositionInBase() const;

    // Derivatives correspond to
    // H1 = df(y,b)/ db
    // H2 = df(y, b) / dy
    // where f(y, b) = y - bi
    // bi is the i-th bias selector in the set of biases
    // serialized as bg, bf,btau, bcom
    gtsam::Vector3 correctGyroscope(const gtsam::Vector3& measurement,
                                    gtsam::OptionalJacobian<3, 12> H1 = boost::none,
                                    gtsam::OptionalJacobian<3, 3> H2 = boost::none) const;
    gtsam::Vector3 correctNetContactForceInBase(const gtsam::Vector3& measurement,
                                                gtsam::OptionalJacobian<3, 12> H1 = boost::none,
                                                gtsam::OptionalJacobian<3, 3> H2 = boost::none) const;
    gtsam::Vector3 correctNetContactTorqueInBase(const gtsam::Vector3& measurement,
                                                 gtsam::OptionalJacobian<3, 12> H1 = boost::none,
                                                 gtsam::OptionalJacobian<3, 3> H2 = boost::none) const;
    gtsam::Vector3 correctCOMPositionInBase(const gtsam::Vector3& measurement,
                                            gtsam::OptionalJacobian<3, 12> H1 = boost::none,
                                            gtsam::OptionalJacobian<3, 3> H2 = boost::none) const;

    /** print with optional string */
    void print(const std::string& s = "") const;
    /** equality up to tolerance */
    bool equals(const CentroidalDynamicsMeasurementBias& expected, double tol = 1e-5) const;


    /** identity for group operation */
    static CentroidalDynamicsMeasurementBias identity()
    {
      return CentroidalDynamicsMeasurementBias();
    }

    /** inverse */
    CentroidalDynamicsMeasurementBias operator-() const;

    /** addition of vector on right */
    CentroidalDynamicsMeasurementBias operator+(const gtsam::Vector12& v) const;

    /** addition */
    CentroidalDynamicsMeasurementBias operator+(const CentroidalDynamicsMeasurementBias& b) const;

    /** subtraction */
    CentroidalDynamicsMeasurementBias operator-(const CentroidalDynamicsMeasurementBias& b) const;

};

template<>
struct traits<CentroidalDynamicsMeasurementBias> :
public internal::VectorSpace<CentroidalDynamicsMeasurementBias> {};

} // namespace gtsam

#endif  // KINDYNVIO_FACTORS_CENTROIDAL_DYNAMICS_MEASUREMENT_BIAS_H

