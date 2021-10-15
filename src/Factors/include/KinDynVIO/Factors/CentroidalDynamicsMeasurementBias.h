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

/**
 * Cumulative centroidal dynamics measurement bias
 * the biases on the external force torque measurements
 * are considered to be cumulatively acting on the base frame directly
 * instead of modeling individual biases on each external force torque measurement.
 * The latter might lead to observability issues in case of handling
 * too many external force torque measurements.
 */
class CDMBiasCumulative
{
private:
    gtsam::Vector3 m_biasCOMPositionInBase;
    gtsam::Vector3 m_biasNetExternalForceInBase;
    gtsam::Vector3 m_biasNetExternalTorqueInBase; // contains net effect of contact force measurement lever arm biases and contact torque measurement biases

public:
    /// dimension of the variable - used to autodetect sizes
    inline static const std::size_t dimension = 12;

    CDMBiasCumulative();
    CDMBiasCumulative(const gtsam::Vector3& bCOMPositionInBase,
                      const gtsam::Vector3& bNetForceInBase,
                      const gtsam::Vector3& bNetTorqueInBase);
    CDMBiasCumulative(const gtsam::Vector9& v);

    gtsam::Vector9 vector() const;

    const gtsam::Vector3& comPositionInBase() const;
    const gtsam::Vector3& netExternalForceInBase() const;
    const gtsam::Vector3& netExternalTorqueInBase() const;

    // Derivatives correspond to
    // H1 = df(y,b)/ db
    // H2 = df(y, b) / dy
    // where f(y, b) = y - bi
    // bi is the i-th bias selector in the set of biases
    // serialized as bcom, bf,btau
    gtsam::Vector3 correctCOMPositionInBase(const gtsam::Vector3& measurement,
                                            gtsam::OptionalJacobian<3, 9> H1 = boost::none,
                                            gtsam::OptionalJacobian<3, 3> H2 = boost::none) const;
    gtsam::Vector3 correctNetExternalForceInBase(const gtsam::Vector3& measurement,
                                                 gtsam::OptionalJacobian<3, 9> H1 = boost::none,
                                                 gtsam::OptionalJacobian<3, 3> H2 = boost::none) const;
    gtsam::Vector3 correctNetExternalTorqueInBase(const gtsam::Vector3& measurement,
                                                  gtsam::OptionalJacobian<3, 9> H1 = boost::none,
                                                  gtsam::OptionalJacobian<3, 3> H2 = boost::none) const;

    /** print with optional string */
    void print(const std::string& s = "") const;
    /** equality up to tolerance */
    bool equals(const CDMBiasCumulative& other, double tol = 1e-5) const;


    /** identity for group operation */
    static CDMBiasCumulative identity()
    {
      return CDMBiasCumulative();
    }

    /** inverse */
    CDMBiasCumulative operator-() const;

    /** addition of vector on right */
    CDMBiasCumulative operator+(const gtsam::Vector9& v) const;

    /** addition */
    CDMBiasCumulative operator+(const CDMBiasCumulative& other) const;

    /** subtraction */
    CDMBiasCumulative operator-(const CDMBiasCumulative& other) const;

private:
    /// @name Advanced Interface
    /// @{

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
        ar & BOOST_SERIALIZATION_NVP(m_biasCOMPositionInBase);
        ar & BOOST_SERIALIZATION_NVP(m_biasNetExternalForceInBase);
        ar & BOOST_SERIALIZATION_NVP(m_biasNetExternalTorqueInBase);
    }

};

template<>
struct traits<CDMBiasCumulative> :
public internal::VectorSpace<CDMBiasCumulative> {};

} // namespace gtsam

#endif  // KINDYNVIO_FACTORS_CENTROIDAL_DYNAMICS_MEASUREMENT_BIAS_H

