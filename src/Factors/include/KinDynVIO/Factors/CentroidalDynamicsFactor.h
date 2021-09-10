/**
 * @file CentroidalDynamicsFactor.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNVIO_FACTORS_CENTROIDAL_DYNAMICS_FACTOR_H
#define KINDYNVIO_FACTORS_CENTROIDAL_DYNAMICS_FACTOR_H

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Rot3.h>

#include <KinDynVIO/Factors/CentroidalDynamicsMeasurementBias.h>
#include <KinDynVIO/Factors/PreintegratedCentroidalDynamicsMeasurements.h>
#include <KinDynVIO/Factors/NoiseModelFactor10.h>
#include <memory>

namespace KinDynVIO
{
namespace Factors
{

/**
 * If you are using the factor, please cite:
 * Fourmy, M., Flayols, T., Mansard, N. and Solà, J., 2021, May.
 * Contact forces pre-integration for the whole body estimation of legged robots.
 * In 2021 IEEE International Conference on Robotics and Automation-ICRA.
 *
 *
 * However, the implementation here differs slightly from the factor derived in the paper.
 * In this implementation, additional biases are considered on the net contact forces and net contact torques
 * expressed in the bias frame along with the gyroscope and the COM position bias
 * Additionally, we also apply a SE_2(3) retraction on the cost function,
 * with base rotation acting on the left-trivialized error of COM position and velocity
 */
class CentroidalDynamicsFactor : public gtsam::NoiseModelFactor10<gtsam::Rot3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector3,
                                                                  gtsam::Rot3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector3,
                                                                  gtsam::CentroidalDynamicsMeasurementBias, gtsam::CentroidalDynamicsMeasurementBias>
{
private:
   using This = CentroidalDynamicsFactor;
   using Base = gtsam::NoiseModelFactor10<gtsam::Rot3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector3,
                                          gtsam::Rot3, gtsam::Vector3, gtsam::Vector3, gtsam::Vector3,
                                          gtsam::CentroidalDynamicsMeasurementBias, gtsam::CentroidalDynamicsMeasurementBias>;
   PreintegratedCentroidalDynamicsMeasurements m_PIM;
public:
    using shared_ptr = boost::shared_ptr<This>;

    CentroidalDynamicsFactor() {}

    gtsam::NonlinearFactor::shared_ptr clone() const override;
    void print(const std::string& s = "",
               const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;
    bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const override;

    /**
     * r = vec(r_deltaRij, r_deltaCdotij, r_deltaCij, r_deltaHaij, r_deltaBij)
     * where r_deltaBij = vec(r_deltaBGyroij, r_deltaBForceij, r_deltaBTorqueij, r_deltaCOMPositionij)
     * dimension of r is 24
     * delta = vec(dphi dcdot dc dha dbGyro dbForce dbTorque dbCOMPosition)
     *
     * H1 = dr/dphi_i, H2 = dr/dcdot_i, H3 = dr/dc_i, H4 = dr/dha_i
     * H5 = dr/dphi_j, H6 = dr/dcdot_j, H7 = dr/dc_j, H8 = dr/dha_j
     * H1 to H8 matrix dimensions (24, 3)
     *
     * H9 = dr/dbi, H10 = dr/dbj of dimensions (24, 12)
     *
     */
    gtsam::Vector evaluateError(const gtsam::Rot3& R_i, const gtsam::Vector3& cdot_i, const gtsam::Vector3& c_i, const gtsam::Vector3& ha_i,
                                const gtsam::Rot3& R_j, const gtsam::Vector3& cdot_j, const gtsam::Vector3& c_j, const gtsam::Vector3& ha_j,
                                const gtsam::CentroidalDynamicsMeasurementBias& bias_i, const gtsam::CentroidalDynamicsMeasurementBias& bias_j,
                                boost::optional<gtsam::Matrix&> H1 = boost::none,
                                boost::optional<gtsam::Matrix&> H2 = boost::none,
                                boost::optional<gtsam::Matrix&> H3 = boost::none,
                                boost::optional<gtsam::Matrix&> H4 = boost::none,
                                boost::optional<gtsam::Matrix&> H5 = boost::none,
                                boost::optional<gtsam::Matrix&> H6 = boost::none,
                                boost::optional<gtsam::Matrix&> H7 = boost::none,
                                boost::optional<gtsam::Matrix&> H8 = boost::none,
                                boost::optional<gtsam::Matrix&> H9 = boost::none,
                                boost::optional<gtsam::Matrix&> H10 = boost::none) const override;

    const PreintegratedCentroidalDynamicsMeasurements& preintegratedMeasurements() const
    {
        return m_PIM;
    }

private:
    /** Serialization function */
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE& ar, const unsigned int /*version*/)
    {
        ar& boost::serialization::make_nvp("CentroidalDynamicsFactor",
                                           boost::serialization::base_object<Base>(*this));
        ar& BOOST_SERIALIZATION_NVP(m_PIM);
    }
};


} // namespace Factors
} // namespace KinDynVIO


#endif  // KINDYNVIO_FACTORS_CENTROIDAL_DYNAMICS_FACTOR_H
