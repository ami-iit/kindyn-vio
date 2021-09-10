/**
 * @file CentroidalDynamicsFactor.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <KinDynVIO/Factors/CentroidalDynamicsFactor.h>

using namespace KinDynVIO::Factors;

gtsam::NonlinearFactor::shared_ptr CentroidalDynamicsFactor::clone() const
{
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
}

void CentroidalDynamicsFactor::print(const std::string& s,
                                     const gtsam::KeyFormatter& keyFormatter) const
{
}

bool CentroidalDynamicsFactor::equals(const NonlinearFactor& other, double tol) const
{
    const This* e = dynamic_cast<const This*>(&other);
    return e != nullptr && Base::equals(*e, tol) && m_PIM.equals(e->m_PIM, tol);
}

gtsam::Vector
CentroidalDynamicsFactor::evaluateError(const gtsam::Rot3& R_i,
                                        const gtsam::Vector3& cdot_i,
                                        const gtsam::Vector3& c_i,
                                        const gtsam::Vector3& ha_i,
                                        const gtsam::Rot3& R_j,
                                        const gtsam::Vector3& cdot_j,
                                        const gtsam::Vector3& c_j,
                                        const gtsam::Vector3& ha_j,
                                        const gtsam::CentroidalDynamicsMeasurementBias& bias_i,
                                        const gtsam::CentroidalDynamicsMeasurementBias& bias_j,
                                        boost::optional<gtsam::Matrix&> H1,
                                        boost::optional<gtsam::Matrix&> H2,
                                        boost::optional<gtsam::Matrix&> H3,
                                        boost::optional<gtsam::Matrix&> H4,
                                        boost::optional<gtsam::Matrix&> H5,
                                        boost::optional<gtsam::Matrix&> H6,
                                        boost::optional<gtsam::Matrix&> H7,
                                        boost::optional<gtsam::Matrix&> H8,
                                        boost::optional<gtsam::Matrix&> H9,
                                        boost::optional<gtsam::Matrix&> H10) const
{
    // error wrt bias evolution model (random walk)
    Eigen::Matrix<double, 12, 1> fbias = gtsam::traits<gtsam::CentroidalDynamicsMeasurementBias>::Between(bias_j, bias_j).vector();

    gtsam::Vector12 rWithoutBias = m_PIM.computeErrorAndJacobians(R_i, cdot_i, c_i, ha_i,
                                                                  R_j, cdot_j, c_j, ha_j,
                                                                  bias_i, bias_j,
                                                                  H1, H2, H3, H4, H5,
                                                                  H6, H7, H8, H9, H10);
    gtsam::Vector r(rWithoutBias.size() + fbias.size());
    r << rWithoutBias, fbias;
    return r;
}
