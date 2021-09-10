/**
 * @file PreintegratedCentroidalDynamicsMeasurements.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <KinDynVIO/Factors/PreintegratedCentroidalDynamicsMeasurements.h>

#include <gtsam/base/Testable.h>
#include <gtsam/geometry/SO3.h>

using namespace KinDynVIO::Factors;

////////////////////////////////////////////////////////
/// Preintegrated Centroidal Dynamics Params methods ///
///////////////////////////////////////////////////////

void PreintegrationCentroidalDynamicsParams::print(const std::string& s) const
{
    std::cout << (s.empty() ? s : s + "\n") << std::endl;
    std::cout << "gyroscopeCovariance:\n[\n" << gyroscopeCovariance << "\n]" << std::endl;
    std::cout << "contactForceCovariance:\n[\n" << contactForceCovariance << "\n]" << std::endl;
    std::cout << "contactTorqueCovariance:\n[\n" << contactTorqueCovariance << "\n]" << std::endl;
    std::cout << "comPositionCovariance:\n[\n" << comPositionCovariance << "\n]" << std::endl;
    std::cout << "base_H_imu:\n[\n" << base_H_imu << "\n]" << std::endl;
}

bool PreintegrationCentroidalDynamicsParams::equals(
    const PreintegrationCentroidalDynamicsParams& other, double tol) const
{
    if (gtsam::assert_equal(base_H_imu, other.base_H_imu, tol))
    {
        return false;
    }

    bool ok{true};
    ok = ok && gtsam::equal_with_abs_tol(gyroscopeCovariance, other.gyroscopeCovariance, tol);
    ok = ok && gtsam::equal_with_abs_tol(contactForceCovariance, other.contactForceCovariance, tol);
    ok = ok
         && gtsam::equal_with_abs_tol(contactTorqueCovariance, other.contactTorqueCovariance, tol);
    ok = ok && gtsam::equal_with_abs_tol(comPositionCovariance, other.comPositionCovariance, tol);
    return ok;
}

//////////////////////////////////////////////////////////////
/// Preintegrated Centroidal Dynamics Measurements methods ///
//////////////////////////////////////////////////////////////
void PreintegratedCentroidalDynamicsMeasurements::print(const std::string& s) const
{
    std::cout << (s.empty() ? s : s + "\n") << std::endl;
    std::cout << "    deltaTij = " << deltaTij_ << std::endl;
    std::cout << "    deltaRij.ypr = (" << deltaRij().ypr().transpose() << ")" << std::endl;
    std::cout << "    deltaCij = " << deltaCij().transpose() << std::endl;
    std::cout << "    deltaCdotij = " << deltaCdotij().transpose() << std::endl;
    std::cout << "    deltaHaij = " << deltaHaij().transpose() << std::endl;
    std::cout << "    gyroBias = " << biasHat_.gyroscope().transpose() << std::endl;
    std::cout << "    netContactForceBiasInBase = " << biasHat_.netContactForceInBase().transpose()
              << std::endl;
    std::cout << "    netContactTorqueBiasInBase = "
              << biasHat_.netContactTorqueInBase().transpose() << std::endl;
    std::cout << "    comBiasInBase = " << biasHat_.comPositionInBase().transpose() << std::endl;
    std::cout << "    preintMeasCov [ \n" << preintMeasCov_ << " ]" << std::endl;
}

bool PreintegratedCentroidalDynamicsMeasurements::equals(
    const PreintegratedCentroidalDynamicsMeasurements& other, double tol) const
{
    bool ok{true};
    ok = ok && p_->equals(*other.p_, tol);
    ok = ok && std::abs(deltaTij_ - other.deltaTij_) < tol;
    ok = ok && biasHat_.equals(other.biasHat_);
    ok = gtsam::equal_with_abs_tol(delRdelBiasGyro_, other.delRdelBiasGyro_, tol);
    ok = gtsam::equal_with_abs_tol(delCdelBiasGyro_, other.delCdelBiasGyro_, tol);
    ok = gtsam::equal_with_abs_tol(delCdelBiasNetForce_, other.delCdelBiasNetForce_, tol);
    ok = gtsam::equal_with_abs_tol(delCdotdelBiasGyro_, other.delCdotdelBiasGyro_, tol);
    ok = gtsam::equal_with_abs_tol(delCdotdelBiasNetForce_, other.delCdotdelBiasNetForce_, tol);
    ok = gtsam::equal_with_abs_tol(delHadelBiasGyro_, other.delHadelBiasGyro_, tol);
    ok = gtsam::equal_with_abs_tol(delHadelBiasNetForce_, other.delHadelBiasNetForce_, tol);
    ok = gtsam::equal_with_abs_tol(delHadelBiasNetTorque_, other.delHadelBiasNetTorque_, tol);
    ok = gtsam::equal_with_abs_tol(delHadelBiasCOMPosition_,
                                   other.delHadelBiasCOMPosition_,
                                   tol);
    ok = gtsam::equal_with_abs_tol(preintMeasCov_, other.preintMeasCov_, tol);

    // check also for centroidal state

    return ok;
}

gtsam::Vector12 PreintegratedCentroidalDynamicsMeasurements::computeErrorAndJacobians(
    const gtsam::Rot3& R_i,
    const gtsam::Vector3& cdot_i,
    const gtsam::Vector3& c_i,
    const gtsam::Vector3& ha_i,
    const gtsam::Rot3& R_j,
    const gtsam::Vector3& cdot_j,
    const gtsam::Vector3& c_j,
    const gtsam::Vector3& ha_j,
    const gtsam::CentroidalDynamicsMeasurementBias& bias_i,
    const gtsam::CentroidalDynamicsMeasurementBias& bias_j,
    gtsam::OptionalJacobian<24, 3> H1,
    gtsam::OptionalJacobian<24, 3> H2,
    gtsam::OptionalJacobian<24, 3> H3,
    gtsam::OptionalJacobian<24, 3> H4,
    gtsam::OptionalJacobian<24, 3> H5,
    gtsam::OptionalJacobian<24, 3> H6,
    gtsam::OptionalJacobian<24, 3> H7,
    gtsam::OptionalJacobian<24, 3> H8,
    gtsam::OptionalJacobian<24, 12> H9,
    gtsam::OptionalJacobian<24, 12> H10) const
{
    gtsam::Vector12 error = computeError(R_i, cdot_i, c_i, ha_i,
                                         R_j, cdot_j, c_j, ha_j,
                                         bias_i, bias_j);

    gtsam::Vector3 r_DRij = error.head<3>();
    gtsam::so3::DexpFunctor Jr_rDRij(r_DRij);
    gtsam::Matrix3 Jrinv = Jr_rDRij.dexp().inverse();
    gtsam::so3::ExpmapFunctor Exp_rDRij(r_DRij);
    gtsam::Rot3 R_i_T = R_i.inverse();
    gtsam::Rot3 R_i_TxR_j = R_i_T*R_j;
    gtsam::Vector3 gDtij= gravity_*deltaTij_;

    // dr/dphi_i
    if (H1)
    {
        H1->resize(residualDim, 3);
        H1->setZero();
        // H11 = dr_DRij/dphi_i
        H1->block<3, 3>(0, 0) = - Jrinv * (R_j.inverse()*R_i).matrix();
        // H12 = dr_DCdotij/dphi_i
        H1->block<3, 3>(3, 0) = gtsam::skewSymmetric(
            R_i_T.rotate(cdot_j - cdot_i - gDtij));
        // H13 = dr_DCij/dphi_i
        H1->block<3, 3>(6, 0) = gtsam::skewSymmetric(
            R_i_T.rotate(c_j - c_i - (cdot_i*deltaTij_) - (0.5*gDtij*deltaTij_)));
        // H14 = dr_DHaij/dphi_i
        H1->block<3, 3>(9, 0) = gtsam::skewSymmetric(
            R_i_T.rotate(ha_j - ha_i));
    }

    // dr/dcdot_i
    if (H2)
    {
        H2->resize(residualDim, 3);
        H2->setZero();
        // H22 = dr_DCdotij/dcdot_i
        H2->block<3, 3>(3, 0) = -gtsam::I_3x3;
        // H23 = dr_DCij/dcdot_i
        H2->block<3, 3>(6, 0) = -gtsam::I_3x3*deltaTij_;
    }

    // dr/dc_i
    if (H3)
    {
        H3->resize(residualDim, 3);
        H3->setZero();
        // H33 = dr_DCij/dc_i
        H3->block<3, 3>(6, 0) = -gtsam::I_3x3;
    }

    // dr/dha_i
    if (H4)
    {
        H4->resize(residualDim, 3);
        H4->setZero();
        // H44 = dr_DHaij/dha_i
        H4->block<3, 3>(9, 0) = -R_i_T.matrix();
    }

    // dr/dphi_j
    if (H5)
    {
        H5->resize(residualDim, 3);
        H5->setZero();
        // H51 = dr_DRaij/dphi_j
        H5->block<3, 3>(0, 0) = Jrinv;
    }

    // dr/dcdot_j
    if (H6)
    {
        H6->resize(residualDim, 3);
        H6->setZero();
        // H62 = dr_DCdotij/dcdot_j
        H6->block<3, 3>(0, 3) = R_i_TxR_j.matrix();
    }

    // dr/dc_j
    if (H7)
    {
        H7->resize(residualDim, 3);
        H7->setZero();
        // H73 = dr_DCij/dc_j
        H7->block<3, 3>(0, 6) = R_i_TxR_j.matrix();
    }

    // dr/dha_j
    if (H8)
    {
        H8->resize(residualDim, 3);
        H8->setZero();
        // H84 = dr_DHaij/dha_j
        H8->block<3, 3>(0, 9) = R_i_T.matrix();
    }

    auto& bDim {gtsam::CentroidalDynamicsMeasurementBias::dimension};
    // dr/db_i
    if (H9)
    {
        H9->resize(residualDim, bDim);
        // dr_DRij/dbGyro
        // TODO the computation of this bias Jacobian should be verified
        gtsam::Matrix3 JrDel = gtsam::so3::DexpFunctor(delRdelBiasGyro_*bias_i.gyroscope()).dexp();
        gtsam::Matrix3 ExprDrijT = (Exp_rDRij.expmap().inverse()).matrix();
        H9->block<3, 3>(0, 0) = -Jrinv*ExprDrijT*JrDel*delRdelBiasGyro_;
        H9->block<3, 9>(0, 3).setZero();

        // dr_DCdotij/dbGyro
        H9->block<3, 3>(3, 0) = -delCdotdelBiasGyro_;
        // dr_DCdotij/dbForce
        H9->block<3, 3>(3, 3) = -delCdotdelBiasNetForce_;
        H9->block<3, 6>(3, 6).setZero();

        // dr_DCij/dbGyro
        H9->block<3, 3>(6, 0) = -delCdelBiasGyro_;
        // dr_DCij/dbForce
        H9->block<3, 3>(6, 3) = -delCdelBiasNetForce_;
        H9->block<3, 6>(6, 6).setZero();

        // dr_DHaij/dbGyro
        H9->block<3, 3>(9, 0) = -delHadelBiasGyro_;
        // dr_DHaij/dbForce
        H9->block<3, 3>(9, 3) = -delHadelBiasNetForce_;
        // dr_DHaij/dbTorque
        H9->block<3, 3>(9, 6) = -delHadelBiasNetTorque_;
        // dr_DHaij/dbCOMPos
        H9->block<3, 3>(9, 9) = -delHadelBiasCOMPosition_;

        H9->block<12, 12>(12, 0) = -Eigen::MatrixXd::Identity(bDim, bDim);
    }

    // dr/db_j
    if (H10)
    {
        H10->resize(residualDim, bDim);
        H10->setZero();
        H10->block<12, 12>(12, 0).setIdentity();
    }

    return error;
}


gtsam::Vector12 PreintegratedCentroidalDynamicsMeasurements::computeError(
    const gtsam::Rot3& R_i,
    const gtsam::Vector3& cdot_i,
    const gtsam::Vector3& c_i,
    const gtsam::Vector3& ha_i,
    const gtsam::Rot3& R_j,
    const gtsam::Vector3& cdot_j,
    const gtsam::Vector3& c_j,
    const gtsam::Vector3& ha_j,
    const gtsam::CentroidalDynamicsMeasurementBias& bias_i,
    const gtsam::CentroidalDynamicsMeasurementBias& bias_j) const
{
    gtsam::Vector12 error;
    return error;
}
