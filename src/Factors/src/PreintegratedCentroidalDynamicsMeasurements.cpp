/**
 * @file PreintegratedCentroidalDynamicsMeasurements.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynVIO/Factors/PreintegratedCentroidalDynamicsMeasurements.h>

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/Math/Constants.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Model.h>

#include <gtsam/base/Testable.h>
#include <gtsam/geometry/SO3.h>

using namespace KinDynVIO::Factors;

constexpr double massThreshold{1e-6};

/////////////////////
/// Naming Sugars ///
/////////////////////

// some sugars for delta residuals
#define rDRij(r) (r)->head<3>()
#define rDcdotij(r) (r)->segment<3>(3)
#define rDcij(r) (r)->segment<3>(6)
#define rDhaij(r) (r)->segment<3>(9)
#define rDbiasij(r) (r)->tail<12>()

// some sugars for bias increment vector
#define dbRij(b) (b)->head<3>()
#define dbcdotij(b) (b)->segment<3>(3)
#define dbcij(b) (b)->segment<3>(6)
#define dbhaij(b) (b)->segment<3>(9)

// some sugars for delta residuals Jacobians
#define J_rDRij_phi_i(H1)     (H1)->block<3,3>(0,0)
#define J_rDcdotij_phi_i(H1)  (H1)->block<3,3>(3,0)
#define J_rDcij_phi_i(H1)     (H1)->block<3,3>(6,0)
#define J_rDhaij_phi_i(H1)    (H1)->block<3,3>(9,0)

#define J_rDcdotij_cdot_i(H2) (H2)->block<3,3>(3,0)
#define J_rDcij_cdot_i(H2)    (H2)->block<3,3>(6,0)

#define J_rDcij_c_i(H3)       (H3)->block<3,3>(6,0)

#define J_rDhaij_ha_i(H4)     (H4)->block<3,3>(9,0)

#define J_rDRij_phi_j(H5)     (H5)->block<3,3>(0,0)

#define J_rDcdotij_cdot_j(H6) (H6)->block<3,3>(3,0)

#define J_rDcij_c_j(H7)       (H7)->block<3,3>(6,0)

#define J_rDhaij_ha_j(H8)     (H8)->block<3,3>(9,0)

#define J_rDRij_bg_i(H9)      (H9)->block<3,3>(0,3)
#define J_rDcdotij_bg_i(H9)   (H9)->block<3,3>(3,3)
#define J_rDcij_bg_i(H9)      (H9)->block<3,3>(6,3)
#define J_rDhaij_bg_i(H9)     (H9)->block<3,3>(9,3)
#define J_rDbiasij_bg_i(H9)   (H9)->block<12,12>(12, 3)

#define J_rDRij_bG_i(H10)      (H10)->block<3,3>(0,0)
#define J_rDRij_bf_i(H10)      (H10)->block<3,3>(0,3)
#define J_rDRij_bt_i(H10)      (H10)->block<3,3>(0,6)
#define J_rDcdotij_bG_i(H10)   (H10)->block<3,3>(3,0)
#define J_rDcdotij_bf_i(H10)   (H10)->block<3,3>(3,3)
#define J_rDcdotij_bt_i(H10)   (H10)->block<3,3>(3,6)
#define J_rDcij_bG_i(H10)      (H10)->block<3,3>(6,0)
#define J_rDcij_bf_i(H10)      (H10)->block<3,3>(6,3)
#define J_rDcij_bt_i(H10)      (H10)->block<3,3>(6,6)
#define J_rDhaij_bG_i(H10)     (H10)->block<3,3>(9,0)
#define J_rDhaij_bf_i(H10)     (H10)->block<3,3>(9,3)
#define J_rDhaij_bt_i(H10)     (H10)->block<3,3>(9,6)
#define J_rDbiasij_b_i(H10)    (H10)->block<9,9>(15,0)

#define J_rDbiasij_bg_j(H11)   (H11)->block<3,3>(12,3)

#define J_rDbiasij_b_j(H12)   (H12)->block<9,9>(15,0)

// some sugars for noise propagation matrix
#define A_phi_phi(A)  (A)->block<3,3>(0,0)
#define A_cdot_phi(A) (A)->block<3,3>(3,0)
#define A_c_phi(A)    (A)->block<3,3>(6,0)
#define A_ha_phi(A)   (A)->block<3,3>(9,0)


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
    std::cout << "base_H_imu:\n[\n" << baseHImu << "\n]" << std::endl;
}

bool PreintegrationCentroidalDynamicsParams::equals(
    const PreintegrationCentroidalDynamicsParams& other, double tol) const
{
    if (gtsam::assert_equal(baseHImu, other.baseHImu, tol))
    {
        return false;
    }

    bool ok{true};
    ok = ok && gtsam::equal_with_abs_tol(gyroscopeCovariance, other.gyroscopeCovariance, tol);
    ok = ok && gtsam::equal_with_abs_tol(contactForceCovariance, other.contactForceCovariance, tol);
    ok = ok && gtsam::equal_with_abs_tol(contactTorqueCovariance, other.contactTorqueCovariance, tol);
    ok = ok && gtsam::equal_with_abs_tol(comPositionCovariance, other.comPositionCovariance, tol);
    return ok;
}

//////////////////////////////////////////////////////////////
/// Preintegrated Centroidal Dynamics Measurements methods ///
//////////////////////////////////////////////////////////////
PreintegratedCDMCumulativeBias::PreintegratedCDMCumulativeBias()
{
    m_gravity = gtsam::Vector3(0., 0., -BipedalLocomotion::Math::StandardAccelerationOfGravitation);
    m_p = std::make_shared<Params>();
    resetIntegration();
}

PreintegratedCDMCumulativeBias::PreintegratedCDMCumulativeBias(const std::shared_ptr<Params>& p,
                                                               const ImuBias& imuBiasHat,
                                                               const CDMBias& cdmBiasHat)
{
    m_gravity = gtsam::Vector3(0., 0., -BipedalLocomotion::Math::StandardAccelerationOfGravitation);
    m_p = p;
    resetIntegration();
    m_imuBiasHat = imuBiasHat;
    m_cdmBiasHat = cdmBiasHat;
}

void PreintegratedCDMCumulativeBias::print(const std::string& s) const
{
    std::cout << (s.empty() ? s : s + "\n") << std::endl;
    std::cout << "    deltaTij = " << m_deltaTij << std::endl;
    std::cout << "    deltaRij.ypr = (" << deltaRij().ypr().transpose() << ")" << std::endl;
    std::cout << "    deltaCij = " << deltaCij().transpose() << std::endl;
    std::cout << "    deltaCdotij = " << deltaCdotij().transpose() << std::endl;
    std::cout << "    deltaHaij = " << deltaHaij().transpose() << std::endl;
    std::cout << "    gyroBias = " << m_imuBiasHat.gyroscope().transpose() << std::endl;
    std::cout << "    comBiasInBase = " << m_cdmBiasHat.comPositionInBase().transpose() << std::endl;
    std::cout << "    netContactForceBiasInBase = " << m_cdmBiasHat.netExternalForceInBase().transpose()
              << std::endl;
    std::cout << "    netContactTorqueBiasInBase = "
              << m_cdmBiasHat.netExternalTorqueInBase().transpose() << std::endl;
    std::cout << "    preintMeasCov [ \n" << m_preintMeasCov << " ]" << std::endl;
}

bool PreintegratedCDMCumulativeBias::equals(
    const PreintegratedCDMCumulativeBias& other, double tol) const
{
    bool ok{true};
    ok = ok && m_p->equals(*other.m_p, tol);
    ok = ok && std::abs(m_deltaTij - other.m_deltaTij) < tol;
    ok = ok && m_imuBiasHat.equals(other.m_imuBiasHat);
    ok = ok && m_cdmBiasHat.equals(other.m_cdmBiasHat);
    ok = ok && gtsam::equal_with_abs_tol(m_delRdelBiasGyro, other.m_delRdelBiasGyro, tol);
    ok = ok && gtsam::equal_with_abs_tol(m_delCdelBiasGyro, other.m_delCdelBiasGyro, tol);
    ok = ok && gtsam::equal_with_abs_tol(m_delCdelBiasNetForce, other.m_delCdelBiasNetForce, tol);
    ok = ok && gtsam::equal_with_abs_tol(m_delCdotdelBiasGyro, other.m_delCdotdelBiasGyro, tol);
    ok = ok && gtsam::equal_with_abs_tol(m_delCdotdelBiasNetForce, other.m_delCdotdelBiasNetForce, tol);
    ok = ok && gtsam::equal_with_abs_tol(m_delHadelBiasGyro, other.m_delHadelBiasGyro, tol);
    ok = ok && gtsam::equal_with_abs_tol(m_delHadelBiasNetTorque, other.m_delHadelBiasNetTorque, tol);
    ok = ok && gtsam::equal_with_abs_tol(m_delHadelBiasCOMPosition,
                                         other.m_delHadelBiasCOMPosition,
                                         tol);
    ok = ok && gtsam::equal_with_abs_tol(m_preintMeasCov, other.m_preintMeasCov, tol);

    // check also for centroidal state
    ok = ok && gtsam::equal_with_abs_tol(m_deltaRij.matrix(), other.m_deltaRij.matrix(), tol);
    ok = ok && gtsam::equal_with_abs_tol(m_deltaCdotij, other.m_deltaCdotij, tol);
    ok = ok && gtsam::equal_with_abs_tol(m_deltaCij, other.m_deltaCij, tol);
    ok = ok && gtsam::equal_with_abs_tol(m_deltaHaij, other.m_deltaHaij, tol);

    return ok;
}

void PreintegratedCDMCumulativeBias::resetIntegration()
{
    m_deltaTij = 0.0;
    m_preintMeasCov.resize(m_residualDim, m_residualDim);
    m_preintMeasCov.setZero();

    m_deltaRij.identity();
    m_deltaCdotij.setZero();
    m_deltaCij.setZero();
    m_deltaHaij.setZero();

    m_delRdelBiasGyro.setZero();
    m_delCdotdelBiasGyro.setZero();
    m_delCdotdelBiasNetForce.setZero();
    m_delCdelBiasGyro.setZero();
    m_delCdelBiasNetForce.setZero();
    m_delHadelBiasGyro.setZero();
    m_delHadelBiasNetTorque.setZero();
    m_delHadelBiasCOMPosition.setZero();
}

bool PreintegratedCDMCumulativeBias::updateCDMModelComputations(const std::shared_ptr<iDynTree::KinDynComputations>& kinDyn,
                                                                const LocalContactWrenchesMap& localContactWrenches)
{
    const std::string printPrefix{"[PreintegratedCDMCumulativeBias::updateCDMModelComputations]"};
    m_currModelComp.mass = kinDyn->getRobotModel().getTotalMass();
    if (m_currModelComp.mass < massThreshold)
    {
        BipedalLocomotion::log()->error("{} Invalid robot mass.",
                                        printPrefix);
        return false;
    }

    m_currModelComp.nrExtWrenches = localContactWrenches.size();
    m_currModelComp.comInBase = iDynTree::toEigen(kinDyn->getWorldBaseTransform().inverse()
                                             *kinDyn->getCenterOfMassPosition());
    m_currModelComp.unBiasedCoMInBase = m_cdmBiasHat.correctCOMPositionInBase(m_currModelComp.comInBase);
    const gtsam::Vector3& B_obar_G = m_currModelComp.unBiasedCoMInBase; // alias ref

    m_currModelComp.contactFrameLeverArmsInBase.clear();
    m_currModelComp.contactFrameRotationsInBase.clear();
    m_currModelComp.netExtForceInBase.setZero();
    m_currModelComp.netExtForceInBase.setZero();
    m_currModelComp.netUnbiasedExtForceInBase.setZero();
    m_currModelComp.netUnbiasedExtTorqueInBase.setZero();

    auto baseLinkIdx = kinDyn->getFrameIndex(m_p->baseLinkName);
    if (baseLinkIdx == iDynTree::FRAME_INVALID_INDEX)
    {
        BipedalLocomotion::log()->error("{} Invalid base link name: {}.",
                                        printPrefix, m_p->baseLinkName);
        return false;
    }

    // for every contact wrench measurement
    // transform it to the base frame and
    // accumulate them as a net wrench
    // also collect lever arm terms and rotations
    // which will be used to fill the covariance matrix
    for (const auto& [id, wrench] : localContactWrenches)
    {
        if (kinDyn->getRobotModel().isValidFrameIndex(id))
        {
            auto B_H_L = kinDyn->getRelativeTransform(baseLinkIdx, id);
            auto B_R_L = iDynTree::toEigen(B_H_L.getRotation());
            auto B_o_L = iDynTree::toEigen(B_H_L.getPosition());
            gtsam::Vector3 B_p_GL = B_o_L - B_obar_G;
            const gtsam::Vector3& B_f = B_R_L*wrench.head<3>();
            const gtsam::Vector3& B_tau = B_R_L*wrench.tail<3>();

            m_currModelComp.contactFrameRotationsInBase[id] = B_R_L;
            m_currModelComp.contactFrameLeverArmsInBase[id] = B_p_GL;
            m_currModelComp.netExtForceInBase += B_f;
            m_currModelComp.netExtTorqueInBase += gtsam::skewSymmetric(B_p_GL)*B_f + B_tau;
        }
        else
        {
            BipedalLocomotion::log()->error("{} Contact Frame index: {} not found in the model.",
                                            printPrefix, id);
            return false;
        }
    }

    m_currModelComp.netUnbiasedExtForceInBase =
                m_cdmBiasHat.correctNetExternalForceInBase(m_currModelComp.netExtForceInBase);
    m_currModelComp.netUnbiasedExtTorqueInBase =
                m_cdmBiasHat.correctNetExternalTorqueInBase(m_currModelComp.netExtTorqueInBase);

    m_currModelComp.SBfnet = gtsam::skewSymmetric(m_currModelComp.netExtForceInBase);
    m_currModelComp.SBtaunet = gtsam::skewSymmetric(m_currModelComp.netExtTorqueInBase);

    return true;
}

void PreintegratedCDMCumulativeBias::updateCDMGyroComputations(const gtsam::Vector3& localGyroMeas,
                                                               const double& dt)
{
    // DRkk+1  = DRik.T DRik+1 = Exp(omega dt)
    m_currGyroComp.unbiasedGyro = m_imuBiasHat.correctGyroscope(localGyroMeas);

    const gtsam::Vector3& omegaBar = m_currGyroComp.unbiasedGyro;
    m_currGyroComp.DRkkplusone = gtsam::Rot3::Expmap(omegaBar*dt);
    m_currGyroComp.Jr = gtsam::so3::DexpFunctor(omegaBar*dt).dexp();
    m_currGyroComp.DRik = m_deltaRij;
}

void PreintegratedCDMCumulativeBias::prepareA(const CDMModelComputations& modelComp,
                                              const CDMGyroComputations& gyroComp,
                                              const double& dt)
{
    double dtSq{dt*dt};
    const auto& m = modelComp.mass;
    const auto& DRik = gyroComp.DRik.matrix();
    const auto& DRkkplusone = gyroComp.DRkkplusone;

    const gtsam::Matrix3& Sf = modelComp.SBfnet;
    const gtsam::Matrix3& Stau = modelComp.SBtaunet;

    if (m_A.rows() != m_residualDim && m_A.cols() != m_residualDim)
    {
        m_A.resize(m_residualDim, m_residualDim);
        m_A.setIdentity();

        // assuming that m_A is not modified elsewhere
        // and only parts related to dphi
        // is modified, rest of the matrix
        // should remain an identity matrix block
    }

    A_phi_phi(&m_A) = DRkkplusone.inverse().matrix();
    A_cdot_phi(&m_A)= -(1/m)*DRik*Sf*dt;
    A_c_phi(&m_A) = -(3/2*m)*DRik*Sf*dtSq;
    A_ha_phi(&m_A) = -DRik*Stau*dt;
}

void PreintegratedCDMCumulativeBias::prepareG(const CDMModelComputations& modelComp,
                                              const CDMGyroComputations& gyroComp,
                                              const double& dt)
{
    const std::size_t obsDim = 6 + modelComp.nrExtWrenches*6;
    const auto& m = modelComp.mass;
    const auto oneByM = (1/m);
    const auto threeByTwoM{3/2*m};
    const auto& DRik = gyroComp.DRik.matrix();

    // both m_B and m_Sigma_n dimensions
    // might change every step depending
    // on number of external wrenches
    m_B.resize(m_residualDim, obsDim);
    m_B.setZero();
    m_Sigma_n.resize(obsDim, obsDim);
    m_Sigma_n.setZero();

    const std::size_t comIdxOffset = 3 + modelComp.nrExtWrenches*6;
    const std::size_t bgOffset{comIdxOffset+3};
    const std::size_t bGOffset{bgOffset+3};
    const std::size_t bfOffset{bGOffset+3};
    const std::size_t btOffset{bfOffset+3};

    m_Sigma_n.block<3,3>(0,0) = m_p->getGyroscopeCovariance();
    m_B.block<3,3>(0, 0) = gyroComp.Jr;

    std::size_t linkIdx{1};
    for (const auto& [id, B_R_L] : modelComp.contactFrameRotationsInBase)
    {
        const std::size_t forceIdxOffset = linkIdx*3;
        const std::size_t torqueIdxOffset = modelComp.nrExtWrenches*3 + linkIdx*3;

        const gtsam::Vector3 B_p_GL = modelComp.contactFrameLeverArmsInBase.at(id);
        const gtsam::Matrix3 SpGL = gtsam::skewSymmetric(B_p_GL);
        const gtsam::Matrix3 B_R_Ldt = B_R_L*dt;
        const gtsam::Matrix3 DRik_B_R_Ldt= DRik*B_R_Ldt;

        m_Sigma_n.block<3, 3>(forceIdxOffset, forceIdxOffset) = m_p->getContactForceCovariance();
        m_Sigma_n.block<3, 3>(torqueIdxOffset, torqueIdxOffset) = m_p->getContactTorqueCovariance();

        m_B.block<3, 3>(3, forceIdxOffset) = oneByM*DRik_B_R_Ldt;
        m_B.block<3, 3>(6, forceIdxOffset) = threeByTwoM*DRik_B_R_Ldt*dt;
        m_B.block<3, 3>(9, forceIdxOffset) = DRik*SpGL*B_R_Ldt;
        m_B.block<3, 3>(9, torqueIdxOffset) = DRik_B_R_Ldt;
        linkIdx++;
    }

    const gtsam::Matrix3& Sf = modelComp.SBfnet;
    m_Sigma_n.block<3, 3>(comIdxOffset, comIdxOffset) = m_p->getCOMPositionCovariance();
    // update dha with com position measurement noise
    m_B.block<3, 3>(9, comIdxOffset) = DRik*Sf*dt;
    m_Sigma_n.block<3,3>(bgOffset, bgOffset) = m_p->getGyroscopeBiasCovariance();
    m_Sigma_n.block<3,3>(bGOffset, bGOffset) = m_p->getCOMPositionBiasCovariance();
    m_Sigma_n.block<3,3>(bfOffset, bfOffset) = m_p->getContactForceBiasCovariance();
    m_Sigma_n.block<3,3>(btOffset, btOffset) = m_p->getContactTorqueBiasCovariance();

    m_G = m_B*m_Sigma_n*(m_B.transpose());
}

void PreintegratedCDMCumulativeBias::resetIntegrationAndSetBias(const ImuBias& imuBiasHat,
                                                                const CDMBias& cdmBiasHat)
{
    resetIntegration();
    m_imuBiasHat = imuBiasHat;
    m_cdmBiasHat = cdmBiasHat;
}

bool PreintegratedCDMCumulativeBias::update(const std::shared_ptr<iDynTree::KinDynComputations>& kinDyn,
                                            const gtsam::Vector3& localGyroMeas,
                                            const LocalContactWrenchesMap& localContactWrenches,
                                            const double& dt)
{
    const std::string printPrefix{"[PreintegratedCDMCumulativeBias::update]"};
    updateCDMGyroComputations(localGyroMeas, dt);
    if (!updateCDMModelComputations(kinDyn, localContactWrenches))
    {
        BipedalLocomotion::log()->error("{} Failed to update model computations.",
                                        printPrefix);
        return false;
    }

    updatePreintegratedMeasurements(m_currModelComp, m_currGyroComp, dt);
    propagatePreintegrationCovariance(m_currModelComp, m_currGyroComp, dt);
    updateBiasJacobians(m_currModelComp, m_currGyroComp, dt);

    return true;
}

void PreintegratedCDMCumulativeBias::updatePreintegratedMeasurements(const CDMModelComputations& modelComp,
                                                                     const CDMGyroComputations& gyroComp,
                                                                     const double& dt)
{
    const auto& m = modelComp.mass;
    const auto& DRik = gyroComp.DRik;
    const auto& ExpOmegaDt = gyroComp.DRkkplusone;
    const gtsam::Vector3& B_f_net = modelComp.netUnbiasedExtForceInBase;
    const gtsam::Vector3& B_tau_net = modelComp.netUnbiasedExtTorqueInBase;

    m_deltaRij = m_deltaRij*ExpOmegaDt;
    m_deltaCdotij += (1/m)*DRik.rotate(B_f_net)*dt;
    m_deltaCij += (3/2*m)*DRik.rotate(B_f_net)*dt*dt;
    m_deltaHaij += DRik.rotate(B_tau_net)*dt;
}

void PreintegratedCDMCumulativeBias::updateBiasJacobians(const CDMModelComputations& modelComp,
                                                         const CDMGyroComputations& gyroComp,
                                                         const double& dt)
{
    double dtSq{dt*dt};
    const auto& m = modelComp.mass;
    const double oneByM = (1/m);
    const double threeByTwoM = (3/2*m);
    const gtsam::Matrix3& DRik = gyroComp.DRik.matrix();

    const auto& DRkkplusone = gyroComp.DRkkplusone;
    const gtsam::Matrix3 DRkkplusoneT = DRkkplusone.inverse().matrix();
    const gtsam::Matrix3& Sf = modelComp.SBfnet;
    const gtsam::Matrix3& Stau = modelComp.SBtaunet;

    gtsam::Matrix3 dRdbg_old = m_delRdelBiasGyro;
    const gtsam::Matrix3 DRik_dt = DRik*dt;
    const gtsam::Matrix3 DRik_Sf = DRik*Sf;
    const gtsam::Matrix3 DRik_Sf_dRbdbg = DRik_Sf*dRdbg_old;
    const gtsam::Matrix3 DRik_Stau = DRik*Stau;
    const gtsam::Matrix3 DRik_Stau_dRbdbg = DRik_Stau*dRdbg_old;

    m_delRdelBiasGyro = DRkkplusoneT*dRdbg_old - gyroComp.Jr*dt;
    m_delCdotdelBiasGyro += (-oneByM*DRik_Sf_dRbdbg*dt);
    m_delCdotdelBiasNetForce += (-oneByM*DRik_dt);
    m_delCdelBiasGyro += (-threeByTwoM*DRik_Sf_dRbdbg*dtSq);
    m_delCdelBiasNetForce += (-threeByTwoM*DRik_dt*dt);
    m_delHadelBiasGyro += (-DRik_Stau_dRbdbg*dt);
    m_delHadelBiasNetTorque += (-DRik_dt);
    m_delHadelBiasCOMPosition += (-DRik_Sf*dt);
}


void PreintegratedCDMCumulativeBias::propagatePreintegrationCovariance(const CDMModelComputations& modelComp,
                                                                       const CDMGyroComputations& gyroComp,
                                                                       const double& dt)
{
    prepareA(modelComp, gyroComp, dt);
    prepareG(modelComp, gyroComp, dt);
    m_preintMeasCov += m_A*m_preintMeasCov*(m_A.transpose()) + m_G;
}

gtsam::Vector PreintegratedCDMCumulativeBias::computeErrorAndJacobians(
    const gtsam::Pose3& H_i,
    const gtsam::Vector3& cdot_i,
    const gtsam::Vector3& c_i,
    const gtsam::Vector3& ha_i,
    const gtsam::Pose3& H_j,
    const gtsam::Vector3& cdot_j,
    const gtsam::Vector3& c_j,
    const gtsam::Vector3& ha_j,
    const ImuBias& imuBias_i,
    const gtsam::CDMBiasCumulative& cdmBias_i,
    const ImuBias& imuBias_j,
    const gtsam::CDMBiasCumulative& cdmBias_j,
    gtsam::OptionalJacobian<24, 6> H1,
    gtsam::OptionalJacobian<24, 3> H2,
    gtsam::OptionalJacobian<24, 3> H3,
    gtsam::OptionalJacobian<24, 3> H4,
    gtsam::OptionalJacobian<24, 6> H5,
    gtsam::OptionalJacobian<24, 3> H6,
    gtsam::OptionalJacobian<24, 3> H7,
    gtsam::OptionalJacobian<24, 3> H8,
    gtsam::OptionalJacobian<24, 6> H9,
    gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H10,
    gtsam::OptionalJacobian<24, 6> H11,
    gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H12) const
{
    // underlying gtsam types are Eigen
    // so to be safe not to use auto
    const gtsam::Rot3& R_i = H_i.rotation();
    const gtsam::Rot3& R_i_T = R_i.inverse();
    const gtsam::Rot3& R_j = H_j.rotation();
    const gtsam::Rot3 R_i_T_R_j = R_i_T*R_j;
    const gtsam::Vector3 gDtij= m_gravity*m_deltaTij;

    const gtsam::Rot3 dR = R_i_T*R_j;
    const gtsam::Vector3 dcDot = R_i_T.rotate(cdot_j - cdot_i - gDtij);
    const gtsam::Vector3 dc = R_i_T.rotate(c_j - c_i - (cdot_i*m_deltaTij) - (0.5*gDtij*m_deltaTij));
    const gtsam::Vector3 dha = R_i_T.rotate(ha_j - ha_i);
    const ImuBias dImuBias = imuBias_j - imuBias_i;
    const CDMBias dCdmBias = cdmBias_j - cdmBias_i;

    const gtsam::Vector error = computeError(dR, dcDot, dc, dha,
                                             dImuBias, dCdmBias,
                                             imuBias_i, cdmBias_i);
    const gtsam::Vector3 r_DRij = rDRij(&error);
    const gtsam::so3::DexpFunctor Jr_rDRij(r_DRij);
    const gtsam::Matrix3 Jrinv = Jr_rDRij.dexp().inverse();
    const gtsam::so3::ExpmapFunctor Exp_rDRij(r_DRij);

    // dr/dpsi_i (where psi_i = [dphi_i, dp_i])
    if (H1)
    {
        H1->resize(m_residualDim, 6);
        H1->setZero();
        // H11 = dr_DRij/dphi_i
        J_rDRij_phi_i(H1) = - Jrinv * (R_j.inverse()*R_i).matrix();
        // H21 = dr_DCdotij/dphi_i
        J_rDcdotij_phi_i(H1) = gtsam::skewSymmetric(dcDot);
        // H31 = dr_DCij/dphi_i
        J_rDcij_phi_i(H1) = gtsam::skewSymmetric(dc);
        // H41 = dr_DHaij/dphi_i
        J_rDhaij_phi_i(H1) = gtsam::skewSymmetric(dha);
    }

    // dr/dcdot_i
    if (H2)
    {
        H2->resize(m_residualDim, 3);
        H2->setZero();
        // H22 = dr_DCdotij/dcdot_i
        J_rDcdotij_cdot_i(H2) = -gtsam::I_3x3;
        // H32 = dr_DCij/dcdot_i
        J_rDcij_cdot_i(H2) = -gtsam::I_3x3*m_deltaTij;
    }

    // dr/dc_i
    if (H3)
    {
        H3->resize(m_residualDim, 3);
        H3->setZero();
        // H33 = dr_DCij/dc_i
        J_rDcij_c_i(H3) = -gtsam::I_3x3;
    }

    // dr/dha_i
    if (H4)
    {
        H4->resize(m_residualDim, 3);
        H4->setZero();
        // H44 = dr_DHaij/dha_i
        J_rDhaij_ha_i(H4) = -R_i_T.matrix();
    }

    // dr/dphi_j
    if (H5)
    {
        H5->resize(m_residualDim, 6);
        H5->setZero();
        // H15 = dr_DRaij/dphi_j
        J_rDRij_phi_j(H5) = Jrinv;
    }

    // dr/dcdot_j
    if (H6)
    {
        H6->resize(m_residualDim, 3);
        H6->setZero();
        // H26 = dr_DCdotij/dcdot_j
        J_rDcdotij_cdot_j(H6) = R_i_T_R_j.matrix();
    }

    // dr/dc_j
    if (H7)
    {
        H7->resize(m_residualDim, 3);
        H7->setZero();
        // H37 = dr_DCij/dc_j
        J_rDcij_c_j(H7) = R_i_T_R_j.matrix();
    }

    // dr/dha_j
    if (H8)
    {
        H8->resize(m_residualDim, 3);
        H8->setZero();
        // H48 = dr_DHaij/dha_j
        J_rDhaij_ha_j(H8) = R_i_T.matrix();
    }

    auto& bCdmDim {gtsam::CDMBiasCumulative::dimension};
    const std::size_t bImuDim{6};
    if (H9)
    {
        H9->resize(m_residualDim, bImuDim);
        H9->setZero();

        const auto deltaBias = imuBias_i.gyroscope() - m_imuBiasHat.gyroscope();
        const gtsam::Matrix3 JrDel = gtsam::so3::DexpFunctor(m_delRdelBiasGyro*deltaBias).dexp();
        const gtsam::Matrix3 ExprDrijT = (Exp_rDRij.expmap().inverse()).matrix();

        // accelerometer bias column must be zero
        // serialized as accBias, gyroBias
        J_rDRij_bg_i(H9) = -Jrinv*ExprDrijT*JrDel*m_delRdelBiasGyro;
        J_rDcdotij_bg_i(H9) = -m_delCdotdelBiasGyro;
        J_rDcij_bg_i(H9) = -m_delCdelBiasGyro;
        J_rDhaij_bg_i(H9) = -m_delHadelBiasGyro;
        J_rDbiasij_bg_i(H9) = -Eigen::MatrixXd::Identity(3, 3);
    }

    if (H10)
    {
        H10->resize(m_residualDim, bCdmDim);
        H10->setZero();
        J_rDRij_bG_i(H10).setZero();
        J_rDRij_bf_i(H10).setZero();
        J_rDRij_bt_i(H10).setZero();

        J_rDcdotij_bG_i(H10).setZero();
        J_rDcdotij_bf_i(H10) = -m_delCdotdelBiasNetForce;
        J_rDcdotij_bt_i(H10).setZero();

        J_rDcij_bG_i(H10).setZero();
        J_rDcij_bf_i(H10) = -m_delCdelBiasNetForce;
        J_rDcij_bt_i(H10).setZero();

        J_rDhaij_bG_i(H10) = -m_delHadelBiasCOMPosition;
        J_rDhaij_bf_i(H10).setZero();
        J_rDhaij_bt_i(H10) = -m_delHadelBiasNetTorque;

        J_rDbiasij_b_i(H10) = -Eigen::MatrixXd::Identity(bCdmDim, bCdmDim);
    }

    if (H11)
    {
        H11->resize(m_residualDim, bImuDim);
        H11->setZero();
        // accelerometer bias column must be zero
        // serialized as accBias, gyroBias
        J_rDbiasij_bg_j(H11).setIdentity();
    }

    if (H12)
    {
        H12->resize(m_residualDim, bCdmDim);
        H12->setZero();
        J_rDbiasij_b_j(H12).setIdentity();
    }

    return error;
}

gtsam::Vector PreintegratedCDMCumulativeBias::computeError(
    const gtsam::Rot3& dR,
    const gtsam::Vector3& dcDot,
    const gtsam::Vector3& dc,
    const gtsam::Vector3& dha,
    const ImuBias& dImuBias,
    const CDMBias& dCdmBias,
    const ImuBias& imuBias_i,
    const CDMBias& cdmBias_i) const
{
    gtsam::Vector r;
    r.resize(m_residualDim);

    const gtsam::Vector db = getBiasCorrections(imuBias_i, cdmBias_i);

    const auto DRikTilde = m_deltaRij*gtsam::Rot3::Expmap(dbRij(&db));
    rDRij(&r) = gtsam::Rot3::Logmap(DRikTilde.inverse()*dR);
    rDcdotij(&r) = dcDot - (m_deltaCdotij + dbcdotij(&db));
    rDcij(&r) = dc - (m_deltaCij + dbcij(&db));
    rDhaij(&r) = dha - (m_deltaHaij + dbhaij(&db));
    rDbiasij(&r) << dImuBias.gyroscope(), dCdmBias.vector();

    return r;
}

gtsam::Vector PreintegratedCDMCumulativeBias::getBiasCorrections(const ImuBias& imuBias_i,
                                                                 const CDMBias& cdmBias_i) const
{
    const auto imuBiasIncr = imuBias_i - m_imuBiasHat;
    const auto cdmBiasIncr = cdmBias_i - m_cdmBiasHat;
    const gtsam::Vector3& dbg = imuBiasIncr.gyroscope();
    const gtsam::Vector3& dbf = cdmBiasIncr.netExternalForceInBase();
    const gtsam::Vector3& dbt = cdmBiasIncr.netExternalTorqueInBase();
    const gtsam::Vector3& dbG = cdmBiasIncr.comPositionInBase();

    const gtsam::Vector3 biasInducedOmega = m_delRdelBiasGyro*dbg;
    gtsam::Vector biasCorrection(12);
    dbRij(&biasCorrection) = biasInducedOmega;
    dbcdotij(&biasCorrection) = (m_delCdotdelBiasGyro*dbg) + (m_delCdotdelBiasNetForce*dbf);
    dbcij(&biasCorrection) = (m_delCdelBiasGyro*dbg) + (m_delCdelBiasNetForce*dbf);
    dbhaij(&biasCorrection) = (m_delHadelBiasGyro*dbg) + (m_delHadelBiasNetTorque*dbt) + (m_delHadelBiasCOMPosition*dbG);
    return biasCorrection;
}
