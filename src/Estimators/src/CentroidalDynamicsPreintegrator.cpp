/**
 * @file CentroidalDynamicsPreintegrator.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynVIO/Estimators/CentroidalDynamicsPreintegrator.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <iDynTree/Core/EigenHelpers.h>

using namespace KinDynVIO::Estimators;
using namespace KinDynVIO::Factors;

class CentroidalDynamicsCumulativeBiasPreintegrator::Impl
{
public:
    void refreshOutput(const PreintegratorStatus& status);
    ProprioceptiveInput input;
    std::shared_ptr<PreintegrationCentroidalDynamicsParams> params;
    std::shared_ptr<PreintegratedCDMCumulativeBias> cdmPreInt;
    CentroidalDynamicsCumulativeBiasPreintegrator::Output out;

    bool initialized{false};
    double prevTime;

    // param placeholders
    double sigmaGyro{1e-3}, sigmaExtForce{2.0};
    double sigmaExtTorque{2.0}, sigmaCOMPosition{0.01};
    double sigmaBiasGyro{1e-3}, sigmaBiasExtForce{1e-3};
    double sigmaBiasExtTorque{1e-3}, sigmaBiasCOMPosition{1e-3};
    std::string baseLinkName, baseImuName;
    gtsam::Pose3 bHImu;

    gtsam::Vector3 gravity;
    Eigen::Matrix<double, 6, 1> imuBiasInitial;
    Eigen::Matrix<double, 9, 1> cdmBiasInitial;
    Eigen::Matrix3d I3;
    Eigen::Matrix4d basePose;
    Eigen::Matrix<double, 6, 1> baseTwist;
};

CentroidalDynamicsCumulativeBiasPreintegrator::CentroidalDynamicsCumulativeBiasPreintegrator()
{
    m_kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    m_pimpl = std::make_unique<CentroidalDynamicsCumulativeBiasPreintegrator::Impl>();
    m_pimpl->imuBiasInitial.setZero();
    m_pimpl->cdmBiasInitial.setZero();
    m_pimpl->I3.setIdentity();
    m_pimpl->bHImu.identity();
    m_pimpl->basePose.setIdentity();
    m_pimpl->baseTwist.setZero();
    m_pimpl->gravity << 0., 0., -BipedalLocomotion::Math::StandardAccelerationOfGravitation;
}

CentroidalDynamicsCumulativeBiasPreintegrator::~CentroidalDynamicsCumulativeBiasPreintegrator() = default;

bool CentroidalDynamicsCumulativeBiasPreintegrator::initialize(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
    const iDynTree::Model& model)
{
    const std::string printPrefix{"[CentroidalDynamicsCumulativeBiasPreintegrator::initialize]"};
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        BipedalLocomotion::log()->error("{} The parameter handler has expired. "
                                        "Please check its scope.",
                                        printPrefix);
        return false;
    }

    if (!m_kinDyn->loadRobotModel(model))
    {
        BipedalLocomotion::log()->error("{} Could not load robot model.",
                                        printPrefix);
        return false;
    }

    if (!m_kinDyn->isValid())
    {
        BipedalLocomotion::log()->error("{} KinDynComputations object is not valid.",
                                        printPrefix);
        return false;
    }

    handle->getParameter("sigma_gyro", m_pimpl->sigmaGyro);
    handle->getParameter("sigma_ext_force", m_pimpl->sigmaExtForce);
    handle->getParameter("sigma_ext_torque", m_pimpl->sigmaExtTorque);
    handle->getParameter("sigma_com_pos", m_pimpl->sigmaCOMPosition);

    handle->getParameter("sigma_b_gyro", m_pimpl->sigmaBiasGyro);
    handle->getParameter("sigma_b_ext_force", m_pimpl->sigmaBiasExtForce);
    handle->getParameter("sigma_b_ext_torque", m_pimpl->sigmaBiasExtTorque);
    handle->getParameter("sigma_b_com_pos", m_pimpl->sigmaBiasCOMPosition);

    handle->getParameter("initial_imu_bias", m_pimpl->imuBiasInitial);
    handle->getParameter("initial_cdm_bias", m_pimpl->cdmBiasInitial);
    handle->getParameter("gravity", m_pimpl->gravity);

    if (!handle->getParameter("base_link_name", m_pimpl->baseLinkName))
    {
        BipedalLocomotion::log()->error("{} Required parameter \"base_link_name\"."
                                         "not found in configuration.", printPrefix);
        return false;
    }

    if (!handle->getParameter("base_link_imu_name", m_pimpl->baseImuName))
    {
        BipedalLocomotion::log()->error("{} Required parameter \"base_link_imu_name\"."
                                         "not found in configuration.", printPrefix);
        return false;
    }

    if (!m_kinDyn->model().isFrameNameUsed(m_pimpl->baseLinkName))
    {
        BipedalLocomotion::log()->error("{} Specified \"base_link_name\"."
                                         "not found in loaded model.", printPrefix);
        return false;
    }

    if (!m_kinDyn->model().isFrameNameUsed(m_pimpl->baseImuName))
    {
        BipedalLocomotion::log()->error("{} Specified \"base_link_imu_name\"."
                                         "not found in loaded model.", printPrefix);
        return false;
    }

    auto imuIdx = m_kinDyn->model().getFrameIndex(m_pimpl->baseImuName);
    auto baseIdx = m_kinDyn->model().getLinkIndex(m_pimpl->baseLinkName);
    bool isBaseIMU = (imuIdx!= iDynTree::FRAME_INVALID_INDEX) &&
                      (baseIdx!= iDynTree::FRAME_INVALID_INDEX) &&
                     (m_kinDyn->model().getFrameLink(imuIdx) == baseIdx);

    if (!isBaseIMU)
    {
        BipedalLocomotion::log()->error("{} Specified \"base_link_imu_name\"."
                                         "is not rigidly attached to specified \"base_link_name\".", printPrefix);
        return false;
    }

    m_kinDyn->setFloatingBase(m_pimpl->baseLinkName);
    auto B_H_IMU_idyn = m_kinDyn->getRelativeTransform(baseIdx, imuIdx);
    m_pimpl->bHImu = gtsam::Pose3(iDynTree::toEigen(B_H_IMU_idyn.asHomogeneousTransform()));

    m_pimpl->params
        = std::make_shared<PreintegrationCentroidalDynamicsParams>();

    m_pimpl->params->setGyroscopeCovariance(m_pimpl->I3 * m_pimpl->sigmaGyro * m_pimpl->sigmaGyro);
    m_pimpl->params->setContactForceCovariance(m_pimpl->I3 * m_pimpl->sigmaExtForce * m_pimpl->sigmaExtForce);
    m_pimpl->params->setContactTorqueCovariance(m_pimpl->I3 * m_pimpl->sigmaExtTorque * m_pimpl->sigmaExtTorque);
    m_pimpl->params->setCOMPositionCovariance(m_pimpl->I3 * m_pimpl->sigmaCOMPosition * m_pimpl->sigmaCOMPosition);

    m_pimpl->params->setGyroscopeBiasCovariance(m_pimpl->I3 * m_pimpl->sigmaBiasGyro * m_pimpl->sigmaBiasGyro);
    m_pimpl->params->setContactForceBiasCovariance(m_pimpl->I3 * m_pimpl->sigmaBiasExtForce * m_pimpl->sigmaBiasExtForce);
    m_pimpl->params->setContactTorqueBiasCovariance(m_pimpl->I3 * m_pimpl->sigmaBiasExtTorque * m_pimpl->sigmaBiasExtTorque);
    m_pimpl->params->setCOMPositionBiasCovariance(m_pimpl->I3 * m_pimpl->sigmaBiasCOMPosition * m_pimpl->sigmaBiasCOMPosition);

    m_pimpl->params->setBaseImuName(m_pimpl->baseImuName);
    m_pimpl->params->setBaseLinkName(m_pimpl->baseLinkName);
    m_pimpl->params->setBaseHIMU(m_pimpl->bHImu);

    gtsam::CDMBiasCumulative cdmBias(m_pimpl->cdmBiasInitial);
    gtsam::imuBias::ConstantBias imuBias(m_pimpl->imuBiasInitial);
    m_pimpl->cdmPreInt
        = std::make_shared<PreintegratedCDMCumulativeBias>(m_pimpl->params, imuBias, cdmBias);
    m_pimpl->cdmPreInt->setGravity(m_pimpl->gravity);

    m_pimpl->initialized = true;
    return true;
}


bool CentroidalDynamicsCumulativeBiasPreintegrator::setInput(const ProprioceptiveInput& input)
{
    std::string printPrefix{"[CentroidalDynamicsCumulativeBiasPreintegrator::setInput]"};
    if (!m_pimpl->initialized)
    {
        BipedalLocomotion::log()->error("{} Please call initialize first", printPrefix);
        return false;
    }

    if (input.meas.encoders.size() != m_kinDyn->getNrOfDegreesOfFreedom() ||
        input.meas.encodersSpeed.size() != m_kinDyn->getNrOfDegreesOfFreedom())
    {
        BipedalLocomotion::log()->error("{} Encoder measurement size does not match"
                                        " KinDynComputations model DOFs", printPrefix);
        return false;
    }

    m_pimpl->input = input;
    return true;
}

bool CentroidalDynamicsCumulativeBiasPreintegrator::advance()
{
    std::string printPrefix{"[CentroidalDynamicsCumulativeBiasPreintegrator::advance]"};
    if (!m_pimpl->initialized)
    {
        BipedalLocomotion::log()->error("{} Please call initialize first.", printPrefix);
        return false;
    }

    const auto& currTime = m_pimpl->input.ts;
    double dt{currTime - m_pimpl->prevTime};

    if (m_status == PreintegratorStatus::PREINTEGRATING)
    {
        // setting base pose and twist to identity and zero respectively
        if (!m_kinDyn->setRobotState(m_pimpl->basePose,
                                     m_pimpl->input.meas.encoders,
                                     m_pimpl->baseTwist,
                                     m_pimpl->input.meas.encodersSpeed,
                                     m_pimpl->cdmPreInt->gravity()))
        {
            BipedalLocomotion::log()->error("{} Could not set KinDynComputations robot state.", printPrefix);
            return false;
        }

        const Eigen::Vector3d& gyro = m_pimpl->input.meas.gyro;
        LocalContactWrenchesMap extWrenchMap;
        for (const auto& [name, wrench] : m_pimpl->input.contactWrenches)
        {
            auto id  = m_kinDyn->getFrameIndex(name);
            if (id != iDynTree::FRAME_INVALID_INDEX)
            {
                extWrenchMap[id] = wrench;
            }
        }

        if (!m_pimpl->cdmPreInt->update(m_kinDyn, gyro, extWrenchMap, dt))
        {
            return false;
        }
    }

    m_pimpl->prevTime = m_pimpl->input.ts;
    m_pimpl->refreshOutput(m_status);

    return true;
}

bool CentroidalDynamicsCumulativeBiasPreintegrator::isOutputValid() const
{
    return m_pimpl->initialized;
}

void CentroidalDynamicsCumulativeBiasPreintegrator::resetIntegration(const gtsam::imuBias::ConstantBias& imuBias,
                                                                     const gtsam::CDMBiasCumulative& cdmBias)
{
    m_status = PreintegratorStatus::IDLE;
    m_pimpl->cdmPreInt->resetIntegrationAndSetBias(imuBias, cdmBias);
    // just to be sure reset again
    m_pimpl->cdmPreInt->resetIntegration();
}

void CentroidalDynamicsCumulativeBiasPreintegrator::resetIntegration()
{
    m_status = PreintegratorStatus::IDLE;
    m_pimpl->cdmPreInt->resetIntegration();
}

const CentroidalDynamicsCumulativeBiasPreintegrator::Output& CentroidalDynamicsCumulativeBiasPreintegrator::getOutput() const
{
    m_pimpl->refreshOutput(m_status);
    return m_pimpl->out;
}

void CentroidalDynamicsCumulativeBiasPreintegrator::Impl::refreshOutput(const PreintegratorStatus& status)
{
    out.status = status;
    if (status == PreintegratorStatus::PREINTEGRATED)
    {
        out.preInt = *cdmPreInt;
    }
}

void CentroidalDynamicsCumulativeBiasPreintegrator::startPreintegration(const double& prevTimeStamp)
{
    m_status = PreintegratorStatus::PREINTEGRATING;
    m_pimpl->prevTime = prevTimeStamp;
}

void CentroidalDynamicsCumulativeBiasPreintegrator::stopPreintegration()
{
    m_status = PreintegratorStatus::PREINTEGRATED;
}
