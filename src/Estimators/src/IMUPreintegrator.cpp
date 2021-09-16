/**
 * @file IMUPreintegrator.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <KinDynVIO/Estimators/IMUPreintegrator.h>

using namespace KinDynVIO::Estimators;

class ForsterIMUPreintegrator::Impl
{
public:
    void refreshOutput(const PreintegratorStatus& status);
    IMUPreintegratorInput input;
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> params; // requires to be
                                                                                // boost shared_ptr
                                                                                // for use in
                                                                                // constructor of
                                                                                // imuPreInt
    std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuPreInt;
    ForsterIMUPreintegratorOutput out;

    gtsam::Pose3 b_H_imu;
    bool initialized{false};
    bool extrinsicSet{false};
    double prevTime;

    // param placeholders
    double sigmaAcc{1e-2}, sigmaGyro{1e-3};
    double sigmaBiasAcc{1e-3}, sigmaBiasGyro{1e-3};
    double sigmaPosIntegration, errorBias{1e-5};
    Eigen::Matrix<double, 6, 1> biasInitial;
    Eigen::Vector3d gravity;
    Eigen::Matrix3d I3;
    Eigen::Matrix<double, 6, 6> I6;
};

ForsterIMUPreintegrator::ForsterIMUPreintegrator()
    : m_pimpl(std::make_unique<ForsterIMUPreintegrator::Impl>())
{
    m_pimpl->biasInitial.setZero();
    m_pimpl->gravity << 0, 0, -BipedalLocomotion::Math::StandardAccelerationOfGravitation;
    m_pimpl->I3.setIdentity();
    m_pimpl->I6.setIdentity();
    m_pimpl->b_H_imu.identity();
}

ForsterIMUPreintegrator::~ForsterIMUPreintegrator() = default;

bool ForsterIMUPreintegrator::initialize(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    const std::string printPrefix{"[ForsterIMUPreintegrator::initialize]"};
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        BipedalLocomotion::log()->error("{} The parameter handler has expired. "
                                        "Please check its scope.",
                                        printPrefix);
        return false;
    }

    if (!m_pimpl->extrinsicSet)
    {
        BipedalLocomotion::log()->error("{} Please call setBaseLinkIMUExtrinsics first",
                                        printPrefix);
        return false;
    }

    handle->getParameter("sigma_acc", m_pimpl->sigmaAcc);
    handle->getParameter("sigma_gyro", m_pimpl->sigmaGyro);
    handle->getParameter("sigma_b_acc", m_pimpl->sigmaBiasAcc);
    handle->getParameter("sigma_b_gyro", m_pimpl->sigmaBiasGyro);
    handle->getParameter("sigma_pos_integration", m_pimpl->sigmaPosIntegration);
    handle->getParameter("error_bias", m_pimpl->errorBias);
    handle->getParameter("initial_bias", m_pimpl->biasInitial);
    handle->getParameter("gravity", m_pimpl->gravity);

    m_pimpl->params
        = boost::make_shared<gtsam::PreintegratedCombinedMeasurements::Params>(m_pimpl->gravity);
    m_pimpl->params->setAccelerometerCovariance(m_pimpl->I3 * m_pimpl->sigmaAcc
                                                * m_pimpl->sigmaAcc);
    m_pimpl->params->setGyroscopeCovariance(m_pimpl->I3 * m_pimpl->sigmaGyro * m_pimpl->sigmaGyro);
    m_pimpl->params->setBiasAccCovariance(m_pimpl->I3 * m_pimpl->sigmaBiasAcc
                                          * m_pimpl->sigmaBiasAcc);
    m_pimpl->params->setBiasOmegaCovariance(m_pimpl->I3 * m_pimpl->sigmaBiasGyro
                                            * m_pimpl->sigmaBiasGyro);
    m_pimpl->params->setIntegrationCovariance(m_pimpl->I3 * m_pimpl->sigmaPosIntegration
                                              * m_pimpl->sigmaPosIntegration);
    m_pimpl->params->setBiasAccOmegaInt(m_pimpl->I6 * m_pimpl->errorBias);

    // force coriolis effect to false
    m_pimpl->params->setUse2ndOrderCoriolis(false);
    m_pimpl->params->setBodyPSensor(m_pimpl->b_H_imu);

    gtsam::imuBias::ConstantBias bias(m_pimpl->biasInitial);
    m_pimpl->imuPreInt
        = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(m_pimpl->params, bias);

    m_pimpl->initialized = true;
    return true;
}

bool ForsterIMUPreintegrator::setBaseLinkIMUExtrinsics(const gtsam::Pose3& b_H_imu)
{
    m_pimpl->b_H_imu = b_H_imu;
    m_pimpl->extrinsicSet = true;
    return true;
}

bool ForsterIMUPreintegrator::setInput(const IMUPreintegratorInput& input)
{
    std::string printPrefix{"[ForsterIMUPreintegrator::setInput]"};
    if (!m_pimpl->initialized)
    {
        BipedalLocomotion::log()->error("{} Please call initialize first", printPrefix);
        return false;
    }

    m_pimpl->input = input;
    return true;
}

bool ForsterIMUPreintegrator::advance()
{
    std::string printPrefix{"[ForsterIMUPreintegrator::advance]"};
    if (!m_pimpl->initialized)
    {
        BipedalLocomotion::log()->error("{} Please call initialize first", printPrefix);
        return false;
    }

    const auto& currTime = m_pimpl->input.ts;
    double dt{currTime - m_pimpl->prevTime};

    if (m_status == PreintegratorStatus::PREINTEGRATING)
    {
        const Eigen::Vector3d& acc = m_pimpl->input.linAcc;
        const Eigen::Vector3d& gyro = m_pimpl->input.gyro;

        m_pimpl->imuPreInt->integrateMeasurement(acc, gyro, dt);
    }

    m_pimpl->prevTime = m_pimpl->input.ts;
    m_pimpl->refreshOutput(m_status);

    return true;
}

bool ForsterIMUPreintegrator::isOutputValid() const
{
    return m_pimpl->initialized;
}

void ForsterIMUPreintegrator::resetIMUIntegration(const gtsam::imuBias::ConstantBias& bias)
{
    m_status = PreintegratorStatus::IDLE;
    m_pimpl->imuPreInt->resetIntegrationAndSetBias(bias);
}

void ForsterIMUPreintegrator::resetIMUIntegration()
{
    m_status = PreintegratorStatus::IDLE;
    m_pimpl->imuPreInt->resetIntegration();
}

const ForsterIMUPreintegrator::ForsterIMUPreintegratorOutput& ForsterIMUPreintegrator::getOutput() const
{
    m_pimpl->refreshOutput(m_status);
    return m_pimpl->out;
}

void ForsterIMUPreintegrator::Impl::refreshOutput(const PreintegratorStatus& status)
{
    out.status = status;
    if (status == PreintegratorStatus::PREINTEGRATED)
    {
        out.preInt = *imuPreInt;
    }
}

bool ForsterIMUPreintegrator::getPredictedState(const IMUState& currentState,
                                                IMUState& predictedState)
{
    gtsam::NavState navStatePred
        = m_pimpl->imuPreInt->predict(gtsam::NavState(currentState.pose(), currentState.v()),
                                      currentState.b());
    predictedState.setPose(navStatePred.pose());
    predictedState.setLinearVelocity(navStatePred.v());
    predictedState.setBias(currentState.b());
    return true;
}

void ForsterIMUPreintegrator::startPreintegration(const double& prevTimeStamp)
{
    m_status = PreintegratorStatus::PREINTEGRATING;
    m_pimpl->prevTime = prevTimeStamp;
}

void ForsterIMUPreintegrator::stopPreintegration()
{
    m_status = PreintegratorStatus::PREINTEGRATED;
}
