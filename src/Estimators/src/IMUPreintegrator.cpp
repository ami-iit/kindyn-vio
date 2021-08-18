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
    IMUPreintegratorInput input;
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> params; // requires to be
                                                                                // boost shared_ptr
                                                                                // for use in
                                                                                // constructor of
                                                                                // imuPreInt
    std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuPreInt;
    gtsam::CombinedImuFactor imuFactor;

    bool initialized{false};
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
}

ForsterIMUPreintegrator::~ForsterIMUPreintegrator()
{
}

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

    gtsam::imuBias::ConstantBias bias(m_pimpl->biasInitial);
    m_pimpl->imuPreInt
        = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(m_pimpl->params, bias);

    m_pimpl->initialized = true;
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
    const auto& currTime = m_pimpl->input.ts;
    const Eigen::Vector3d& acc = m_pimpl->input.linAcc;
    const Eigen::Vector3d& gyro = m_pimpl->input.gyro;
    double dt{currTime - m_pimpl->prevTime};

    m_pimpl->imuPreInt->integrateMeasurement(acc, gyro, dt);
    m_pimpl->prevTime = m_pimpl->input.ts;
    return true;
}

bool ForsterIMUPreintegrator::isOutputValid() const
{
    return m_pimpl->initialized;
}

void ForsterIMUPreintegrator::resetIMUIntegration(const gtsam::imuBias::ConstantBias& bias)
{
    m_pimpl->imuPreInt->resetIntegrationAndSetBias(bias);
}

void ForsterIMUPreintegrator::resetIMUIntegration()
{
    m_pimpl->imuPreInt->resetIntegration();
}

const gtsam::CombinedImuFactor& ForsterIMUPreintegrator::getOutput() const
{
    const auto& Xi = m_pimpl->input.posei;
    const auto& Xj = m_pimpl->input.posei;
    const auto& vi = m_pimpl->input.vi;
    const auto& vj = m_pimpl->input.vj;
    const auto& bi = m_pimpl->input.bi;
    const auto& bj = m_pimpl->input.bj;

    m_pimpl->imuFactor = gtsam::CombinedImuFactor(Xi, vi, Xj, vj, bi, bj, *m_pimpl->imuPreInt);
    return m_pimpl->imuFactor;
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
