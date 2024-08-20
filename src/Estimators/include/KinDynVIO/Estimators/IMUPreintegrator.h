/**
 * @file IMUPreintegrator.h
 * @authors Prashanth Ramadoss
 * @copyright Fondazione Istituto Italiano di Tecnologia (IIT). SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef KINDYNVIO_ESTIMATORS_IMU_PREINTEGRATOR_H
#define KINDYNVIO_ESTIMATORS_IMU_PREINTEGRATOR_H
#include <BipedalLocomotion/System/Advanceable.h>
#include <KinDynVIO/Estimators/States.h>
// gtsam includes
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/inference/Key.h>

#include <Eigen/Dense>
#include <memory>

namespace KinDynVIO
{
namespace Estimators
{

enum class PreintegratorStatus
{
    IDLE,
    PREINTEGRATING,
    PREINTEGRATED
};

struct IMUPreintegratorInput
{
    double ts;
    Eigen::Vector3d linAcc; // m per second per second
    Eigen::Vector3d gyro; // radians per second
};

template <typename PreintegratedMeasurements>
struct IMUPreintegratorOutput
{
    PreintegratedMeasurements preInt;
    PreintegratorStatus status;
};

template <typename PreintegratedMeasurements>
class IMUPreintegrator : public BipedalLocomotion::System::Advanceable<IMUPreintegratorInput, IMUPreintegratorOutput<PreintegratedMeasurements> >
{
protected:
    PreintegratorStatus m_status{PreintegratorStatus::IDLE};
public:
    virtual ~IMUPreintegrator() = default;
    virtual bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
    {
        return true;
    }

    virtual bool setBaseLinkIMUExtrinsics(const gtsam::Pose3& b_H_imu) = 0;
    virtual bool setInput(const IMUPreintegratorInput& input) = 0;

    virtual bool advance() = 0;
    virtual const IMUPreintegratorOutput<PreintegratedMeasurements>& getOutput() const = 0;
    virtual bool isOutputValid() const = 0;

    inline void startPreintegration()
    {
        m_status = PreintegratorStatus::PREINTEGRATING;
    }

    inline void stopPreintegration()
    {
        m_status = PreintegratorStatus::PREINTEGRATED;
    }

    virtual bool getPredictedState(const IMUState& currentState,
                                   IMUState& predictedState) = 0;
    virtual void resetIMUIntegration() = 0;
};



/*
 * An advanceable block for IMU measurements preintegration
 *
 * This class uses the GTSAM's PreintegratedMeasurements class,
 * which can be used in two ways depending on how GTSAM is compiled,
 * 1. Manifold preintegration (described in [4])
 * 2. Tangent preintegration (check gtsam/doc/ImuFactor.pdf) [default since GTSAM 4.0]
 * Please note the documentation in https://gtsam.org/notes/IMU-Factor.html
 * We have decided to call this class ForsterIMUPreintegrator in general.
 *
 * If you are using the factor, please cite:
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, F. Dellaert, Eliminating
 * conditionally independent sets in factor graphs: a unifying perspective based
 * on smart factors, Int. Conf. on Robotics and Automation (ICRA), 2014.
 *
 * REFERENCES:
 * [1] G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups",
 *     Volume 2, 2008.
 * [2] T. Lupton and S.Sukkarieh, "Visual-Inertial-Aided Navigation for
 *     High-Dynamic Motion in Built Environments Without Initial Conditions",
 *     TRO, 28(1):61-76, 2012.
 * [3] L. Carlone, S. Williams, R. Roberts, "Preintegrated IMU factor:
 *     Computation of the Jacobian Matrices", Tech. Report, 2013.
 * [4] C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, IMU Preintegration on
 *     Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation,
 *     Robotics: Science and Systems (RSS), 2015.
 */
class ForsterIMUPreintegrator : public IMUPreintegrator<gtsam::PreintegratedCombinedMeasurements>
{
public:
    ForsterIMUPreintegrator();
    virtual ~ForsterIMUPreintegrator();

    using ForsterIMUPreintegratorOutput = IMUPreintegratorOutput<gtsam::PreintegratedCombinedMeasurements>;
    /**
     * Initialize the ForsterIMUPreintegrator.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * | Parameter Name        |        Type        |                     Description                                         | Mandatory |
     * |:---------------------:|:------------------:|:-----------------------------------------------------------------------:|:---------:|
     * |       `sigma_acc`     |      `double`      | Isotropic standard deviation of accelerometer noise in continuous time. |    No     |
     * |       `sigma_gyro`    |      `double`      | Isotropic standard deviation of gyroscope noise in continuous time.     |    No     |
     * |       `sigma_b_acc`   |      `double`      | Isotropic standard deviation of accelerometer bias in continuous time.  |    No     |
     * |       `sigma_b_gyro`  |      `double`      | Isotropic standard deviation of gyroscope bias in continuous time.      |    No     |
     * |`sigma_pos_integration`|      `double`      | Isotropic standard deviation for position integration from velocities   |    No     |
     * |       `error_bias`    |      `double`      |             Error for IMU bias in preintegration.                       |    No     |
     * |       `initial_bias`  |`vector of double`  |                      Initial bias values.                               |    No     |
     * |       `gravity`       |`vector of double`  |           Acceleration due to gravity in the inertial frame.            |    No     |
     */
    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler) final;
    bool setBaseLinkIMUExtrinsics(const gtsam::Pose3& b_H_imu) final;

    bool setInput(const IMUPreintegratorInput& input) final;
    virtual bool advance() final;

    // the following functions are all intended to be called
    // only after multiple calls to advance(),
    // relevant to the intermediate integration steps
    // between states at i-th and j-th timestamp, has been called
    // currently no internal check is available for this
    // and the user needs to be careful about these function calls
    // depending on the satisfaction of some conditional statements

    const ForsterIMUPreintegratorOutput& getOutput() const final;
    bool isOutputValid() const final;

    // if setBaseLinkIMUExtrinsics() is properly used
    // this actually returns the base state, otherwise returns IMU state
    virtual bool getPredictedState(const IMUState& currentState,
                                   IMUState& predictedState) final;
    virtual void resetIMUIntegration() final;
    virtual void resetIMUIntegration(const gtsam::imuBias::ConstantBias& bias);

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace Estimators
} // namespace KinDynVIO
#endif // KINDYNVIO_ESTIMATORS_SMOOTHER_H
