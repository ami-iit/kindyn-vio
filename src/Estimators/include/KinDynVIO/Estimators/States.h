/**
 * @file States.h
 * @authors Prashanth Ramadoss
 * @copyright Fondazione Istituto Italiano di Tecnologia (IIT). SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef KINDYNVIO_ESTIMATORS_STATES_H
#define KINDYNVIO_ESTIMATORS_STATES_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

namespace KinDynVIO
{
namespace Estimators
{

class IMUState
{
public:
    IMUState();
    IMUState(const IMUState& other);
    IMUState(const gtsam::Pose3& pose,
             const gtsam::Vector3& velocity,
             const gtsam::imuBias::ConstantBias& bias);

    void setPose(const gtsam::Pose3& pose);
    void setLinearVelocity(const gtsam::Vector3& velocity);
    void setBias(const gtsam::imuBias::ConstantBias& bias);

    const gtsam::Pose3& pose() const;
    gtsam::Vector3 p() const;
    gtsam::Quaternion quat() const;
    gtsam::Vector3 rpy() const;
    const gtsam::Vector3& v() const;
    const gtsam::imuBias::ConstantBias& b() const;
    const gtsam::Vector3& ba() const;
    const gtsam::Vector3& bg() const;

    void print() const;

private:
    gtsam::Pose3 m_pose;                 // Pose of IMU wrt inertial
    gtsam::Vector3 m_linearVelocity;     // mixed trivialized linear velocity of IMU wrt inertial
    gtsam::imuBias::ConstantBias m_bias; // IMU Bias
};

} // namespace Estimators
} // namespace KinDynVIO

#endif // KINDYNVIO_ESTIMATORS_KINDYNVIO_STATES_H
