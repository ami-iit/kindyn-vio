/**
 * @file States.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <KinDynVIO/Estimators/States.h>

using namespace KinDynVIO::Estimators;
using Bias = gtsam::imuBias::ConstantBias;

IMUState::IMUState()
    : m_pose(gtsam::Pose3())
    , m_linearVelocity(gtsam::Vector3())
    , m_bias(Bias())
{
}

IMUState::IMUState(const gtsam::Pose3& pose, const gtsam::Vector3& velocity, const Bias& bias)
{
    this->m_pose = pose;
    this->m_linearVelocity = velocity;
    this->m_bias = bias;
}

IMUState::IMUState(const IMUState& other)
{
    this->m_pose = other.m_pose;
    this->m_linearVelocity = other.m_linearVelocity;
    this->m_bias = other.m_bias;
}

void IMUState::setPose(const gtsam::Pose3& pose)
{
    this->m_pose = pose;
}

void IMUState::setLinearVelocity(const gtsam::Vector3& velocity)
{
    this->m_linearVelocity = velocity;
}

void IMUState::setBias(const Bias& bias)
{
    this->m_bias = bias;
}

const gtsam::Pose3& IMUState::pose() const
{
    return m_pose;
}

const gtsam::Vector3& IMUState::v() const
{
    return m_linearVelocity;
}

const Bias& IMUState::b() const
{
    return m_bias;
}

const gtsam::Vector3& IMUState::ba() const
{
    return b().accelerometer();
}

const gtsam::Vector3& IMUState::bg() const
{
    return b().gyroscope();
}

gtsam::Vector3 IMUState::p() const
{
    return pose().translation();
}

gtsam::Quaternion IMUState::quat() const
{
    return pose().rotation().toQuaternion();
}

gtsam::Vector3 IMUState::rpy() const
{
    return pose().rotation().rpy();
}

void IMUState::print() const
{
    BipedalLocomotion::log()->info("[IMUState] RPY: {} {} {} \n "
                                   "p: {} {} {} \n"
                                   "v: {} {} {} \n",
                                   "ba: {} {} {} \n "
                                   "bg {} {} {}",
                                   rpy()(0),
                                   rpy()(1),
                                   rpy()(2),
                                   p()(0),
                                   p()(1),
                                   p()(2),
                                   v()(0),
                                   v()(1),
                                   v()(2),
                                   ba()(0),
                                   ba()(1),
                                   ba()(2),
                                   bg()(0),
                                   bg()(1),
                                   bg()(2));
}
