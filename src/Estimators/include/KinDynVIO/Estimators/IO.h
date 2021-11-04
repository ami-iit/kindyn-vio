/**
 * @file IO.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNVIO_ESTIMATORS_IO_H
#define KINDYNVIO_ESTIMATORS_IO_H

#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimatorIO.h>
#include <BipedalLocomotion/Perception/Features/ArucoDetector.h>
#include <KinDynVIO/Perception/Features/DataTypes.h>
#include <gtsam/geometry/Pose3.h>
#include <unordered_map>
#include <Eigen/Dense>

namespace KinDynVIO
{
namespace Estimators
{

struct ProprioceptiveInput
{
    BipedalLocomotion::Estimators::FloatingBaseEstimators::Measurements meas;
    std::unordered_map<std::string, Eigen::Matrix<double, 6, 1> > contactWrenches;
    double ts;
    std::string lfContact, rfContact; // needed for InvEKF
};

using KinematicInertialFilterOutput = BipedalLocomotion::Estimators::FloatingBaseEstimators::Output;

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
struct CentroidalDynamicsPreintegratorOutput
{
    PreintegratedMeasurements preInt;
    PreintegratorStatus status;
};

struct ArucoWrapperInput
{
    gtsam::Pose3 Xhat; // predicted base link pose
    KinDynVIO::Perception::TimeStampedImg imgTs;
};

struct ArucoKeyFrame
{
    BipedalLocomotion::Perception::ArucoDetectorOutput detectorOut; // in camera frame
    bool isKeyFrame{false};
    std::unordered_map<int, gtsam::Pose3> markerMeasures;
    std::unordered_map<int, gtsam::Pose3> markerPriors; // new markers for which a prior needs to be generated
    gtsam::Pose3 b_H_cam;
};

struct CentroidalStateStdDev
{
    Eigen::Vector3d com;
    Eigen::Vector3d dcom;
    Eigen::Vector3d ha;
    // net wrench bias and com position bias in base frame
    Eigen::Vector3d forceBias, torqueBias, comPositionBias;
};

} // namespace Estimators
} // namespace KinDynVIO

#endif // KINDYNVIO_ESTIMATORS_IO_H
