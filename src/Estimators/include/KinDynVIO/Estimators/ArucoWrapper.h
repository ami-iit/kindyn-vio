/**
 * @file ArucoWrapper.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNVIO_ESTIMATORS_ARUCO_WRAPPER_H
#define KINDYNVIO_ESTIMATORS_ARUCO_WRAPPER_H

#include <KinDynVIO/Perception/Features/DataTypes.h>
#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/Perception/Features/ArucoDetector.h>
#include <gtsam/geometry/BearingRange.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>

#include <memory>

#define USE_ARUCO_FULL_POSE 0

namespace KinDynVIO
{
namespace Estimators
{

#if USE_ARUCO_FULL_POSE == 1
using BetweenMeasure = gtsam::Pose3;
using Prior = gtsam::Pose3;
using NoiseSigmasV = gtsam::Vector6;
#elif USE_ARUCO_FULL_POSE == 0
using BetweenMeasure = gtsam::Point3;
using Prior = gtsam::Point3;
using NoiseSigmasV = gtsam::Vector3;
#endif

struct ArucoWrapperInput
{
    gtsam::Pose3 Xhat; // predicted base link pose
    KinDynVIO::Perception::TimeStampedImg imgTs;
};

struct ArucoKeyFrame
{
    BipedalLocomotion::Perception::ArucoDetectorOutput detectorOut;
    bool isKeyFrame{false};
    std::unordered_map<int, BetweenMeasure> markerMeasures;
    std::unordered_map<int, Prior> markerPriors; // new markers for which a prior needs to be generated
};

class ArucoWrapper : public BipedalLocomotion::System::Advanceable<ArucoWrapperInput, ArucoKeyFrame>
{
public:
    ArucoWrapper();
    ~ArucoWrapper();

    ArucoWrapper(const ArucoWrapper& other) = default; // copy constructor
    ArucoWrapper& operator=(const ArucoWrapper& other) = default;
    ArucoWrapper(ArucoWrapper&& other) = default; // move constructor
    ArucoWrapper& operator=(ArucoWrapper&& other) noexcept = default;

    bool setBaseLinkCameraExtrinsics(const gtsam::Pose3& b_H_cam);
    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler) final;
    bool setInput(const ArucoWrapperInput& input) final;
    bool advance() final;
    const ArucoKeyFrame& getOutput() const final;
    bool isOutputValid() const final;

    NoiseSigmasV landmarkPriorNoiseModel() const;
    NoiseSigmasV landmarkMeasurementNoiseModel() const;
private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace KinDynVIO
} // namespace Estimators


#endif // KINDYNVIO_ESTIMATORS_ARUCO_FACTOR_GENERATOR_H
