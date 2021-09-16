/**
 * @file ArucoWrapper.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNVIO_ESTIMATORS_ARUCO_WRAPPER_H
#define KINDYNVIO_ESTIMATORS_ARUCO_WRAPPER_H

#include <KinDynVIO/Estimators/IO.h>
#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/Perception/Features/ArucoDetector.h>
#include <gtsam/geometry/BearingRange.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>

#include <memory>


namespace KinDynVIO
{
namespace Estimators
{

using BetweenMeasure = gtsam::Pose3;
using Prior = gtsam::Pose3;
using NoiseSigmasV = gtsam::Vector6;

class ArucoWrapper : public BipedalLocomotion::System::Advanceable<ArucoWrapperInput, ArucoKeyFrame>
{
public:
    ArucoWrapper();
    ~ArucoWrapper();

    ArucoWrapper(const ArucoWrapper& other) = delete; // copy constructor
    ArucoWrapper& operator=(const ArucoWrapper& other) = delete;
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
