/**
 * @file GraphManager.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNVIO_ESTIMATORS_GRAPH_MANAGER_H
#define KINDYNVIO_ESTIMATORS_GRAPH_MANAGER_H

#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimatorParams.h>

#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>

#include <KinDynVIO/Estimators/IO.h>

#include <memory>
namespace KinDynVIO
{
namespace Estimators
{
using BaseStateDev = BipedalLocomotion::Estimators::FloatingBaseEstimators::StateStdDev;

class GraphManager
{
public:
    GraphManager();
    ~GraphManager();
    GraphManager(const GraphManager& other) = delete;
    GraphManager& operator=(const GraphManager& other) = delete;
    GraphManager(GraphManager &&) = default;
    GraphManager& operator=(GraphManager &&) = default;

    void setCameraIntrinsics(const std::vector<double>& K);
    bool addBaseStatePriorAtCurrentKey(const double& timeStamp,
                                       const gtsam::Pose3& basePose,
                                       const gtsam::Vector3& baseVel,
                                       const BaseStateDev& priorNoise,
                                       const gtsam::imuBias::ConstantBias& bias = gtsam::imuBias::ConstantBias());
    void spawnNewState(const double& timeStamp);
    void processPreintegratedIMUMeasurements(const gtsam::PreintegratedCombinedMeasurements& preintIMU, gtsam::Pose3& predictedPose);
    void processArucoKeyFrames(const ArucoKeyFrame& arucoKF, double pixelNoise = 1.0);

    bool optimize();
    void resetManager();

    gtsam::Pose3 getEstimatedBasePose() const;
    gtsam::imuBias::ConstantBias getEstimatedIMUBias() const;

    const gtsam::IncrementalFixedLagSmoother& smoother() const;
//     const gtsam::ISAM2& smoother() const;
    const gtsam::NonlinearFactorGraph& graph() const;

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace Estimators
} // namespace KinDynVIO

#endif
