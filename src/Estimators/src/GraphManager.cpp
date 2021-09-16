/**
 * @file GraphManager.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <KinDynVIO/Estimators/GraphManager.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <unordered_map>
#include <map>

using namespace KinDynVIO::Estimators;
using gtsam::symbol_shorthand::B; // IMU bias
using gtsam::symbol_shorthand::L; // IMU bias
using gtsam::symbol_shorthand::V; // Base Velocity
using gtsam::symbol_shorthand::X; // Base Pose

using SmartFactor = gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>;

//     using gtsam::symbol_shorthand::C; // COM position
//     using gtsam::symbol_shorthand::D; // COM velocity
//     using gtsam::symbol_shorthand::H; // Angular momentum
//     using gtsam::symbol_shorthand::E; // Centroidal Dynamics Measurement Bias

class GraphManager::Impl
{
public:
    void resetTemporaryGraph();
    gtsam::FastList<gtsam::Key> findKeysBefore(double timestamp) const;

    long long int currentStateIdx{0}, previousStateIdx{0};
    double lag{150.0};
    gtsam::IncrementalFixedLagSmoother smoother;
//     gtsam::ISAM2 smoother;
    gtsam::ISAM2Params isam2Params;

    // temporary graph variables
    gtsam::NonlinearFactorGraph newFactors;
    gtsam::Values newValues, fullValues;

    gtsam::FixedLagSmoother::KeyTimestampMap newTimeStamps; // timestamp lookup by key
    std::map<double, std::size_t> keyIdxLookupByTimeStamp;  // Key Index based on timestamp
    std::map<std::size_t, double> timestampLookupByKeyIdx;
    std::unordered_map<int, SmartFactor::shared_ptr> smartArucoLdmk;
    gtsam::Cal3_S2 camK;
};

GraphManager::GraphManager()
    : m_pimpl(std::make_unique<GraphManager::Impl>())
{
    m_pimpl->isam2Params.relinearizeThreshold = 0.01; // Set the relin threshold to zero such that
                                                     // the batch estimate is recovered
    m_pimpl->isam2Params.relinearizeSkip = 1; // Relinearize every time
//     m_pimpl->isam2Params.factorization = gtsam::ISAM2Params::QR;
//     m_pimpl->isam2Params.optimizationParams = gtsam::ISAM2GaussNewtonParams();
    resetManager();
}

GraphManager::~GraphManager()
{
}

void GraphManager::setCameraIntrinsics(const std::vector<double>& K)
{
    // fx fy skew px(u0) py(v0)
    m_pimpl->camK = gtsam::Cal3_S2(K[0], K[4], 0.0, K[2], K[5]);
}

bool GraphManager::addBaseStatePriorAtCurrentKey(const double& timeStamp,
                                                 const gtsam::Pose3& basePose,
                                                 const gtsam::Vector3& baseVel,
                                                 const BaseStateDev& priorNoise,
                                                 const gtsam::imuBias::ConstantBias& bias)
{
    auto poseKey = X(m_pimpl->currentStateIdx);
    auto velKey = V(m_pimpl->currentStateIdx);
    auto imuBiasKey = B(m_pimpl->currentStateIdx);

    Eigen::Matrix<double, 6, 1> poseNoiseV;
    poseNoiseV << priorNoise.imuOrientation, priorNoise.imuPosition;
    auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas(poseNoiseV);

    Eigen::Vector3d velNoiseV;
    velNoiseV << priorNoise.imuLinearVelocity;
    auto velNoise = gtsam::noiseModel::Diagonal::Sigmas(velNoiseV);

    Eigen::Matrix<double, 6, 1> biasNoiseV;
    biasNoiseV << priorNoise.accelerometerBias, priorNoise.gyroscopeBias;
    auto biasNoise = gtsam::noiseModel::Diagonal::Sigmas(biasNoiseV);

    m_pimpl->newFactors.addPrior(poseKey, basePose, poseNoise);
    m_pimpl->newFactors.addPrior(velKey, baseVel, velNoise);
    m_pimpl->newFactors.addPrior(imuBiasKey, bias, biasNoise);

    m_pimpl->newValues.insert(poseKey, basePose);
    m_pimpl->newValues.insert(velKey, baseVel);
    m_pimpl->newValues.insert(imuBiasKey, bias);
    m_pimpl->fullValues.insert(poseKey, basePose);
    m_pimpl->fullValues.insert(velKey, baseVel);
    m_pimpl->fullValues.insert(imuBiasKey, bias);

    m_pimpl->newTimeStamps[poseKey] = timeStamp;
    m_pimpl->newTimeStamps[velKey] = timeStamp;
    m_pimpl->newTimeStamps[imuBiasKey] = timeStamp;

    m_pimpl->keyIdxLookupByTimeStamp[timeStamp] = m_pimpl->currentStateIdx;
    m_pimpl->timestampLookupByKeyIdx[m_pimpl->currentStateIdx] = timeStamp;

    return true;
}

void GraphManager::resetManager()
{
    m_pimpl->currentStateIdx = 0;
    m_pimpl->previousStateIdx = 0;
    m_pimpl->smoother = gtsam::IncrementalFixedLagSmoother(m_pimpl->lag, m_pimpl->isam2Params);
//     m_pimpl->smoother = gtsam::ISAM2(m_pimpl->isam2Params);

    m_pimpl->keyIdxLookupByTimeStamp.clear();
    m_pimpl->timestampLookupByKeyIdx.clear();
    m_pimpl->smartArucoLdmk.clear();
    m_pimpl->fullValues.clear();

    m_pimpl->resetTemporaryGraph();
}

void GraphManager::Impl::resetTemporaryGraph()
{
    newValues.clear();
    newFactors.resize(0);
    newTimeStamps.clear();
}

bool GraphManager::optimize()
{

    if (m_pimpl->currentStateIdx > 0)
    {
        // update smoother with only new content
        try
        {
            m_pimpl->smoother.update(m_pimpl->newFactors,
                                     m_pimpl->newValues,
                                     m_pimpl->newTimeStamps);
//             m_pimpl->smoother.update(m_pimpl->newFactors,
//                                      m_pimpl->newValues);
            m_pimpl->fullValues = m_pimpl->smoother.calculateEstimate();
            m_pimpl->smoother.getISAM2Result().print();
        } catch (gtsam::IndeterminantLinearSystemException& e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
        // clear temporary variables
        m_pimpl->resetTemporaryGraph();
    }

    return true;
}

void GraphManager::spawnNewState(const double& timeStamp)
{
    m_pimpl->previousStateIdx = m_pimpl->currentStateIdx;
    m_pimpl->currentStateIdx++;

    m_pimpl->keyIdxLookupByTimeStamp[timeStamp] = m_pimpl->currentStateIdx;
    m_pimpl->timestampLookupByKeyIdx[m_pimpl->currentStateIdx] = timeStamp;

    auto marginalizablekeys = m_pimpl->findKeysBefore(timeStamp - m_pimpl->lag);
//     if (marginalizablekeys.size() > 0)
//     {
//         m_pimpl->smoother.marginalizeLeaves(marginalizablekeys);
//     }
}

void GraphManager::processPreintegratedIMUMeasurements(
    const gtsam::PreintegratedCombinedMeasurements& preintIMU, gtsam::Pose3& predictedPose)
{
    auto prevPoseKey{X(m_pimpl->previousStateIdx)};
    auto prevVelKey{V(m_pimpl->previousStateIdx)};
    auto prevIMUBiasKey{B(m_pimpl->previousStateIdx)};

    auto currentPoseKey{X(m_pimpl->currentStateIdx)};
    auto currentVelKey{V(m_pimpl->currentStateIdx)};
    auto currentIMUBiasKey{B(m_pimpl->currentStateIdx)};

    auto imuFactor = gtsam::CombinedImuFactor(prevPoseKey,
                                              prevVelKey,
                                              currentPoseKey,
                                              currentVelKey,
                                              prevIMUBiasKey,
                                              currentIMUBiasKey,
                                              preintIMU);
    auto prevBias = m_pimpl->fullValues.at<gtsam::imuBias::ConstantBias>(prevIMUBiasKey);
    auto predictedState
        = preintIMU.predict(gtsam::NavState(m_pimpl->fullValues.at<gtsam::Pose3>(prevPoseKey),
                                            m_pimpl->fullValues.at<gtsam::Vector3>(prevVelKey)),
                            prevBias);

    m_pimpl->newFactors.add(imuFactor);
    m_pimpl->newValues.insert(currentPoseKey, predictedPose);
    m_pimpl->newValues.insert(currentVelKey, predictedState.v());
    m_pimpl->newValues.insert(currentIMUBiasKey, prevBias);
    m_pimpl->fullValues.insert(currentPoseKey, predictedPose);
    m_pimpl->fullValues.insert(currentVelKey, predictedState.v());
    m_pimpl->fullValues.insert(currentIMUBiasKey, prevBias);

    // get time stamp from the spawned state index
    m_pimpl->newTimeStamps[currentPoseKey]
        = m_pimpl->timestampLookupByKeyIdx.at(m_pimpl->currentStateIdx);
    m_pimpl->newTimeStamps[currentVelKey]
        = m_pimpl->timestampLookupByKeyIdx.at(m_pimpl->currentStateIdx);
    m_pimpl->newTimeStamps[currentIMUBiasKey]
        = m_pimpl->timestampLookupByKeyIdx.at(m_pimpl->currentStateIdx);
}

void GraphManager::processArucoKeyFrames(const ArucoKeyFrame& arucoKF,
                                         double pixelNoise)
{
    auto currentPoseKey{X(m_pimpl->currentStateIdx)};
    for (const auto& [id, marker] : arucoKF.detectorOut.markers)
    {
        auto uv = marker.corners[0];
        // if smart factor related to marker already exists
        // then update the measurement
        if (m_pimpl->smartArucoLdmk.find(id) != m_pimpl->smartArucoLdmk.end())
        {
            m_pimpl->smartArucoLdmk.at(id)->add(gtsam::Point2(uv.x, uv.y), currentPoseKey);
            continue;
        }

        // if smart factor does not exist, add it to the graph
        auto measNoise = gtsam::noiseModel::Isotropic::Sigma(2, pixelNoise);
        auto K = boost::make_shared<gtsam::Cal3_S2>(m_pimpl->camK);
        auto smartAruco = boost::make_shared<SmartFactor>(measNoise, K, arucoKF.b_H_cam);
        m_pimpl->smartArucoLdmk[id] = smartAruco;
        smartAruco->add(gtsam::Point2(uv.x, uv.y), currentPoseKey);

        m_pimpl->newFactors.add(smartAruco);
    }
}

const gtsam::IncrementalFixedLagSmoother& GraphManager::smoother() const
{
    return m_pimpl->smoother;
}

// const gtsam::ISAM2& GraphManager::smoother() const
// {
//     return m_pimpl->smoother;
// }

const gtsam::NonlinearFactorGraph& GraphManager::graph() const
{
    return m_pimpl->newFactors;
}

gtsam::Pose3 GraphManager::getEstimatedBasePose() const
{
    auto currentPoseKey{X(m_pimpl->currentStateIdx)};
    return m_pimpl->smoother.calculateEstimate<gtsam::Pose3>(currentPoseKey);
}

gtsam::imuBias::ConstantBias GraphManager::getEstimatedIMUBias() const
{
    auto currentIMUBiasKey{B(m_pimpl->currentStateIdx)};
    return m_pimpl->fullValues.at<gtsam::imuBias::ConstantBias>(currentIMUBiasKey);
}

gtsam::FastList<gtsam::Key> GraphManager::Impl::findKeysBefore(double timestamp) const
{
    gtsam::FastList<gtsam::Key> keys;
    auto end = keyIdxLookupByTimeStamp.lower_bound(timestamp);
    for (auto iter = keyIdxLookupByTimeStamp.begin(); iter != end; ++iter)
    {
        keys.emplace_back(X(iter->second));
        keys.emplace_back(V(iter->second));
        keys.emplace_back(B(iter->second));
    }

    return keys;
}
