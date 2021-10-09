/**
 * @file GraphManager.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <KinDynVIO/Estimators/GraphManager.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <map>
#include <unordered_map>

using namespace KinDynVIO::Estimators;
using namespace KinDynVIO::Perception;
using gtsam::symbol_shorthand::B; // IMU bias
using gtsam::symbol_shorthand::L; // IMU bias
using gtsam::symbol_shorthand::V; // Base Velocity
using gtsam::symbol_shorthand::X; // Base Pose

using SmartFactor = gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2>;
using Slot = long int;
using LandmarkID = long long int;
using SmartFactorMap = std::unordered_map<long long int, SmartFactor::shared_ptr>;
using SmartFactorMapExtended = gtsam::FastMap<long long int, std::pair<SmartFactor::shared_ptr, Slot> >;

//     using gtsam::symbol_shorthand::C; // COM position
//     using gtsam::symbol_shorthand::D; // COM velocity
//     using gtsam::symbol_shorthand::H; // Angular momentum
//     using gtsam::symbol_shorthand::E; // Centroidal Dynamics Measurement Bias

class GraphManager::Impl
{
public:
    void resetTemporaryGraph();
    gtsam::FastList<gtsam::Key> findAndMoveKeysBefore(double timestamp);
    void addPointFeature(const long long int& id,
                         cv::Point2f uv,
                         const double& pixelNoise);
    void updatePointFeature(const long long int& id,
                            cv::Point2f uv);
    void setupSmartFactorParameters();
    void setupSmartFactorNoiseModel(const double& pixelNoise);
    bool addOrMarkRemovableSmartFactorsToFactorGraph();
    bool updateSmartFactorSlotsFromISAMResult();
    bool factorHasAMarginalizableKey(boost::shared_ptr<gtsam::NonlinearFactor> factor,
                                     const gtsam::FastList<gtsam::Key>& marginalizableKeys);

    long long int currentStateIdx{0}, previousStateIdx{0};
    double lag{6.0};
    gtsam::IncrementalFixedLagSmoother smoother;
    gtsam::ISAM2Params isam2Params;

    // temporary graph variables
    gtsam::NonlinearFactorGraph newFactors;
    gtsam::Values newValues, fullValues;
    gtsam::FactorIndices factorsToRemove;

    gtsam::FixedLagSmoother::KeyTimestampMap newTimeStamps, fullTimeStamps; // timestamp lookup by key
    std::map<double, std::size_t> keyIdxLookupByTimeStamp; // Key Index based on timestamp
    std::map<std::size_t, double> timestampLookupByKeyIdx;

    gtsam::SmartProjectionParams smartParams;
    gtsam::SharedNoiseModel smartFactorNoiseModel;
    SmartFactorMap smartPointLdmks;
    SmartFactorMapExtended existingSmartFactorsMap;
    std::vector<LandmarkID> newlyAddedLdmkIds;

    gtsam::Cal3_S2::shared_ptr camK;
    gtsam::Pose3 b_H_cam;


    gtsam::Pose3 estimatedBasePose;
    gtsam::Vector3 estimatedBaseLinVel;
    gtsam::imuBias::ConstantBias estimatedIMUBias;
};

GraphManager::GraphManager()
    : m_pimpl(std::make_unique<GraphManager::Impl>())
{
    m_pimpl->isam2Params.relinearizeThreshold = 0.01; // Set the relin threshold to zero such that
                                                      // the batch estimate is recovered
    m_pimpl->isam2Params.relinearizeSkip = 1; // Relinearize every time
    m_pimpl->isam2Params.findUnusedFactorSlots = true;
    m_pimpl->isam2Params.factorization = gtsam::ISAM2Params::Factorization::QR;
    resetManager();
    m_pimpl->setupSmartFactorParameters();
}

GraphManager::~GraphManager()
{
}

void GraphManager::setCameraIntrinsics(const std::vector<double>& K)
{
    // fx fy skew px(u0) py(v0)
    m_pimpl->camK = boost::make_shared<gtsam::Cal3_S2>(K[0], K[4], 0.0, K[2], K[5]);
}

void GraphManager::setBaseCameraExtrinsics(const gtsam::Pose3& b_H_cam)
{
    m_pimpl->b_H_cam = b_H_cam;
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

    m_pimpl->newTimeStamps[poseKey] = m_pimpl->newTimeStamps[velKey]
        = m_pimpl->newTimeStamps[imuBiasKey] = timeStamp;
    m_pimpl->fullTimeStamps[poseKey] = m_pimpl->fullTimeStamps[velKey]
        = m_pimpl->fullTimeStamps[imuBiasKey] = timeStamp;

    m_pimpl->keyIdxLookupByTimeStamp[timeStamp] = m_pimpl->currentStateIdx;
    m_pimpl->timestampLookupByKeyIdx[m_pimpl->currentStateIdx] = timeStamp;

    return true;
}

void GraphManager::Impl::setupSmartFactorParameters()
{
    smartParams.setDegeneracyMode(gtsam::DegeneracyMode::ZERO_ON_DEGENERACY);
    smartParams.setLinearizationMode(gtsam::LinearizationMode::JACOBIAN_SVD);
    smartParams.setEnableEPI(false); // set to true, refines triangulation using LM iterations
    smartParams.setLandmarkDistanceThreshold(20.0); // max distance to triangulate in meters
    smartParams.setRankTolerance(1.0);
    smartParams.setRetriangulationThreshold(1e-3);
    smartParams.setDynamicOutlierRejectionThreshold(8.0); // max acceptable reprojection error
    smartParams.throwCheirality = false;
    smartParams.verboseCheirality = false;
}

void GraphManager::Impl::setupSmartFactorNoiseModel(const double& pixelNoise)
{
    smartFactorNoiseModel = gtsam::noiseModel::Isotropic::Sigma(2, pixelNoise);
}

void GraphManager::resetManager()
{
    m_pimpl->currentStateIdx = 0;
    m_pimpl->previousStateIdx = 0;
    m_pimpl->smoother = gtsam::IncrementalFixedLagSmoother(m_pimpl->lag, m_pimpl->isam2Params);

    m_pimpl->keyIdxLookupByTimeStamp.clear();
    m_pimpl->timestampLookupByKeyIdx.clear();
    m_pimpl->smartPointLdmks.clear();
    m_pimpl->newlyAddedLdmkIds.clear();
    m_pimpl->fullValues.clear();

    m_pimpl->resetTemporaryGraph();
}

void GraphManager::Impl::resetTemporaryGraph()
{
    newValues.clear();
    newFactors.resize(0);
    newTimeStamps.clear();
    smartPointLdmks.clear();
    newlyAddedLdmkIds.clear();
}

bool GraphManager::optimize()
{
    const std::string printPrefix{"[GraphManager::Optimize]"};
    if (m_pimpl->currentStateIdx > 0)
    {
        // update smoother with only new content
        try
        {
            m_pimpl->addOrMarkRemovableSmartFactorsToFactorGraph();

            for (auto& factorID : m_pimpl->factorsToRemove)
            {
                BipedalLocomotion::log()->warn( "Factor to remove requested to smoother: {}",factorID);
            }

            m_pimpl->smoother.update(m_pimpl->newFactors,
                                     m_pimpl->newValues,
                                     m_pimpl->newTimeStamps,
                                     m_pimpl->factorsToRemove);

            m_pimpl->fullValues = m_pimpl->smoother.calculateEstimate();
        }
        catch (gtsam::IndeterminantLinearSystemException& e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
        catch (const gtsam::InvalidNoiseModel& e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
        catch (const gtsam::InvalidMatrixBlock& e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
        catch (const gtsam::InvalidDenseElimination& e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
        catch (const gtsam::InvalidArgumentThreadsafe& e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
        catch (const gtsam::ValuesKeyDoesNotExist& e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
        catch (const gtsam::CholeskyFailed& e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
        catch (const gtsam::CheiralityException& e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
        catch (const gtsam::RuntimeErrorThreadsafe& e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
        catch (const gtsam::OutOfRangeThreadsafe& e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
        catch (const std::out_of_range& e)
        {
            std::cerr << e.what() << std::endl;
            return false;
        }
        catch (const std::exception& e)
        {
            // Catch anything thrown within try block that derives from
            // std::exception.
            std::cerr << e.what() << std::endl;
            return false;
        }
        catch (...)
        {
            // Catch the rest of exceptions.
            std::cerr << "Unrecognized exception." << std::endl;
            return false;
        }

        // clear temporary variables
        m_pimpl->resetTemporaryGraph();
        if (!m_pimpl->updateSmartFactorSlotsFromISAMResult())
        {
            BipedalLocomotion::log()->error("{} Could not update smart factor slots."
                                            " This might cause issues during marginalization.", printPrefix);
        }
    }

    auto poseKey = X(m_pimpl->currentStateIdx);
    auto velKey = V(m_pimpl->currentStateIdx);
    auto imuBiasKey = B(m_pimpl->currentStateIdx);
    m_pimpl->estimatedBasePose = m_pimpl->smoother.calculateEstimate<gtsam::Pose3>(poseKey);
    m_pimpl->estimatedBaseLinVel = m_pimpl->smoother.calculateEstimate<gtsam::Vector3>(velKey);
    m_pimpl->estimatedIMUBias
        = m_pimpl->smoother.calculateEstimate<gtsam::imuBias::ConstantBias>(imuBiasKey);

    return true;
}

void GraphManager::spawnNewState(const double& timeStamp)
{
    const std::string printPrefix{"[GraphManager::spawnNewState]"};
    m_pimpl->previousStateIdx = m_pimpl->currentStateIdx;
    m_pimpl->currentStateIdx++;

    m_pimpl->keyIdxLookupByTimeStamp[timeStamp] = m_pimpl->currentStateIdx;
    m_pimpl->timestampLookupByKeyIdx[m_pimpl->currentStateIdx] = timeStamp;

    auto currentPoseKey{X(m_pimpl->currentStateIdx)};
    auto currentVelKey{V(m_pimpl->currentStateIdx)};
    auto currentIMUBiasKey{B(m_pimpl->currentStateIdx)};

    BipedalLocomotion::log()->info("{} Spawned new state idx: {} at ts: {}.",
                                   printPrefix,
                                   m_pimpl->currentStateIdx, timeStamp);
    // get time stamp from the spawned state index
    m_pimpl->newTimeStamps[currentPoseKey] = m_pimpl->newTimeStamps[currentVelKey]
        = m_pimpl->newTimeStamps[currentIMUBiasKey] = timeStamp;
}

void GraphManager::setInitialGuessForCurrentStates(const gtsam::Pose3& pose,
                                                   const gtsam::Vector3& vel,
                                                   gtsam::imuBias::ConstantBias imuBias)
{
    auto currentPoseKey{X(m_pimpl->currentStateIdx)};
    auto currentVelKey{V(m_pimpl->currentStateIdx)};
    auto currentIMUBiasKey{B(m_pimpl->currentStateIdx)};
    m_pimpl->newValues.insert(currentPoseKey, pose);
    m_pimpl->newValues.insert(currentVelKey, vel);
    m_pimpl->newValues.insert(currentIMUBiasKey, imuBias);
    m_pimpl->fullValues.insert(currentPoseKey, pose);
    m_pimpl->fullValues.insert(currentVelKey, vel);
    m_pimpl->fullValues.insert(currentIMUBiasKey, imuBias);
}

void GraphManager::processPreintegratedIMUMeasurements(
    const gtsam::PreintegratedCombinedMeasurements& preintIMU)
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
    m_pimpl->newFactors.add(imuFactor);
}

void GraphManager::processPointFeatures(const TrackedFeatures& keyFrameFeatures,
                                        double pixelNoise)
{
    const auto& featureIds = keyFrameFeatures.points.ids;
    const auto& uvs = keyFrameFeatures.points.uvs;

    for (std::size_t idx = 0; idx < featureIds.size(); idx++)
    {
        auto uv = uvs[idx];
        auto id = featureIds[idx];
        // if smart factor related to marker already exists
        // then update the measurement
        if (m_pimpl->smartPointLdmks.find(id) != m_pimpl->smartPointLdmks.end())
        {
            m_pimpl->updatePointFeature(id, uv);
            continue;
        }

        m_pimpl->addPointFeature(id, uv, pixelNoise);
    }
}

void GraphManager::Impl::addPointFeature(const long long int& id,
                                         cv::Point2f uv,
                                         const double& pixelNoise)
{
    auto currentPoseKey{X(currentStateIdx)};
    // if smart factor does not exist, add it to the graph
    if (!smartFactorNoiseModel)
    {
        setupSmartFactorNoiseModel(pixelNoise);
    }

    auto smartPoint = boost::make_shared<SmartFactor>(smartFactorNoiseModel,
                                                      camK,
                                                      b_H_cam,
                                                      smartParams);
    smartPoint->add(gtsam::Point2(uv.x, uv.y), currentPoseKey);

    smartPointLdmks[id] = smartPoint;
    // this slot=-1 will be updated after calling optimize
    existingSmartFactorsMap[id] = {smartPoint, -1};
}

void GraphManager::Impl::updatePointFeature(const long long int& id,
                                            cv::Point2f uv)
{
    auto currentPoseKey{X(currentStateIdx)};
    smartPointLdmks.at(id)->add(gtsam::Point2(uv.x, uv.y), currentPoseKey);
}


bool GraphManager::Impl::updateSmartFactorSlotsFromISAMResult()
{
    // Documentation from Kimera-VIO
    // BOOKKEEPING: updates the SlotIdx in the existingSmartFactorsMap such that
    // this idx points to the updated slots in the graph after optimization.
    // for next iteration to know which slots have to be deleted
    // before adding the new smart factors.
    std::string printPrefix{"[GraphManager::Impl::updateSmartFactorSlotsFromISAMResult]"};
    const auto& result = smoother.getISAM2Result();
    const auto& factors = smoother.getFactors();
    const auto& newSlots = result.newFactorsIndices;
    if (newlyAddedLdmkIds.size() > newSlots.size())
    {
        BipedalLocomotion::log()->error("{} Size of newly added point smart factors"
                                        " seems to be greater than newly added factors."
                                        " This is not expected.", printPrefix);
        return false;
    }

    for (std::size_t idx = 0; idx < newSlots.size(); idx++)
    {
        auto& slot = newSlots[idx];
        auto newlyAddedFactor = boost::dynamic_pointer_cast<SmartFactor>(factors.at(slot));
        if (newlyAddedFactor)
        {
            for (auto& [id, factorWithSlot] : existingSmartFactorsMap)
            {
                if (factorWithSlot.first.get() == newlyAddedFactor.get())
                {
                    factorWithSlot.second = slot;
                }
            }
        }
    }

    return true;
}

bool GraphManager::Impl::addOrMarkRemovableSmartFactorsToFactorGraph()
{
    std::string printPrefix{"[GraphManager::Impl::addSmartFactorsToFactorGraph]"};
    auto timeBeforeLag = timestampLookupByKeyIdx[currentStateIdx] - lag;
    auto marginalizableKeys = findAndMoveKeysBefore(timeBeforeLag);
    factorsToRemove.clear();
    for (const auto& [id, newSmartFactor] : smartPointLdmks)
    {
        if (existingSmartFactorsMap.find(id) ==
            existingSmartFactorsMap.end())
        {
            BipedalLocomotion::log()->error("{} Smart factor with id: {}"
                                            " cannot be found in existing smart factors map.",
                                            printPrefix, id);
            return false;
        }

        const auto& existingFactorIt = existingSmartFactorsMap.find(id);
        auto slotInGraph = existingFactorIt->second.second;
        if (slotInGraph != -1)
        {
            // slot different than the initial value -1
            // factor has been already added and the slot has been updated
            // post optimization from the ISAM2 result
            if (smoother.getFactors().exists(slotInGraph))
            {
                auto factor = smoother.getFactors().at(slotInGraph);
                auto hasMarginalizableKey = factorHasAMarginalizableKey(factor, marginalizableKeys);
                if (hasMarginalizableKey)
                {
                    factorsToRemove.emplace_back(slotInGraph);
                }
            }
            else
            {
                // the smart factor has been marginalized out
                // let's remove it from the existingSmartFactorsMap
                existingSmartFactorsMap.erase(existingFactorIt);
            }
        }
        else
        {
            // add to factor graph
            newFactors.add(newSmartFactor);
            newlyAddedLdmkIds.emplace_back(id);
        }
    }
    return true;
}

void GraphManager::processAbsolutePoseMeasurement(const gtsam::Pose3& absPose,
                                                  double sigmaPos,
                                                  double sigmaRot)
{
    // gtsam pose vector serialized as rotation then tranlation
    // in contrast to iDynTree serialization, i.e. translation then rotation
    auto currentPoseKey{X(m_pimpl->currentStateIdx)};
    auto noiseModel = gtsam::noiseModel::Diagonal::Precisions(
        (gtsam::Vector6() << gtsam::Vector3::Constant(sigmaRot), gtsam::Vector3::Constant(sigmaPos))
            .finished());
    // We define our robust error model here,
    // providing the default parameter value for the estimator.
    auto huber = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345), noiseModel);
    auto absolutePoseFactor = gtsam::PriorFactor<gtsam::Pose3>(currentPoseKey, absPose, huber);
    m_pimpl->newFactors.add(absolutePoseFactor);
}

void GraphManager::processOdometryMeasurement(const gtsam::Pose3& betweenPose,
                                              const double& sigmaPos,
                                              const double& sigmaRot)
{
    // gtsam pose vector serialized as rotation then tranlation
    // in contrast to iDynTree serialization, i.e. translation then rotation
    auto previousPoseKey{X(m_pimpl->previousStateIdx)};
    auto currentPoseKey{X(m_pimpl->currentStateIdx)};
    auto noiseModel = gtsam::noiseModel::Diagonal::Precisions(
        (gtsam::Vector6() << gtsam::Vector3::Constant(sigmaRot), gtsam::Vector3::Constant(sigmaPos))
            .finished());
    auto betweenFactor = gtsam::BetweenFactor<gtsam::Pose3>(previousPoseKey, currentPoseKey, betweenPose, noiseModel);
    m_pimpl->newFactors.add(betweenFactor);
}

void GraphManager::processZeroMotionMeasurements(const double& sigmaPos,
                                                 const double& sigmaRot)
{
    auto previousPoseKey{X(m_pimpl->previousStateIdx)};
    auto currentPoseKey{X(m_pimpl->currentStateIdx)};
    auto noiseModel = gtsam::noiseModel::Diagonal::Precisions(
        (gtsam::Vector6() << gtsam::Vector3::Constant(sigmaRot), gtsam::Vector3::Constant(sigmaPos))
            .finished());
    auto noMotionFactor = gtsam::BetweenFactor<gtsam::Pose3>(previousPoseKey, currentPoseKey,
                                                             gtsam::Pose3::identity(), noiseModel);
    m_pimpl->newFactors.add(noMotionFactor);
}

void GraphManager::processZeroVelocityMeasurements(const double& sigmaLin)
{
    auto currentVelKey{V(m_pimpl->currentStateIdx)};
    auto noiseModel = gtsam::noiseModel::Isotropic::Sigma(3, sigmaLin);
    auto zeroVelocityFactor = gtsam::PriorFactor<gtsam::Vector3>(currentVelKey,
                                                                 gtsam::Vector3::Zero(),
                                                                 noiseModel);
    m_pimpl->newFactors.add(zeroVelocityFactor);
}

const gtsam::IncrementalFixedLagSmoother& GraphManager::smoother() const
{
    return m_pimpl->smoother;
}

const gtsam::NonlinearFactorGraph& GraphManager::graph() const
{
    return m_pimpl->smoother.getFactors();
}

gtsam::Pose3 GraphManager::getEstimatedBasePose() const
{
    if (m_pimpl->currentStateIdx > 0)
    {
        return m_pimpl->estimatedBasePose;
    } else
    {
        auto currentPoseKey{X(m_pimpl->currentStateIdx)};
        return m_pimpl->fullValues.at<gtsam::Pose3>(currentPoseKey);
    }
}

gtsam::imuBias::ConstantBias GraphManager::getEstimatedIMUBias() const
{
    if (m_pimpl->currentStateIdx > 0)
    {
        return m_pimpl->estimatedIMUBias;
    } else
    {
        auto currentIMUBiasKey{B(m_pimpl->currentStateIdx)};
        return m_pimpl->fullValues.at<gtsam::imuBias::ConstantBias>(currentIMUBiasKey);
    }
}

gtsam::Vector3 GraphManager::getEstimatedBaseLinearVelocity() const
{
    if (m_pimpl->currentStateIdx > 0)
    {
        return m_pimpl->estimatedBaseLinVel;
    } else
    {
        auto currentVelKey{V(m_pimpl->currentStateIdx)};
        return m_pimpl->fullValues.at<gtsam::Vector3>(currentVelKey);
    }
}

gtsam::FastList<gtsam::Key> GraphManager::Impl::findAndMoveKeysBefore(double timestamp)
{
    gtsam::FastList<gtsam::Key> keys;
    std::vector<double> removeTimeStamps;
    auto end = keyIdxLookupByTimeStamp.lower_bound(timestamp);
    for (auto iter = keyIdxLookupByTimeStamp.begin(); iter != end; ++iter)
    {
        keys.emplace_back(X(iter->second));
        keys.emplace_back(V(iter->second));
        keys.emplace_back(B(iter->second));

        removeTimeStamps.emplace_back(iter->first);
    }

    for (const auto& ts : removeTimeStamps)
    {
        keyIdxLookupByTimeStamp.erase(ts);
    }

    return keys;
}

bool GraphManager::Impl::factorHasAMarginalizableKey(boost::shared_ptr< gtsam::NonlinearFactor> factor,
                                                    const gtsam::FastList<gtsam::Key>& marginalizableKeys)
{
    std::string printPrefix{"[GraphManager::Impl::factorHasAMarginalizableKey]"};
    if (factor)
    {
        BipedalLocomotion::log()->error("{} Invalid pointer to factor.",
                                        printPrefix);
        return false;
    }

    bool factorHasAMarginalizableKey{false};
    auto affectedKeys = factor->keys();
    for (const auto& key : marginalizableKeys)
    {
        auto it = std::find(affectedKeys.begin(), affectedKeys.end(), key);
        if (it != affectedKeys.end())
        {
            factorHasAMarginalizableKey = true;
            break;
        }
    }

    return factorHasAMarginalizableKey;
}
