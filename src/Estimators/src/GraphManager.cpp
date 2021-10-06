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

    long long int currentStateIdx{0}, previousStateIdx{0};
    double lag{150.0};
    gtsam::IncrementalFixedLagSmoother smoother;
    gtsam::ISAM2Params isam2Params;

    // temporary graph variables
    gtsam::NonlinearFactorGraph newFactors;
    gtsam::Values newValues, fullValues;

    gtsam::FixedLagSmoother::KeyTimestampMap newTimeStamps; // timestamp lookup by key
    std::map<double, std::size_t> keyIdxLookupByTimeStamp; // Key Index based on timestamp
    std::map<std::size_t, double> timestampLookupByKeyIdx;
    std::unordered_map<int, SmartFactor::shared_ptr> smartArucoLdmk;
    gtsam::Cal3_S2 camK;

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

    m_pimpl->newTimeStamps[poseKey] = m_pimpl->newTimeStamps[velKey]
        = m_pimpl->newTimeStamps[imuBiasKey] = timeStamp;

    m_pimpl->keyIdxLookupByTimeStamp[timeStamp] = m_pimpl->currentStateIdx;
    m_pimpl->timestampLookupByKeyIdx[m_pimpl->currentStateIdx] = timeStamp;

    return true;
}

void GraphManager::resetManager()
{
    m_pimpl->currentStateIdx = 0;
    m_pimpl->previousStateIdx = 0;
    m_pimpl->smoother = gtsam::IncrementalFixedLagSmoother(m_pimpl->lag, m_pimpl->isam2Params);

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
    const std::string printPrefix{"[GraphManager::Optimize]"};
    if (m_pimpl->currentStateIdx > 0)
    {
        // update smoother with only new content
        try
        {
            m_pimpl->smoother.update(m_pimpl->newFactors,
                                     m_pimpl->newValues,
                                     m_pimpl->newTimeStamps);

            m_pimpl->fullValues = m_pimpl->smoother.calculateEstimate();
            m_pimpl->smoother.getISAM2Result().print();
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
    m_pimpl->previousStateIdx = m_pimpl->currentStateIdx;
    m_pimpl->currentStateIdx++;

    m_pimpl->keyIdxLookupByTimeStamp[timeStamp] = m_pimpl->currentStateIdx;
    m_pimpl->timestampLookupByKeyIdx[m_pimpl->currentStateIdx] = timeStamp;

    auto currentPoseKey{X(m_pimpl->currentStateIdx)};
    auto currentVelKey{V(m_pimpl->currentStateIdx)};
    auto currentIMUBiasKey{B(m_pimpl->currentStateIdx)};

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

void GraphManager::processArucoKeyFrames(const ArucoKeyFrame& arucoKF, double pixelNoise)
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

        gtsam::SmartProjectionParams smartParams;
        smartParams.setDegeneracyMode(gtsam::DegeneracyMode::ZERO_ON_DEGENERACY);
        smartParams.setLinearizationMode(gtsam::LinearizationMode::HESSIAN);
        smartParams.setEnableEPI(false); // set to true, refines triangulation using LM iterations
        smartParams.setLandmarkDistanceThreshold(20.0); // max distance to triangulate in meters
        smartParams.setRankTolerance(1.0);
        smartParams.setRetriangulationThreshold(1e-3);
        smartParams.setDynamicOutlierRejectionThreshold(8.0); // max acceptable reprojection error
        smartParams.throwCheirality = false;
        smartParams.verboseCheirality = false;

        auto smartAruco = boost::make_shared<SmartFactor>(measNoise, K, arucoKF.b_H_cam, smartParams);
        m_pimpl->smartArucoLdmk[id] = smartAruco;
        smartAruco->add(gtsam::Point2(uv.x, uv.y), currentPoseKey);

        m_pimpl->newFactors.add(smartAruco);
    }
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

const gtsam::IncrementalFixedLagSmoother& GraphManager::smoother() const
{
    return m_pimpl->smoother;
}

const gtsam::NonlinearFactorGraph& GraphManager::graph() const
{
    return m_pimpl->newFactors;
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
