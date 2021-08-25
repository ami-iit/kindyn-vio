/**
 * @file ArucoWrapper.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <KinDynVIO/Estimators/ArucoWrapper.h>
#include <unordered_set>

using namespace KinDynVIO::Estimators;
using namespace KinDynVIO::Perception;

class ArucoWrapper::Impl
{
public:
    BipedalLocomotion::Perception::ArucoDetector detector;
    ArucoWrapperInput in;
    ArucoKeyFrame out;

    bool initialized{false};
    bool extrinsicSet{false};
    std::unordered_set<int> markerHistory; // set of detected marker IDs

    gtsam::Pose3 b_H_cam; // extrinsic body/base link to camera
    gtsam::Pose3 cam_H_m; // placeholder for marker poses wrt cam
    gtsam::Pose3 b_H_m; // placeholder for marker poses wrt base
    gtsam::Pose3 w_H_m; // placeholder for marker poses in inertial

    NoiseSigmasV measurementNoise, priorNoise; // landmark noise
    double measSigmaPos{0.005};
    double measSigmaRot{0.01};
    double priorSigmaPos{0.1};
    double priorSigmaRot{0.1};
};

ArucoWrapper::ArucoWrapper()
    : m_pimpl(std::make_unique<ArucoWrapper::Impl>())
{
    m_pimpl->b_H_cam.identity();
}

ArucoWrapper::~ArucoWrapper() { }

bool ArucoWrapper::initialize(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    std::string printPrefix{"[ArucoWrapper::initialize]"};
    if (!m_pimpl->extrinsicSet)
    {
        BipedalLocomotion::log()->error("{} Please set b_H_cam extrinsics first.", printPrefix);
        return false;
    }

    auto handle = handler.lock();
    if (!handle)
    {
        BipedalLocomotion::log()->error("{} Invalid ParametersHandler.", printPrefix);
        return false;
    }

    handle->getParameter("sigma_pos", m_pimpl->measSigmaPos);
#ifdef USE_ARUCO_FULL_POSE
    handle->getParameter("sigma_rot", m_pimpl->measSigmaRot);
    m_pimpl->measurementNoise << m_pimpl->measSigmaRot, m_pimpl->measSigmaRot,
        m_pimpl->measSigmaRot, m_pimpl->measSigmaPos, m_pimpl->measSigmaPos, m_pimpl->measSigmaPos;

    m_pimpl->priorNoise << m_pimpl->priorSigmaRot, m_pimpl->priorSigmaRot, m_pimpl->priorSigmaRot,
        m_pimpl->priorSigmaPos, m_pimpl->priorSigmaPos, m_pimpl->priorSigmaPos;
#else
    m_pimpl->measurementNoise << m_pimpl->measSigmaPos, m_pimpl->measSigmaPos,
        m_pimpl->measSigmaPos;
    m_pimpl->priorNoise << m_pimpl->priorSigmaPos, m_pimpl->priorSigmaPos, m_pimpl->priorSigmaPos;
#endif

    m_pimpl->initialized = m_pimpl->detector.initialize(handler);
    if (!m_pimpl->initialized)
    {
        BipedalLocomotion::log()->error("{} Failed to initialize ArucoDetector.", printPrefix);
    }

    return m_pimpl->initialized;
}

bool ArucoWrapper::setBaseLinkCameraExtrinsics(const gtsam::Pose3& b_H_cam)
{
    m_pimpl->b_H_cam = b_H_cam;
    m_pimpl->extrinsicSet = true;
    return true;
}

bool ArucoWrapper::setInput(const ArucoWrapperInput& input)
{
    m_pimpl->in = input;
    return true;
}

bool ArucoWrapper::advance()
{
    std::string printPrefix{"[ArucoWrapper::advance]"};
    if (!m_pimpl->initialized)
    {
        BipedalLocomotion::log()->error("{} Please call initialize first.", printPrefix);
        return false;
    }

    // to avoid input-output mismatch during multi-threading
    m_pimpl->detector.setImage(m_pimpl->in.imgTs.img, m_pimpl->in.imgTs.ts);

    if (!m_pimpl->detector.advance())
    {
        BipedalLocomotion::log()->error("{} Failed to advance ArucoDetector.", printPrefix);
        return false;
    }

    m_pimpl->out.detectorOut = m_pimpl->detector.getOutput();

    // check if its a keyframe
    if (m_pimpl->out.detectorOut.markers.size() > 0)
    {
        m_pimpl->out.isKeyFrame = true;
    } else
    {
        m_pimpl->out.isKeyFrame = false;
    }

    // clear the buffers
    m_pimpl->out.markerMeasures.clear();
    m_pimpl->out.markerPriors.clear();

    if (m_pimpl->out.isKeyFrame)
    {
        for (const auto& [id, markerData] : m_pimpl->out.detectorOut.markers)
        {
            m_pimpl->cam_H_m = gtsam::Pose3(markerData.pose);
            // if a keyframe, check if a detected marker is a new landmark
            if (m_pimpl->markerHistory.find(id) == m_pimpl->markerHistory.end())
            {
                m_pimpl->markerHistory.insert(id);
                // if new landmark, then add prior, compensating for the extrinsics
                // prior = w_H_b_hat * b_H_c * c_H_m
                m_pimpl->b_H_m = m_pimpl->b_H_cam*m_pimpl->cam_H_m;
                m_pimpl->w_H_m = m_pimpl->in.Xhat * m_pimpl->b_H_m;
#if USE_ARUCO_FULL_POSE == 1
                m_pimpl->out.markerPriors[id] = m_pimpl->w_H_m;
#elif USE_ARUCO_FULL_POSE == 0
                m_pimpl->out.markerPriors[id] = m_pimpl->w_H_m.translation();
#endif
            }

            // for each detected marker in marker out
            // add between factor measurement
            // (measurement between camera pose and landmark)
#if USE_ARUCO_FULL_POSE == 1
            m_pimpl->out.markerMeasures[id] = m_pimpl->b_H_m;
#elif USE_ARUCO_FULL_POSE == 0
            m_pimpl->out.markerMeasures[id] = m_pimpl->b_H_m.translation();
#endif
        }
    }
    return true;
}

const ArucoKeyFrame& ArucoWrapper::getOutput() const
{
    return m_pimpl->out;
}

bool ArucoWrapper::isOutputValid() const
{
    return m_pimpl->initialized;
}

NoiseSigmasV ArucoWrapper::landmarkPriorNoiseModel() const
{
    return m_pimpl->priorNoise;
}

NoiseSigmasV ArucoWrapper::landmarkMeasurementNoiseModel() const
{
    return m_pimpl->measurementNoise;
}
