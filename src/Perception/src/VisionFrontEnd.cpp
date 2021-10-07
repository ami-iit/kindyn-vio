/**
 * @file VisionFrontEnd.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynVIO/Perception/Pipelines/VisionFrontEnd.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <algorithm>
#include <cmath>

using namespace KinDynVIO::Perception;
using namespace BipedalLocomotion::ParametersHandler;

bool VisionFrontEnd::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    const std::string printPrefix{"[VisionFrontEnd::initialize]"};
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        BipedalLocomotion::log()->error("{} The parameter handler has expired. "
                                        "Please check its scope.",
                                        printPrefix);
        return false;
    }

    // Get ImageProcessor configuration
    auto imgProcHandler = handle->getGroup("IMAGE_PROCESSOR");
    if (imgProcHandler.lock() == nullptr)
    {
        BipedalLocomotion::log()->error("{} Please check if \"IMAGE_PROCESSOR\" "
                                        "group is available in config.",
                                        printPrefix);
        return false;
    }

    if (!m_imgProc.initialize(imgProcHandler))
    {
        BipedalLocomotion::log()->error("{} Failed to initialize Image Processor.",
                                        printPrefix);
        return false;
    }

    // Get camera model configuration
    auto camModelHandler = handle->getGroup("CAMERA_MODEL");
    if (camModelHandler.lock() == nullptr)
    {
        BipedalLocomotion::log()->error("{} Please check if \"CAMERA_MODEL\" "
                                        "group is available in config.",
                                        printPrefix);
        return false;
    }

    m_camera = std::make_shared<PinHoleCamera>();
    if (!m_camera->initialize(camModelHandler))
    {
        BipedalLocomotion::log()->error("{} Failed to initialize PinHoleCamera.\".",
                                        printPrefix);
        return false;
    }
    m_imgProc.setCameraModel(m_camera);

    // Get ArucoDetector configuration if necessary
    bool useAruco{false};
    imgProcHandler.lock()->getParameter("force_features_from_aruco", useAruco);
    if (useAruco)
    {
        auto arucoParameterHandler = handle->getGroup("ARUCO_DETECTOR");
        if (arucoParameterHandler.lock() == nullptr)
        {
            BipedalLocomotion::log()->error("{} Please check if \"ARUCO_DETECTOR\" "
                                            "group is available in config.",
                                            printPrefix);
            return false;
        }

        m_arucoDetector = std::make_shared<BipedalLocomotion::Perception::ArucoDetector>();
        if (!m_arucoDetector->initialize(arucoParameterHandler))
        {
            BipedalLocomotion::log()->error("{} Failed to initialize aruco detector.\".",
                                            printPrefix);
            return false;
        }

        m_imgProc.setArucoDetector(m_arucoDetector);
    }

    // initialize feature manager config, if found
    auto fMgrHandler = handle->getGroup("FEATURE_MANAGER");
    m_fMgr.initialize(fMgrHandler);

    m_initialized = true;
    return true;
}


bool VisionFrontEnd::setInput(const TimeStampedImg& stampedImg)
{
    return m_imgProc.setInput(stampedImg);
}

bool VisionFrontEnd::advance()
{
    std::string printPrefix{"[VisionFrontEnd::advance]"};
    if (!m_initialized)
    {
        BipedalLocomotion::log()->error("{} Please call initialize first",
                                        printPrefix);
        return false;
    }

    if (!m_imgProc.advance())
    {
        BipedalLocomotion::log()->error("{} Failed to advance ImageProcessor.",
                                        printPrefix);
        return false;
    }

    m_currentFrameFeatures = m_imgProc.getOutput();

    m_fMgr.setCurrentFrameFeatures(m_currentFrameFeatures);
    if (!m_fMgr.advance())
    {
        BipedalLocomotion::log()->error("{} Failed to advance FeatueManager.",
                                        printPrefix);
        return false;
    }

    m_keyFrameFeatures = m_fMgr.getCurrentKeyFrameFeatures();

    return true;
}

const TrackedFeatures& VisionFrontEnd::getOutput() const
{
    return m_keyFrameFeatures;
}

bool VisionFrontEnd::isOutputValid() const
{
    return m_initialized;
}

bool VisionFrontEnd::getImageWithDetectedFeatures(cv::Mat& outImg)
{
    return m_imgProc.getImageWithDetectedFeatures(outImg);
}

const FeatureManager& VisionFrontEnd::featureManager() const
{
    return m_fMgr;
}

const TrackedFeatures& VisionFrontEnd::getCurrentFrameFeatures() const
{
    return m_currentFrameFeatures;
}

