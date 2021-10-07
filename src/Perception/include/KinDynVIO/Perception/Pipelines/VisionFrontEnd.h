/**
 * @file VisionFrontEnd.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNVIO_ESTIMATORS_VISION_FRONT_END_H
#define KINDYNVIO_ESTIMATORS_VISION_FRONT_END_H

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/Perception/Features/ArucoDetector.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <KinDynVIO/Perception/CameraModels/PinHoleCamera.h>
#include <KinDynVIO/Perception/Features/FeaturesManager.h>
#include <KinDynVIO/Perception/Features/ImageProcessor.h>
#include <KinDynVIO/Perception/Features/DataTypes.h>
#include <memory>

namespace KinDynVIO
{
namespace Perception
{

class VisionFrontEnd : public BipedalLocomotion::System::Advanceable<TimeStampedImg, TrackedFeatures>
{
public:
    VisionFrontEnd() = default;

    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler) override;
    bool setInput(const TimeStampedImg& stampedImg) override;
    bool advance() override;
    const TrackedFeatures& getOutput() const override;
    bool isOutputValid() const override;

    bool getImageWithDetectedFeatures(cv::Mat& outImg);
    const FeatureManager& featureManager() const;
    const TrackedFeatures& getCurrentFrameFeatures() const;

private:
    std::shared_ptr<PinHoleCamera> m_camera;
    std::shared_ptr<BipedalLocomotion::Perception::ArucoDetector> m_arucoDetector;
    ImageProcessor m_imgProc;
    FeatureManager m_fMgr;
    TrackedFeatures m_keyFrameFeatures, m_currentFrameFeatures;
    bool m_initialized{false};
};

} // namespace Perception
} // namespace KinDynVIO

#endif // KINDYNVIO_ESTIMATORS_VISION_FRONT_END_H

