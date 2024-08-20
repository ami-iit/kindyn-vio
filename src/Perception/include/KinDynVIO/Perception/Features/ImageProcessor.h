/**
 * @file ImageProcessor.h
 * @authors Prashanth Ramadoss
 * @copyright Fondazione Istituto Italiano di Tecnologia (IIT). SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef KINDYNVIO_PERECEPTION_FEATURES_IMAGE_PROCESSOR_H
#define KINDYNVIO_PERECEPTION_FEATURES_IMAGE_PROCESSOR_H

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>
#include <KinDynVIO/Perception/Features/DataTypes.h>
#include <KinDynVIO/Perception/CameraModels/PinHoleCamera.h>
#include <KinDynVIO/Perception/Features/PointsTracker.h>
#include <KinDynVIO/Perception/Features/LinesTracker.h>
#include <opencv2/opencv.hpp>
#include <memory>


namespace KinDynVIO
{
namespace Perception
{

struct TrackedFeatures
{
    TrackedLines2D lines;
    TrackedPoints2D points;
};

class ImageProcessor : public BipedalLocomotion::System::Advanceable<TimeStampedImg, TrackedFeatures>
{
public:
    ImageProcessor();
    ~ImageProcessor();
    /**
     * Initialize the ImageProcessor.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |               Parameter Name              |   Type   |                                                   Description                                                                 | Mandatory |
     * |:-----------------------------------------:|:--------:|:-----------------------------------------------------------------------------------------------------------------------------:|:---------:|
     * |              `tracker_type`               |`string`  | Type of features to be tracked in the image. The available options are: `points`, `lines` and `points_and_lines`.             |    No     |
     * |              `equalize_img`               |`boolean` |                        Equalize image using histograms to improve contrast of the image.                                      |    No     |
     * |              `debug`                      |`boolean` |                                           Enable Verbose outputs and debug prints.                                            |    No     |
     * |   `drawn_tracked_feature_window_size`     |   `int`  |Length of window tracked features to interpolate its color from blue/green to red during visualization depending on longevity. |    No     |
     * |         `drawn_feature_radius`            |   `int`  |                                 Radius for drawn point features in pixels. Default value is 2.                                |    No     |
     * |       `drawn_feature_thickness`           |   `int`  |                                  Thickness  for drawn features in pixels. Default value is 2.                                 |    No     |
     * |           `drawn_font_scale`              |`double`  |                                  Font sclae for accompanying text for drawn features. Default value is 0.35.                  |    No     |
     */
    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler) override;
    bool setImage(const cv::Mat& img, const double& receiveTimeInSeconds);
    bool setInput(const TimeStampedImg& stampedImg) override;

    //Pinhole camera for points manipulation
    bool setCameraModel(std::shared_ptr<KinDynVIO::Perception::PinHoleCamera> camera);

    bool advance() override;
    bool getImageWithDetectedFeatures(cv::Mat& outImg);
    const TrackedFeatures& getOutput() const override;
    bool isOutputValid() const override;

    // delete copy assignment and copy constructor
    ImageProcessor(const ImageProcessor&) = delete;
    ImageProcessor operator=(const ImageProcessor&) = delete;

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};


} // namespace Perception
} // namespace KinDynVIO


#endif // KINDYNVIO_PERECEPTION_FEATURES_IMAGE_PROCESSOR_H
