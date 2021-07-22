/**
 * @file ImageProcessor.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNVIO_PERECEPTION_FEATURES_IMAGE_PROCESSOR_H
#define KINDYNVIO_PERECEPTION_FEATURES_IMAGE_PROCESSOR_H

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <KinDynVIO/Perception/CameraModels/PinHoleCamera.h>
#include <opencv2/opencv.hpp>
#include <memory>


namespace KinDynVIO
{
namespace Perception
{

class ImageProcessor
{
public:
    ImageProcessor();
    ~ImageProcessor();
    /**
     * Initialize the ImageProcessor.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |               Parameter Name              |   Type   |                                                   Description                                                     | Mandatory |
     * |:-----------------------------------------:|:--------:|:-----------------------------------------------------------------------------------------------------------------:|:---------:|
     * |              `tracker_type`               |`string`  | Type of features to be tracked in the image. The available options are: `points`, `lines` and `points_and_lines`. |    No     |
     * |              `equalize_img`               |`boolean` |                        Equalize image using histograms to improve contrast of the image.                          |    No     |
     * |              `debug`                      |`boolean` |                                           Enable Verbose outputs and debug prints.                                |    No     |
     */
    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);
    bool setImage(const cv::Mat& img, const double& receiveTimeInSeconds);

    //Pinhole camera for points manipulation
    bool setCameraModel(std::shared_ptr<KinDynVIO::Perception::PinHoleCamera> camera);

    bool advance();
    bool getImageWithDetectedFeatures(cv::Mat& outImg);

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
