/**
 * @file PinHoleCamera.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNVIO_PERECEPTION_CAMERA_MODELS_PINHOLE_H
#define KINDYNVIO_PERECEPTION_CAMERA_MODELS_PINHOLE_H

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <memory>


namespace KinDynVIO
{
namespace Perception
{


class PinHoleCamera
{
public:
    PinHoleCamera();
    ~PinHoleCamera();
    /**
     * Initialize the PinHoleCamera model.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |               Parameter Name              |        Type        |                                                   Description                                                     | Mandatory |
     * |:-----------------------------------------:|:------------------:|:-----------------------------------------------------------------------------------------------------------------:|:---------:|
     * |              `camera_matrix`              |`vector of double`  |                      9d vector representing the camera calbration matrix in row major order.                      |    Yes    |
     * |         `distortion_coefficients`         |`vector of double`  |                             5d vector containing camera distortion coefficients.                                  |    Yes    |
     * |                  `width`                  |        `int`       |                                           camera image width                                                      |    Yes    |
     * |                  `height`                 |        `int`       |                                           camera image height                                                     |    Yes    |
     */
    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    bool inBorder(const cv::Point2f& point, const int& borderSize);

    void undistortPoints(const std::vector<cv::Point2f>& corners,
                         std::vector<cv::Point2f>& imagePoints);
    void unprojectPoints(const std::vector<cv::Point2f>& uvs,
                         std::vector<cv::Point2f>& xcs);
    void unprojectPoint(const cv::Point2f& uv,
                        cv::Point2f& xc);

    const cv::Mat& calibMat() const;
    const Eigen::Matrix3d& K() const;
    const Eigen::Matrix3d& Kinv() const;
    const Eigen::Matrix<double, 5, 1>& d() const;

    const int& rows() const;
    const int& cols() const;
private:
    Eigen::Matrix3d m_K;
    Eigen::Matrix3d m_Kinv;
    Eigen::Matrix<double, 5, 1> m_d;
    int m_height;
    int m_width;
    bool m_noDistortion{false};

    cv::Mat m_calibMat; // opencv mirror of K
    cv::Mat m_distCoeff;  // opencv mirror of d
};


} // namespace Perception
} // namespace KinDynVIO


#endif // KINDYNVIO_PERECEPTION_CAMERA_MODELS_PINHOLE_H
