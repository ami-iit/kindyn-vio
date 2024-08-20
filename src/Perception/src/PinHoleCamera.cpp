/**
 * @file PinHoleCamera.cpp
 * @authors Prashanth Ramadoss
 * @copyright Fondazione Istituto Italiano di Tecnologia (IIT). SPDX-License-Identifier: BSD-3-Clause
 */

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <KinDynVIO/Perception/CameraModels/PinHoleCamera.h>

#include <opencv2/core/eigen.hpp>

using namespace KinDynVIO::Perception;
using namespace BipedalLocomotion::ParametersHandler;

PinHoleCamera::PinHoleCamera()
{
}

PinHoleCamera::~PinHoleCamera()
{
}

bool PinHoleCamera::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    const std::string printPrefix{"[ImageProcessor::initialize]"};
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        BipedalLocomotion::log()->error("{} The parameter handler has expired. "
                                        "Please check its scope.",
                                        printPrefix);
        return false;
    }

    Eigen::Matrix<double, 9, 1> calibVec;
    if (!handle->getParameter("camera_matrix", calibVec))
    {
        BipedalLocomotion::log()->error("{} The parameter handler could not find "
                                        "\" camera_matrix \" in the configuration file.",
                                        printPrefix);
        return false;
    }

    m_K = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(calibVec.data());
    m_calibMat = cv::Mat(3, 3, CV_64F);
    cv::eigen2cv(m_K, m_calibMat);

    const auto& fx{m_K(0, 0)};
    const auto& fy{m_K(1, 1)};
    const auto& cx{m_K(0, 2)};
    const auto& cy{m_K(1, 2)};

    if (fx == 0.0 || fy == 0.0)
    {
        BipedalLocomotion::log()->error("{} Invalid focal length in parameter \" camera_matrix \" ",
                                        printPrefix);
        return false;
    }

    m_Kinv << (1 / fx), 0, (-cx / fx), 0, (1 / fy), (-cy / fy), 0, 0, 1;

    if (!handle->getParameter("distortion_coefficients", m_d))
    {
        BipedalLocomotion::log()->error("{} The parameter handler could not find "
                                        "\" distortion_coefficients \" in the configuration file.",
                                        printPrefix);
        return false;
    }

    m_distCoeff = cv::Mat(5, 1, CV_64F);
    cv::eigen2cv(m_d, m_distCoeff);
    if (m_d.norm() < 1e-12)
    {
        m_noDistortion = true;
    }

    if (!handle->getParameter("width", m_width))
    {
        BipedalLocomotion::log()->error("{} The parameter handler could not find "
                                        "\" width \" in the configuration file.",
                                        printPrefix);
        return false;
    }

    if (!handle->getParameter("height", m_height))
    {
        BipedalLocomotion::log()->error("{} The parameter handler could not find "
                                        "\" height \" in the configuration file.",
                                        printPrefix);
        return false;
    }

    return true;
}

bool PinHoleCamera::inBorder(const cv::Point2f& point, const int& borderSize)
{
    int imgX = cvRound(point.x);
    int imgY = cvRound(point.y);

    auto xCheck{(borderSize <= imgX && imgX < m_width - borderSize)};
    auto yCheck{(borderSize <= imgY && imgY < m_height - borderSize)};

    return xCheck && yCheck;
}

void PinHoleCamera::undistortPoints(const std::vector<cv::Point2f>& corners,
                                    std::vector<cv::Point2f>& imagePoints)
{
    if (m_noDistortion)
    {
        imagePoints = corners;
    } else
    {
        cv::undistortPoints(corners, imagePoints, m_calibMat, m_distCoeff);
    }
}

void PinHoleCamera::unprojectPoints(const std::vector<cv::Point2f>& uvs,
                                    std::vector<cv::Point2f>& xcs)
{
    // assumes no distortion
    const auto& fx{m_K(0, 0)};
    const auto& fy{m_K(1, 1)};
    const auto& cx{m_K(0, 2)};
    const auto& cy{m_K(1, 2)};
    xcs.clear();

    // does the operation of Kinv * uv
    // to get normalized coordinates
    for (const auto& uv : uvs)
    {
        cv::Point2f nPt;
        nPt.x = (uv.x - cx) / fx;
        nPt.y = (uv.y - cy) / fy;

        xcs.emplace_back(nPt);
    }
}

void PinHoleCamera::unprojectPoint(const cv::Point2f& uv, cv::Point2f& xc)
{
    const auto& fx{m_K(0, 0)};
    const auto& fy{m_K(1, 1)};
    const auto& cx{m_K(0, 2)};
    const auto& cy{m_K(1, 2)};
    xc.x = (uv.x - cx) / fx;
    xc.y = (uv.y - cy) / fy;
}

const cv::Mat& PinHoleCamera::calibMat() const
{
    return m_calibMat;
}

const Eigen::Matrix3d& PinHoleCamera::K() const
{
    return m_K;
}

const Eigen::Matrix3d& PinHoleCamera::Kinv() const
{
    return m_Kinv;
}

const Eigen::Matrix<double, 5, 1>& PinHoleCamera::d() const
{
    return m_d;
}

const int& PinHoleCamera::cols() const
{
    return m_width;
}

const int& PinHoleCamera::rows() const
{
    return m_height;
}
