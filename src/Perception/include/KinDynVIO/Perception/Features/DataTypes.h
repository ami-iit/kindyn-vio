/**
 * @file DataTypes.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNVIO_PERECEPTION_FEATURES_DATA_TYPES_H
#define KINDYNVIO_PERECEPTION_FEATURES_DATA_TYPES_H

#include <opencv2/opencv.hpp>

namespace KinDynVIO
{
namespace Perception
{

struct TimeStampedImg
{
    double ts{-1.0};
    cv::Mat img;
};

struct TrackedPoints2D
{
    std::vector<cv::Point2f> uvs; // image points in pixels
    std::vector<long long int> ids;
    std::vector<long long int> counts;
    std::vector<cv::Point2f> pts; // normalized coordinates after passing uvs through Kinv
};

struct Line2D
{
    cv::Point2f startPixelCoord; // uv
    cv::Point2f endPixelCoord;   // uv
    cv::Point2f startPoint;      // normalized unprojected
    cv::Point2f endPoint;        // normalized unprojected
};

struct TrackedLines2D
{
    std::vector<Line2D> lines;
    std::vector<long long int> ids;
    std::vector<long long int> counts;
};


struct TrackedFeatures
{
    TrackedLines2D lines;
    TrackedPoints2D points;
};

} // namespace Perception
} // namespace KinDynVIO

#endif // KINDYNVIO_PERECEPTION_FEATURES_DATA_TYPES_H
