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
    cv::Mat img;
    double ts{-1.0};
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
    long long int frameID;
};

template<typename PointOrLine>
struct FeatureTrack
{
    // feature ID
    long long int id{-1};

    // look up for frame ID to pixel coordinates of the feature
    std::map<long long int, PointOrLine> uvsAcrossFrames;

    long long int startFrameId{-1}, lastUpdatedFrameId{-1};
    long long int consecutiveTrackCount{0};
};


using PointFeatureTrack = FeatureTrack<cv::Point2f>;
using LineFeatureTrack = FeatureTrack<Line2D>;

} // namespace Perception
} // namespace KinDynVIO

#endif // KINDYNVIO_PERECEPTION_FEATURES_DATA_TYPES_H
