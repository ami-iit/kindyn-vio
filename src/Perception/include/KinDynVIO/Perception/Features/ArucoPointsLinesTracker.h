/**
 * @file ArucoPointsLinesTracker.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNVIO_PERECEPTION_FEATURES_ARUCO_POINTS_LINES_TRACKER_H
#define KINDYNVIO_PERECEPTION_FEATURES_ARUCO_POINTS_LINES_TRACKER_H

#include <BipedalLocomotion/Perception/Features/ArucoDetector.h>
#include <KinDynVIO/Perception/CameraModels/PinHoleCamera.h>
#include <KinDynVIO/Perception/Features/DataTypes.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace KinDynVIO
{
namespace Perception
{

class ArucoPointsLinesTracker
{
public:
    using Input = BipedalLocomotion::Perception::ArucoDetectorOutput;
    ArucoPointsLinesTracker() = default;

    bool trackPoints(std::shared_ptr<PinHoleCamera> camera,
                     const Input& currIn,
                     TrackedPoints2D& trackedPoints);

    bool trackLines(std::shared_ptr<PinHoleCamera> camera,
                    const Input& currIn,
                    TrackedLines2D& trackedLines);

private:
    std::vector<cv::Point2f> m_prevPoints, m_forwardedPoints;
    std::unordered_map<int, cv::Point2f> m_newPoints;
    std::vector<long long int> m_trackedPtIDs;
    std::vector<long long int> m_trackPtCount;

    std::vector<Line2D> m_prevLines, m_forwardedLines;
    std::unordered_map<int, Line2D> m_newLines;
    std::vector<long long int> m_trackedLnIDs;
    std::vector<long long int> m_trackLnCount;
};



} // namespace Perception
} // namespace KinDynVIO

#endif // KINDYNVIO_PERECEPTION_FEATURES_ARUCO_POINTS_LINES_TRACKER_H
