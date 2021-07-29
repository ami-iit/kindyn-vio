/**
 * @file PointsTracker.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2017 HKUST-Aerial-Robotics Group. This software may be modified and
 * distributed under the terms of the GNU General Public License v3.0 (GPL-3.0 License) or any later
 * version.
 *
 * This file is authored by:
 *  (c) 2021 Prashanth Ramadoss @ DIC-IIT, Genova, Italy
 *
 *  adapted from the file `VINS-MONO/feature_tracker.cpp`:
 *  (c) 2017 Tong QIN @ HKUST-Aerial-Robotics Group, Hong Kong
 */

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <KinDynVIO/Perception/Features/PointsTracker.h>

using namespace KinDynVIO::Perception;

bool PointsTracker::trackPoints(std::shared_ptr<PinHoleCamera> camera,
                                const cv::Mat& prevImg,
                                const cv::Mat& currImg,
                                TrackedPoints& trackedPoints)
{
    const std::string printPrefix{"[PointsTracker::trackPoints]"};
    if (camera == nullptr)
    {
        BipedalLocomotion::log()->error("{} Invalid pointer to camera model.", printPrefix);
        return false;
    }

    m_forwardedPoints.clear();
    m_newPoints.clear();
    // if currImg is not the first image then,
    // track features between image pair
    if (m_prevPoints.size() > 0)
    {
        // get forwarded points using optical flow based KLT tracker
        std::vector<uchar> status, statusReverse;
        std::vector<float> err, errReverse;
        cv::calcOpticalFlowPyrLK(prevImg,
                                 currImg,
                                 m_prevPoints,
                                 m_forwardedPoints,
                                 status,
                                 err,
                                 m_searchWindowSize,
                                 m_maxPyramidLevel);
        std::vector<cv::Point2f> prevPointsReverse;
        cv::calcOpticalFlowPyrLK(currImg,
                                 prevImg,
                                 m_forwardedPoints,
                                 prevPointsReverse,
                                 statusReverse,
                                 errReverse,
                                 m_searchWindowSize,
                                 m_maxPyramidLevel);

        // if reverse optical flow from forwarded points
        // do not match the previous points, then mark the point as untracked
        for (std::size_t idx = 0; idx < m_prevPoints.size(); idx++)
        {
            const auto& p1 = m_prevPoints[idx];
            const auto& p2 = prevPointsReverse[idx];
            if (std::abs(p1.x - p2.x) > 1 || std::abs(p1.y - p2.y) > 1)
            {
                status[idx] = static_cast<uchar>(0);
            }
        }

        // set tracked features failing border check as untracked features
        for (std::size_t idx = 0; idx < m_forwardedPoints.size(); idx++)
        {
            const auto& pt = m_forwardedPoints[idx];
            if (static_cast<int>(status[idx]) && !camera->inBorder(pt, m_borderSize))
            {
                status[idx] = static_cast<uchar>(0);
            }
        }

        reduceVector(status, m_prevPoints);
        reduceVector(status, m_forwardedPoints);
        reduceVector(status, m_trackedIDs);
        reduceVector(status, m_trackCount);
    }

    // reject outliers with RANSAC (Nister 5-point algorithm Essential matrix)
    rejectOutliersWithEssentialMatrix(camera);

    // increment the track count of persisting features
    for (auto& track : m_trackCount)
    {
        track++;
    }

    // set mask for image sub space with tracked features
    setMask(camera);

    // detect and add new features
    auto nrNewFeatures = m_maxNrFeatures - m_forwardedPoints.size();
    if (nrNewFeatures > 0)
    {
        if (m_mask.empty() || (m_mask.type() != CV_8UC1) || (m_mask.size() != currImg.size()))
        {
            cv::goodFeaturesToTrack(currImg,
                                    m_newPoints,
                                    nrNewFeatures,
                                    m_detectionQuality,
                                    m_minDistanceBetweenFeatures);
        } else
        {
            cv::goodFeaturesToTrack(currImg,
                                    m_newPoints,
                                    nrNewFeatures,
                                    m_detectionQuality,
                                    m_minDistanceBetweenFeatures,
                                    m_mask);
        }
    } else
    {
        m_newPoints.clear();
    }

    // update the ids of the new points
    for (auto& pt : m_newPoints)
    {
        m_forwardedPoints.emplace_back(pt);
        m_trackedIDs.emplace_back(m_featureID++);
        m_trackCount.emplace_back(1);
    }

    m_prevPoints = m_forwardedPoints;

    trackedPoints.uvs = m_forwardedPoints;
    trackedPoints.ids = m_trackedIDs;
    trackedPoints.counts = m_trackCount;
    camera->unprojectPoints(trackedPoints.uvs, trackedPoints.pts);
    return true;
}

void PointsTracker::rejectOutliersWithEssentialMatrix(std::shared_ptr<PinHoleCamera> camera)
{
    if (m_forwardedPoints.size() >= 5)
    {
        std::vector<cv::Point2f> undistPrevPts, undistForwPts;
        camera->undistortPoints(m_prevPoints, undistPrevPts);
        camera->undistortPoints(m_forwardedPoints, undistForwPts);

        std::vector<cv::Point2f> unPrevPts, unForwPts;
        camera->unprojectPoints(undistPrevPts, unPrevPts);
        camera->unprojectPoints(undistForwPts, unForwPts);

        std::vector<uchar> inlierStatus;
        cv::findEssentialMat(unPrevPts,
                             unForwPts,
                             camera->calibMat(),
                             cv::RANSAC,
                             m_probRANSAC,
                             m_normalizedThresholdRANSAC,
                             inlierStatus);

        reduceVector(inlierStatus, m_prevPoints);
        reduceVector(inlierStatus, m_forwardedPoints);
        reduceVector(inlierStatus, m_trackedIDs);
        reduceVector(inlierStatus, m_trackCount);
    }
}

void PointsTracker::setMask(std::shared_ptr<PinHoleCamera> camera)
{
    m_mask = cv::Mat(camera->rows(), camera->cols(), CV_8UC1, cv::Scalar(255));
    // prefer to keep features that are tracked for long time
    std::vector<std::pair<int, std::pair<cv::Point2f, int>>> countPointID;

    for (std::size_t idx = 0; idx < m_forwardedPoints.size(); idx++)
    {
        countPointID.emplace_back(
            std::make_pair(m_trackCount[idx],
                           std::make_pair(m_forwardedPoints[idx], m_trackedIDs[idx])));
    }

    // highly tracked feature goes to the top
    std::sort(countPointID.begin(),
              countPointID.end(),
              [](const std::pair<int, std::pair<cv::Point2f, int>>& a,
                 const std::pair<int, std::pair<cv::Point2f, int>>& b) {
                  return a.first > b.first;
              });

    // clear and rearrange buffers
    m_forwardedPoints.clear();
    m_trackedIDs.clear();
    m_trackCount.clear();

    for (auto& it : countPointID)
    {
        if (m_mask.at<uchar>(it.second.first) == 255)
        {
            m_forwardedPoints.emplace_back(it.second.first);
            m_trackedIDs.emplace_back(it.second.second);
            m_trackCount.emplace_back(it.first);
            auto color = cv::Scalar(0);
            int thickness{-1};
            cv::circle(m_mask, it.second.first, m_minDistanceBetweenFeatures, color, thickness);
        }
    }
}
