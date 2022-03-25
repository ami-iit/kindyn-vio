/**
 * @file LinesTracker.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2017 HKUST-Aerial-Robotics Group. This software may be modified and
 * distributed under the terms of the GNU General Public License v3.0 (GPL-3.0 License) or any later
 * version.
 *
 * This file is authored by:
 *  (c) 2021 Prashanth Ramadoss @ ami-iit, Genova, Italy
 *
 *  adapted from the file `VINS-MONO/feature_tracker.cpp`:
 *  (c) 2017 Tong QIN @ HKUST-Aerial-Robotics Group, Hong Kong
 */

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <KinDynVIO/Perception/Features/LinesTracker.h>

using namespace KinDynVIO::Perception;

LinesTracker::LinesTracker()
{
    // this must be moved to a initialize() method
    m_fld = cv::ximgproc::createFastLineDetector(m_fldParams.lengthThreshold,
                                                 m_fldParams.distanceThreshold,
                                                 m_fldParams.cannyThreshold1,
                                                 m_fldParams.cannyThreshold2,
                                                 m_fldParams.cannyApertureSize,
                                                 m_fldParams.doMerge);
}

LinesTracker::~LinesTracker()
{
}

bool LinesTracker::trackLines(std::shared_ptr<PinHoleCamera> camera,
                              const cv::Mat& prevImg,
                              const cv::Mat& currImg,
                              TrackedLines2D& trackedLines)
{
    const std::string printPrefix{"[LinesTracker::trackPoints]"};
    if (camera == nullptr)
    {
        BipedalLocomotion::log()->error("{} Invalid pointer to camera model.", printPrefix);
        return false;
    }

    m_forwardedLines.clear();
    m_newLines.clear();

    if (m_prevLines.size() > 0)
    {
        // stack end points from lines detected in previous frame
        // and track these points using optical flow in current frame
        std::vector<cv::Point2f> prevPoints, forwardedPoints, prevPointsReversed;
        for (const auto& l : m_prevLines)
        {
            prevPoints.emplace_back(l.startPixelCoord);
            prevPoints.emplace_back(l.endPixelCoord);
        }

        // get forwarded points and reverse previous points
        // using bi-directional optical flow based KLT tracker
        std::vector<uchar> status, statusReverse;
        std::vector<float> err, errReverse;
        cv::calcOpticalFlowPyrLK(prevImg,
                                 currImg,
                                 prevPoints,
                                 forwardedPoints,
                                 status,
                                 err,
                                 m_searchWindowSize,
                                 m_maxPyramidLevel);
        cv::calcOpticalFlowPyrLK(currImg,
                                 prevImg,
                                 forwardedPoints,
                                 prevPointsReversed,
                                 statusReverse,
                                 errReverse,
                                 m_searchWindowSize,
                                 m_maxPyramidLevel);

        // get forwarded lines from forwarded points
        for (std::size_t idx = 0; idx < forwardedPoints.size(); idx += 2)
        {
            Line2D l;
            l.startPixelCoord = forwardedPoints[idx];
            l.endPixelCoord = forwardedPoints[idx + 1];

            camera->unprojectPoint(l.startPixelCoord, l.startPoint);
            camera->unprojectPoint(l.endPixelCoord, l.endPoint);
            m_forwardedLines.emplace_back(l);
        }

        // if reverse optical flow from forwarded points
        // do not match the previous points, then mark the point as untracked
        for (std::size_t idx = 0; idx < prevPoints.size(); idx++)
        {
            const auto& p1 = prevPoints[idx];
            const auto& p2 = prevPointsReversed[idx];
            if (std::abs(p1.x - p2.x) > m_trackingPixelErrThreshold
                || std::abs(p1.y - p2.y) > m_trackingPixelErrThreshold)
            {
                status[idx] = static_cast<uchar>(0);
            }
        }

        // set tracked features failing border check as untracked features
        for (std::size_t idx = 0; idx < forwardedPoints.size(); idx++)
        {
            const auto& pt = forwardedPoints[idx];
            if (static_cast<int>(status[idx]) && !camera->inBorder(pt, m_borderSize))
            {
                status[idx] = static_cast<uchar>(0);
            }
        }

        reduceLinesVector(status, m_prevLines);
        reduceLinesVector(status, m_forwardedLines);
        reduceLinesVector(status, m_trackedIDs);
        reduceLinesVector(status, m_trackCount);
    }

    // increment the track count of persisting features
    for (auto& track : m_trackCount)
    {
        track++;
    }

    // set mask for image sub space with tracked features
    setMask(camera);

    // detect and add new lines
    cv::Mat maskedImg;
    currImg.copyTo(maskedImg, m_mask);

    m_fld->detect(maskedImg, m_detectedLines);
    for (auto& dLine : m_detectedLines)
    {
        Line2D l;
        l.startPixelCoord.x = dLine(0);
        l.startPixelCoord.y = dLine(1);
        l.endPixelCoord.x = dLine(2);
        l.endPixelCoord.y = dLine(3);

        camera->unprojectPoint(l.startPixelCoord, l.startPoint);
        camera->unprojectPoint(l.endPixelCoord, l.endPoint);
        m_newLines.emplace_back(l);
    }

    for (auto& l : m_newLines)
    {
        m_forwardedLines.emplace_back(l);
        m_trackedIDs.emplace_back(m_featureID++);
        m_trackCount.emplace_back(1);
    }

    m_prevLines = m_forwardedLines;
    trackedLines.lines = m_forwardedLines;
    trackedLines.counts = m_trackCount;
    trackedLines.ids = m_trackedIDs;
    return true;
}

void LinesTracker::setMask(std::shared_ptr<PinHoleCamera> camera)
{
    m_mask = cv::Mat(camera->rows(), camera->cols(), CV_8UC1, cv::Scalar(255));
    // prefer to keep features that are tracked for long time
    std::vector<std::pair<int, std::pair<Line2D, int>>> countPointID;

    for (std::size_t idx = 0; idx < m_forwardedLines.size(); idx++)
    {
        countPointID.emplace_back(
            std::make_pair(m_trackCount[idx],
                           std::make_pair(m_forwardedLines[idx], m_trackedIDs[idx])));
    }

    // highly tracked feature goes to the top
    std::sort(countPointID.begin(),
              countPointID.end(),
              [](const std::pair<int, std::pair<Line2D, int>>& a,
                 const std::pair<int, std::pair<Line2D, int>>& b) { return a.first > b.first; });

    // clear and rearrange buffers
    m_forwardedLines.clear();
    m_trackedIDs.clear();
    m_trackCount.clear();

    for (auto& it : countPointID)
    {
        if (m_mask.at<uchar>(it.second.first.startPixelCoord) == 255
            && m_mask.at<uchar>(it.second.first.endPixelCoord) == 255)
        {
            m_forwardedLines.emplace_back(it.second.first);
            m_trackedIDs.emplace_back(it.second.second);
            m_trackCount.emplace_back(it.first);
            auto color = cv::Scalar(0);
            int thickness{2}; // apply non-zero thickness so as to avoid linear masks

            // iterate through the pixels along the line and add a non-linear mask
            auto lineIter = cv::LineIterator(m_mask,
                                             it.second.first.startPixelCoord,
                                             it.second.first.endPixelCoord);

            for (auto ldx = 0; ldx < lineIter.count; ldx++, ++lineIter)
            {
                // skip every other pixel so as to form a irregular mask (mask should not introduce
                // phantom edges)
                if (ldx % m_minDistanceBetweenFeatures == 0)
                {
                    cv::circle(m_mask,
                               lineIter.pos(),
                               m_minDistanceBetweenFeatures,
                               color,
                               thickness);
                }
            }
        }
    }
}
