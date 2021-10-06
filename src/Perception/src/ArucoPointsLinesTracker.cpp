/**
 * @file ArucoPointsLinesTracker.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <KinDynVIO/Perception/Features/ArucoPointsLinesTracker.h>
#include <KinDynVIO/Perception/Features/Utils.h>
#include <algorithm>

using namespace KinDynVIO::Perception;

bool ArucoPointsLinesTracker::trackPoints(std::shared_ptr<PinHoleCamera> camera,
                                          const Input& currIn,
                                          TrackedPoints2D& trackedPoints)
{
    const std::string printPrefix{"[ArucoPointsLinesTracker::trackPoints]"};
    if (camera == nullptr)
    {
        BipedalLocomotion::log()->error("{} Invalid pointer to camera model.",
                                        printPrefix);
        return false;
    }

    m_newPoints.clear();
    m_forwardedPoints.clear();

    const auto& markers = currIn.markers;

    if (m_prevPoints.size() > 0)
    {
        // go through the trackedIDs vector
        // check with current input
        // if matches found, track count ++
        std::vector<uchar> status(m_trackedPtIDs.size(), static_cast<uchar>(1));

        for (std::size_t idx = 0; idx < m_trackedPtIDs.size(); idx++)
        {
            const auto& id = m_trackedPtIDs[idx];
            if (markers.find(id) == markers.end())
            {
                status[idx] = static_cast<uchar>(0);
            }
            else
            {
                // take first corner of detected aruco marker
                auto feature = markers.at(id).corners[0];
                m_forwardedPoints.emplace_back(feature);
            }
        }

        reducePointsVector(status, m_trackedPtIDs);
        reducePointsVector(status, m_trackPtCount);
    }

    if (m_trackedPtIDs.size() != m_forwardedPoints.size())
    {
        BipedalLocomotion::log()->error("{} Tracked points size mismatch.",
                                        printPrefix);
        return false;
    }

    // increment the track count of persisting features
    for (auto& track : m_trackPtCount)
    {
        track++;
    }

    // get new ids
    for (auto& [id, marker] : markers)
    {
        if (std::find(m_trackedPtIDs.begin(), m_trackedPtIDs.end(),
                      id) == m_trackedPtIDs.end())
        {
            m_newPoints[id] = marker.corners[0];
        }
    }

    // update the ids of the new points
    for (auto& [id, pt] : m_newPoints)
    {
        m_forwardedPoints.emplace_back(pt);
        m_trackedPtIDs.emplace_back(id);
        m_trackPtCount.emplace_back(1);
    }

    m_prevPoints = m_forwardedPoints;

    trackedPoints.uvs = m_forwardedPoints;
    trackedPoints.ids = m_trackedPtIDs;
    trackedPoints.counts = m_trackPtCount;
    camera->unprojectPoints(trackedPoints.uvs, trackedPoints.pts);
    return true;
}

bool ArucoPointsLinesTracker::trackLines(std::shared_ptr<PinHoleCamera> camera,
                                         const Input& currIn,
                                         TrackedLines2D& trackedLines)
{
    const std::string printPrefix{"[ArucoPointsLinesTracker::trackLines]"};
    if (camera == nullptr)
    {
        BipedalLocomotion::log()->error("{} Invalid pointer to camera model.",
                                        printPrefix);
        return false;
    }

    m_newLines.clear();
    m_forwardedLines.clear();

    const auto& markers = currIn.markers;

    if (m_prevLines.size() > 0)
    {
        // go through the trackedIDs vector
        // check with current input
        // if matches found, track count ++
        std::vector<uchar> status(m_trackedLnIDs.size(), static_cast<uchar>(1));

        for (std::size_t idx = 0; idx < m_trackedLnIDs.size(); idx++)
        {
            const auto& id = m_trackedLnIDs[idx];
            if (markers.find(id) == markers.end())
            {
                status[idx] = static_cast<uchar>(0);
            }
            else
            {
                // create line from first two corners
                Line2D l;
                l.startPixelCoord = markers.at(id).corners[0];
                l.endPixelCoord = markers.at(id).corners[1];

                camera->unprojectPoint(l.startPixelCoord, l.startPoint);
                camera->unprojectPoint(l.endPixelCoord, l.endPoint);
                m_forwardedLines.emplace_back(l);
            }
        }

        reducePointsVector(status, m_trackedLnIDs);
        reducePointsVector(status, m_trackLnCount);
    }

    if (m_trackedLnIDs.size() != m_forwardedLines.size())
    {
        BipedalLocomotion::log()->error("{} Tracked points size mismatch.",
                                        printPrefix);
        return false;
    }

    // increment the track count of persisting features
    for (auto& track : m_trackLnCount)
    {
        track++;
    }

    // get new ids
    for (auto& [id, marker] : markers)
    {
        if (std::find(m_trackedLnIDs.begin(), m_trackedLnIDs.end(),
                      id) == m_trackedLnIDs.end())
        {
            Line2D l;
            l.startPixelCoord = markers.at(id).corners[0];
            l.endPixelCoord = markers.at(id).corners[1];

            camera->unprojectPoint(l.startPixelCoord, l.startPoint);
            camera->unprojectPoint(l.endPixelCoord, l.endPoint);
            m_newLines[id] = l;
        }
    }

    // update the ids of the new points
    for (auto& [id, line] : m_newLines)
    {
        m_forwardedLines.emplace_back(line);
        m_trackedLnIDs.emplace_back(id);
        m_trackLnCount.emplace_back(1);
    }

    m_prevLines = m_forwardedLines;
    trackedLines.lines = m_forwardedLines;
    trackedLines.ids = m_trackedLnIDs;
    trackedLines.counts = m_trackLnCount;

    return true;
}
