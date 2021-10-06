/**
 * @file FeatureManager.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynVIO/Perception/Features/FeaturesManager.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <algorithm>
#include <cmath>

using namespace KinDynVIO::Perception;

void FeatureManager::setCurrentFrameFeatures(const TrackedFeatures& features)
{
    m_lastNFeatures.push(features);
}

bool FeatureManager::advance()
{
    m_currFeatures = m_lastNFeatures.back();
    binLineFeatures();
    binPointFeatures();
    detectKeyFrame();
    pruneTracks();

    m_isStationary =  detectZeroMotion();
    m_prevFeatures = m_lastNFeatures.back();
    return true;
}

void FeatureManager::binLineFeatures()
{
    for (std::size_t idx = 0; idx < m_currFeatures.lines.ids.size(); idx++)
    {
        const auto& lineID = m_currFeatures.lines.ids[idx];
        if (m_lineFeatureTracksMap.find(lineID) == m_lineFeatureTracksMap.end())
        {
            // add new FeatureTrack
            LineFeatureTrack track;
            track.id = lineID;
            track.consecutiveTrackCount = m_currFeatures.lines.counts[idx];
            track.startFrameId = track.lastUpdatedFrameId = m_currFeatures.frameID;
            track.uvsAcrossFrames[m_currFeatures.frameID] = m_currFeatures.lines.lines[idx];
            m_lineFeatureTracksMap[lineID] = track;
        }
        else
        {
            // update existing FeatureTrack
            auto& track = m_lineFeatureTracksMap.at(lineID);
            if (track.lastUpdatedFrameId != m_currFeatures.frameID)
            {
                // update only if already does not exist
                if (track.uvsAcrossFrames.find(m_currFeatures.frameID) ==
                    track.uvsAcrossFrames.end())
                {
                    track.consecutiveTrackCount = m_currFeatures.lines.counts[idx];
                    track.lastUpdatedFrameId = m_currFeatures.frameID;
                    track.uvsAcrossFrames[m_currFeatures.frameID] = m_currFeatures.lines.lines[idx];
                }
            }
        }
    }
}

void FeatureManager::binPointFeatures()
{
    for (std::size_t idx = 0; idx < m_currFeatures.points.ids.size(); idx++)
    {
        const auto& pointID = m_currFeatures.points.ids[idx];
        if (m_pointFeatureTracksMap.find(pointID) == m_pointFeatureTracksMap.end())
        {
            // add new FeatureTrack
            PointFeatureTrack track;
            track.id = pointID;
            track.consecutiveTrackCount = m_currFeatures.points.counts[idx];
            track.startFrameId = track.lastUpdatedFrameId = m_currFeatures.frameID;
            track.uvsAcrossFrames[m_currFeatures.frameID] = m_currFeatures.points.uvs[idx];
            m_pointFeatureTracksMap[pointID] = track;
        }
        else
        {
            // update existing FeatureTrack
            auto& track = m_pointFeatureTracksMap.at(pointID);
            if (track.lastUpdatedFrameId != m_currFeatures.frameID)
            {
                // update only if already does not exist
                if (track.uvsAcrossFrames.find(m_currFeatures.frameID) ==
                    track.uvsAcrossFrames.end())
                {
                    track.lastUpdatedFrameId = m_currFeatures.frameID;
                    track.uvsAcrossFrames[m_currFeatures.frameID] = m_currFeatures.points.uvs[idx];
                }
            }
        }
    }
}

bool FeatureManager::detectZeroMotion()
{
    auto dispSq = getFeatureDisparitiesSquared(m_prevFeatures, m_currFeatures);
    m_nrConsecutiveFrameCount++;
    for (auto& disp : dispSq)
    {
        if (m_nrConsecutiveFrameCount < m_nrZeroMotionFrames)
        {
            m_cumulativeSumMotionDisparity += std::sqrt(disp);
        }
    }

    if (m_nrConsecutiveFrameCount == m_nrZeroMotionFrames)
    {
        m_cumulativeSumMotionDisparity /= m_nrZeroMotionFrames;
        m_nrConsecutiveFrameCount = 0;

        if (m_cumulativeSumMotionDisparity < m_zeroMotionThreshold)
        {
            m_cumulativeSumMotionDisparity = 0.0;
            return true;
        }
        else
        {
            m_cumulativeSumMotionDisparity = 0.0;
        }
    }

    return false;
}

void FeatureManager::detectKeyFrame()
{
    // if the sliding window of last m
    // key frames is empty and
    // if detected features is greater than zero
    // then choose this current feature as keyframe
    if (m_lastMKeyFrameFeatures.empty())
    {
        std::size_t nrFeats = m_currFeatures.lines.counts.size() +
                                m_currFeatures.points.counts.size();
        if (nrFeats > 0)
        {
            m_lastMKeyFrameFeatures.push(m_currFeatures);
            return;
        }
    }
    else
    {
        // if we already have a key frame
        // check disparity between current frame
        // and last key frame to decide
        // if we want to add a new keyframe
        const auto& lastKeyFrameFeats = m_lastMKeyFrameFeatures.back();
        auto dispSq = getFeatureDisparitiesSquared(lastKeyFrameFeats, m_currFeatures);
        m_medianDisparity = computeMedianDisparity(dispSq);

        if (m_medianDisparity < m_medianDisparityThreshold)
        {
            m_lastMKeyFrameFeatures.push(m_currFeatures);
        }
    }
}

double FeatureManager::computeMedianDisparity(std::vector<double>& disparitySquared)
{
    double disparity{0.0};
    if (disparitySquared.empty())
    {
        return disparity;
    }

    // Reference Kimera-VIO, see
    // https://github.com/MIT-SPARK/Kimera-VIO/blob/master/src/frontend/Tracker.cpp
    // Compute median
    const std::size_t center = disparitySquared.size() / 2;
    // nth element sorts the array partially until it finds the median.
    std::nth_element(disparitySquared.begin(),
                     disparitySquared.begin() + center,
                     disparitySquared.end());
    disparity = std::sqrt(disparitySquared[center]);

    return disparity;
}


std::vector<double>
FeatureManager::getFeatureDisparitiesSquared(const TrackedFeatures& refFrame,
                                             const TrackedFeatures& current)
{
    std::vector<double> disparitySquared;
    // populate disparity squared using point features
    for (std::size_t kfIdx = 0; kfIdx < refFrame.points.ids.size(); kfIdx++)
    {
        auto kfId = refFrame.points.ids[kfIdx];
        const auto& currIds = current.points.ids;
        auto it = std::find(currIds.begin(), currIds.end(), kfId);
        if (it != currIds.end())
        {
            auto currIdx = std::distance(currIds.begin(), it);
            auto pxDiff = refFrame.points.uvs[kfIdx] - current.points.uvs[currIdx];
            double pxDist = (pxDiff.x*pxDiff.x) + (pxDiff.y*pxDiff.y);
            disparitySquared.emplace_back(pxDist);
        }
    }

    // populate disparity squared using line features
    for (std::size_t kfIdx = 0; kfIdx < refFrame.lines.ids.size(); kfIdx++)
    {
        auto kfId = refFrame.lines.ids[kfIdx];
        const auto& currIds = current.lines.ids;
        auto it = std::find(currIds.begin(), currIds.end(), kfId);
        if (it != currIds.end())
        {
            auto currIdx = std::distance(currIds.begin(), it);
            auto pxDiffStart = refFrame.lines.lines[kfIdx].startPixelCoord
                                - current.lines.lines[currIdx].startPixelCoord;
            double pxDistStart = (pxDiffStart.x*pxDiffStart.x) + (pxDiffStart.y*pxDiffStart.y);

            auto pxDiffEnd = refFrame.lines.lines[kfIdx].endPixelCoord
                                - current.lines.lines[currIdx].endPixelCoord;
            double pxDistEnd = (pxDiffEnd.x*pxDiffEnd.x) + (pxDiffEnd.y*pxDiffEnd.y);

            disparitySquared.emplace_back(pxDistStart);
            disparitySquared.emplace_back(pxDistEnd);
        }
    }

    return disparitySquared;
}


void FeatureManager::pruneTracks()
{
    auto currId = m_currFeatures.frameID;

    std::vector<int> pruneLineIds;
    for (const auto& [id, track] : m_lineFeatureTracksMap)
    {
        if (currId - track.lastUpdatedFrameId > m_waitNrFramesBeforePruning)
        {
            pruneLineIds.emplace_back(id);
        }
    }

    for (auto& id : pruneLineIds)
    {
        m_lineFeatureTracksMap.erase(id);
    }

    std::vector<int> prunePtIds;
    for (const auto& [id, track] : m_pointFeatureTracksMap)
    {
        if (currId - track.lastUpdatedFrameId > m_waitNrFramesBeforePruning)
        {
            prunePtIds.emplace_back(id);
        }
    }

    for (auto& id : prunePtIds)
    {
        m_pointFeatureTracksMap.erase(id);
    }
}

const bool&  FeatureManager::isCameraStationary() const
{
    return m_isStationary;
}

const TrackedFeatures& FeatureManager::getCurrentKeyFrameFeatures() const
{
    return m_lastMKeyFrameFeatures.back();
}

void FeatureManager::printFeatureTracks()
{
    std::cout << "Point Features map: " << m_pointFeatureTracksMap.size() << std::endl;
    for (const auto& [id, track] : m_pointFeatureTracksMap)
    {
        std::cout << "---------" << std::endl;
        std::cout << "Point Features id: " << id << std::endl
                  << "Track count: " << track.consecutiveTrackCount << std::endl
                  << "Start frame id: " << track.startFrameId << std::endl
                  << "Last frame id: " << track.lastUpdatedFrameId << std::endl;
    }

    std::cout << "Line Features map: " << m_lineFeatureTracksMap.size() << std::endl;
    for (const auto& [id, track] : m_lineFeatureTracksMap)
    {
        std::cout << "---------" << std::endl;
        std::cout << "Line Features id: " << id << std::endl
                  << "Track count: " << track.consecutiveTrackCount << std::endl
                  << "Start frame id: " << track.startFrameId << std::endl
                  << "Last frame id: " << track.lastUpdatedFrameId << std::endl;
    }
}
