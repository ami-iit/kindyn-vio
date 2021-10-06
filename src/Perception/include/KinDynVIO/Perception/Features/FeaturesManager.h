/**
 * @file FeatureManager.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNVIO_ESTIMATORS_FEATURE_MANAGER_H
#define KINDYNVIO_ESTIMATORS_FEATURE_MANAGER_H

#include <KinDynVIO/Perception/Features/DataTypes.h>
#include <queue>
#include <deque>

namespace KinDynVIO
{
namespace Perception
{

// fixed size queue as a SlidingWindow class thanks to
// https://stackoverflow.com/questions/56334492/c-create-fixed-size-queue
template <typename T, typename Container=std::deque<T> >
class SlidingWindow : public std::queue<T, Container>
{
public:
    void setMaxLength(const int& length)
    {
        m_maxLength = length;
    }

    void push(const T& value)
    {
        if (this->size() == m_maxLength)
        {
            // remove oldest element
            // from underlying container
           this->c.pop_front();
        }
        std::queue<T, Container>::push(value);
    }

private:
    int m_maxLength{10};
};


class FeatureManager
{
public:
    FeatureManager() = default;
    void setCurrentFrameFeatures(const TrackedFeatures& features);
    bool advance();

    void printFeatureTracks();

    const bool& isCameraStationary() const;
    const TrackedFeatures& getCurrentKeyFrameFeatures() const;

private:
    void binLineFeatures();
    void binPointFeatures();

    // check whether current tracked features
    // are from a keyframe image
    void detectKeyFrame();

    // detects for zero motion in current frame
    // accounting for last N frames
    bool detectZeroMotion();

    // between keyframe and current frames
    // matching point features and line features
    double computeMedianDisparity(std::vector<double>& disparitySquared);

    std::vector<double>
    getFeatureDisparitiesSquared(const TrackedFeatures& refFrame,
                                 const TrackedFeatures& current);

    // remove feature tracks if not seen
    // for more than 20 frames
    void pruneTracks();

    // maintain a sliding window of last N tracked features
    SlidingWindow<TrackedFeatures> m_lastNFeatures;
    TrackedFeatures m_currFeatures, m_prevFeatures;
    int  m_nrConsecutiveFrameCount{0};
    int m_nrZeroMotionFrames{10};
    double m_cumulativeSumMotionDisparity{0.0};

    // maintain a sliding window of last M keyframe features
    SlidingWindow<TrackedFeatures> m_lastMKeyFrameFeatures;

    double m_medianDisparityThreshold{20.0}; // disparity threshold over which new keyframe is selected
    double m_medianDisparity{0}; // disparity between current frame and last key frame
    double m_zeroMotionThreshold{0.5}; // in pixel per N frames
    int m_nrMaxKeyFramesInStore{5};

    int m_waitNrFramesBeforePruning{20};

    bool m_isStationary{false};

    // lookup for feature ID and corresponding feature track
    std::unordered_map<long long int, PointFeatureTrack> m_pointFeatureTracksMap;
    std::unordered_map<long long int, LineFeatureTrack> m_lineFeatureTracksMap;
};

} // namespace Perception
} // namespace KinDynVIO

#endif // KINDYNVIO_ESTIMATORS_FEATURE_MANAGER_H
