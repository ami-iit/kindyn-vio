/**
 * @file PointsTracker.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynVIO/Perception/Features/PointsTracker.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace KinDynVIO::Perception;

bool PointsTracker::trackPoints(std::shared_ptr<PinHoleCamera> camera,
                                const cv::Mat& prevImg,
                                const cv::Mat& currImg,
                                TrackedPoints& trackedPoints)
{
    const std::string printPrefix{"[PointsTracker::trackPoints]"};
    if (camera == nullptr)
    {
        BipedalLocomotion::log()->error("{} Invalid pointer to camera model.",
                                        printPrefix);
        return false;
    }

    forwardedPoints.clear();
    // if currImg is not the first image then,
    // track features between image pair
    if (prevPoints.size() > 0)
    {
        // get forwarded points using optical flow based KLT tracker
        std::vector<uchar> status;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK(prevImg, currImg,
                                 prevPoints, forwardedPoints,
                                 status, err,
                                 searchWindowSize, maxPyramidLevel);

        // set tracked features failing border check as untracked features
        for (std::size_t idx = 0; idx < forwardedPoints.size(); idx++)
        {
            const auto& pt = forwardedPoints[idx];
            if (status[idx] && camera->inBorder(pt, borderSize))
            {
                status[idx] = 0;
            }
        }

        reduceVector(status, prevPoints);
        reduceVector(status, forwardedPoints);
        reduceVector(status, trackedIDs);
        reduceVector(status, trackCount);
    }

    // reject outliers with RANSAC (Nister 5-point algorithm Essential matrix)
    rejectOutliersWithEssentialMatrix(camera);

    // increment the track count of persisting features
    for (auto& track : trackCount)
    {
        track++;
    }

    // set mask for image sub space with tracked features
    setMask(camera);

    // detect and add new features
    auto nrNewFeatures = maxNrFeatures - forwardedPoints.size();
    if (nrNewFeatures > 0)
    {
        if ( mask.empty() || (mask.type() != CV_8UC1) ||
            (mask.size() != currImg.size()))
        {
            cv::goodFeaturesToTrack(currImg, newPoints, nrNewFeatures,
                                    detectionQuality, minDistanceBetweenFeatures);
        }
        else
        {
            cv::goodFeaturesToTrack(currImg, newPoints, nrNewFeatures,
                                    detectionQuality, minDistanceBetweenFeatures, mask);
        }
    }
    else
    {
        newPoints.clear();
    }

    // update the ids of the new points
    for (auto& pt : newPoints)
    {
        forwardedPoints.emplace_back(pt);
        trackedIDs.emplace_back(featureID++);
        trackCount.emplace_back(1);
    }

    prevPoints = forwardedPoints;

    trackedPoints.uvs = forwardedPoints;
    trackedPoints.ids = trackedIDs;
    trackedPoints.counts = trackCount;
    camera->unprojectPoints(trackedPoints.uvs, trackedPoints.pts);
    return true;
}

void PointsTracker::rejectOutliersWithEssentialMatrix(std::shared_ptr<PinHoleCamera> camera)
{
    if (forwardedPoints.size() >= 5)
    {
        std::vector<cv::Point2f> undistPrevPts, undistForwPts;
        camera->undistortPoints(prevPoints, undistPrevPts);
        camera->undistortPoints(forwardedPoints, undistForwPts);

        std::vector<cv::Point2f> unPrevPts, unForwPts;
        camera->unprojectPoints(undistPrevPts, unPrevPts);
        camera->unprojectPoints(undistForwPts, unForwPts);

        std::vector<uchar> inlierStatus;
        cv::findEssentialMat(unPrevPts, unForwPts, camera->calibMat(), cv::RANSAC, probRANSAC, normalizedThresholdRANSAC, inlierStatus);

        reduceVector(inlierStatus, prevPoints);
        reduceVector(inlierStatus, forwardedPoints);
        reduceVector(inlierStatus, trackedIDs);
        reduceVector(inlierStatus, trackCount);
    }
}

void PointsTracker::setMask(std::shared_ptr<PinHoleCamera> camera)
{
    mask = cv::Mat(camera->rows(), camera->cols(), CV_8UC1, cv::Scalar(255));
    // prefer to keep features that are tracked for long time
    std::vector<std::pair<int, std::pair<cv::Point2f, int> > > countPointID;

    for (std::size_t idx = 0; idx < forwardedPoints.size(); idx++)
    {
        countPointID.emplace_back(std::make_pair(trackCount[idx],
                                                 std::make_pair(forwardedPoints[idx], trackedIDs[idx])));
    }

    // highly tracked feature goes to the top
    std::sort(countPointID.begin(), countPointID.end(),
              [](const std::pair<int, std::pair<cv::Point2f, int> > &a,
                 const std::pair<int, std::pair<cv::Point2f, int>> &b)
                {
                    return a.first > b.first;
                });

    // clear and rearrange buffers
    forwardedPoints.clear();
    trackedIDs.clear();
    trackCount.clear();

    for (auto& it : countPointID)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forwardedPoints.emplace_back(it.second.first);
            trackedIDs.emplace_back(it.second.second);
            trackCount.emplace_back(it.first);
            auto color = cv::Scalar(0);
            int thickness{-1};
            cv::circle(mask, it.second.first, minDistanceBetweenFeatures, color, thickness);
        }
    }
}

