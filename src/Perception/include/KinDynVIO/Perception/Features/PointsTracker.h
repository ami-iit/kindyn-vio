/**
 * @file PointsTracker.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNVIO_PERECEPTION_FEATURES_POINTS_TRACKER_H
#define KINDYNVIO_PERECEPTION_FEATURES_POINTS_TRACKER_H

#include <KinDynVIO/Perception/CameraModels/PinHoleCamera.h>
#include <opencv2/opencv.hpp>
#include <vector>


namespace KinDynVIO
{
namespace Perception
{

struct TrackedPoints
{
    std::vector<cv::Point2f> pts;
    std::vector<long long int> ids;
    std::vector<long long int> counts;
};

/**
 *  An Improvised version of VINS-MONO
 */
class PointsTracker
{
public:
    bool trackPoints(std::shared_ptr<PinHoleCamera> camera,
                     const cv::Mat& prevImg,
                     const cv::Mat& currImg,
                     TrackedPoints& trackedPoints);

private:
    template <typename T>
    void reduceVector(const std::vector<uchar> status, std::vector<T>& v)
    {
        std::size_t jdx{0};
        for (std::size_t idx = 0; idx < v.size(); idx++)
        {
            if (status[idx])
            {
                v[jdx++] = v[idx]; // places active features in the front of the vector
            }
        }

        v.resize(jdx); // remove the last inactive features
    }

    void rejectOutliersWithEssentialMatrix(std::shared_ptr<PinHoleCamera> camera);
    void setMask(std::shared_ptr<PinHoleCamera> camera);

    std::size_t maxNrFeatures{1000};
    double fastThreshold{10};
    int minDistanceBetweenFeatures{30}; // in pixels
    int maxPyramidLevel{3}; // 0-based levels of sub-image pyramids for KLT tracker
    cv::Size searchWindowSize{cv::Size(21, 21)}; // window size in pixels for KLT tracker
    int borderSize{1}; // image border size to check for interior tracked features
    double probRANSAC{0.99};
    double normalizedThresholdRANSAC{0.0003};
    double detectionQuality{0.1};

    cv::Mat mask;
    std::vector<cv::Point2f> prevPoints;
    std::vector<cv::Point2f> newPoints, forwardedPoints;
    std::vector<long long int> trackedIDs;
    std::vector<long long int> trackCount;

    TrackedPoints out;
    long long int featureID{0};
};

} // namespace Perception
} // namespace KinDynVIO

#endif // KINDYNVIO_PERECEPTION_FEATURES_POINTS_TRACKER_H
