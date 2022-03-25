/**
 * @file PointsTracker.h
 * @authors Prashanth Ramadoss
 * @copyright 2017 HKUST-Aerial-Robotics Group. This software may be modified and
 * distributed under the terms of the GNU General Public License v3.0 (GPL-3.0 License) or any later version.
 *
 * This file is authored by:
 *  (c) 2021 Prashanth Ramadoss @ ami-iit, Genova, Italy
 *
 *  adapted from the file `VINS-MONO/feature_tracker.cpp`:
 *  (c) 2017 Tong QIN @ HKUST-Aerial-Robotics Group, Hong Kong
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

struct TrackedPoints2D
{
    std::vector<cv::Point2f> uvs; // image points in pixels
    std::vector<long long int> ids;
    std::vector<long long int> counts;
    std::vector<cv::Point2f> pts; // normalized coordinates after passing uvs through Kinv
};

/**
 *  An Improvised version of VINS-MONO
 *
 *  If  you are using this piece of code, please cite the original papers
 *  @inproceedings{qin2018online,
 *  title={Online Temporal Calibration for Monocular Visual-Inertial Systems},
 *  author={Qin, Tong and Shen, Shaojie},
 *  booktitle={2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
 *  pages={3662--3669},
 *  year={2018},
 *  organization={IEEE}
 *  }
 *  @article{qin2017vins,
 *  title={VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator},
 *  author={Qin, Tong and Li, Peiliang and Shen, Shaojie},
 *  journal={IEEE Transactions on Robotics},
 *  year={2018},
 *  volume={34},
 *  number={4},
 *  pages={1004-1020}
 *  }
 */
class PointsTracker
{
public:
    bool trackPoints(std::shared_ptr<PinHoleCamera> camera,
                     const cv::Mat& prevImg,
                     const cv::Mat& currImg,
                     TrackedPoints2D& trackedPoints);

private:
    template <typename T>
    void reduceVector(const std::vector<uchar> status, std::vector<T>& v)
    {
        std::size_t jdx{0};
        for (std::size_t idx = 0; idx < v.size(); idx++)
        {
            if (static_cast<int>(status[idx]))
            {
                v[jdx++] = v[idx]; // places active features in the front of the vector
            }
        }

        v.resize(jdx); // remove the last inactive features
    }

    void rejectOutliersWithEssentialMatrix(std::shared_ptr<PinHoleCamera> camera);
    void setMask(std::shared_ptr<PinHoleCamera> camera);

    // NOTE: IMPORTANT PARAMETERS TO TUNE
    // In case the tracking quality seems poor,
    // then reduce the min distance between features so that
    // more features are detected in such a way that
    // RANSAC can eliminate most outliers that did not
    // correspond to an equivalent rigid body motion of the camera
    // Additionally, the detection quality is helpful
    // but might not be useful in texture-less images
    // higher the number of pyramids, higher the tracking performance
    // since we keep reducing the resolution of the tracked features
    // however, more pyramids is time-expensive
    // TODO move these hard-coded parameters as configurable parameters
    // detector parameters
    double m_detectionQuality{0.75};
    std::size_t m_maxNrFeatures{1000};
    int m_minDistanceBetweenFeatures{20}; // in pixels

    // tracker parameters
    int m_maxPyramidLevel{4}; // 0-based levels of sub-image pyramids for KLT tracker
    cv::Size m_searchWindowSize{cv::Size(15, 15)}; // window size in pixels for KLT tracker

    int m_borderSize{1}; // image border size to check for interior tracked features in pixels

    // RANSAC parameters for outlier removal
    double m_probRANSAC{0.99};
    double m_normalizedThresholdRANSAC{0.0003};


    cv::Mat m_mask;
    std::vector<cv::Point2f> m_prevPoints;
    std::vector<cv::Point2f> m_newPoints, m_forwardedPoints;
    std::vector<long long int> m_trackedIDs;
    std::vector<long long int> m_trackCount;

    long long int m_featureID{0};
};

} // namespace Perception
} // namespace KinDynVIO

#endif // KINDYNVIO_PERECEPTION_FEATURES_POINTS_TRACKER_H
