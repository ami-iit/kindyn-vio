/**
 * @file LinesTracker.h
 * @authors Prashanth Ramadoss
 * @copyright 2017 HKUST-Aerial-Robotics Group. This software may be modified and
 * distributed under the terms of the GNU General Public License v3.0 (GPL-3.0 License) or any later version.
 *
 * This file is authored by:
 *  (c) 2021 Prashanth Ramadoss @ DIC-IIT, Genova, Italy
 *
 *  adapted from the file `VINS-MONO/feature_tracker.cpp`:
 *  (c) 2017 Tong QIN @ HKUST-Aerial-Robotics Group, Hong Kong
 */

#ifndef KINDYNVIO_PERECEPTION_FEATURES_LINES_TRACKER_H
#define KINDYNVIO_PERECEPTION_FEATURES_LINES_TRACKER_H

#include <KinDynVIO/Perception/CameraModels/PinHoleCamera.h>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc/fast_line_detector.hpp>
#include <vector>
#include <utility>

namespace KinDynVIO
{
namespace Perception
{

struct FLDParameters
{
    // TODO move these hard-coded parameters as configurable parameters
    // The following documentation is from
    // https://docs.opencv.org/4.5.2/df/d4c/classcv_1_1ximgproc_1_1FastLineDetector.html#details
    // Param               Default value   Description
    // lengthThreshold       10           - Segments shorter than this will be discarded
    // distanceThreshold     1.41421356   - A point placed from a hypothesis line
    //                                       segment farther than this will be
    //                                       regarded as an outlier
    // cannyThreshold1       50           - First threshold for
    //                                       hysteresis procedure in Canny()
    // cannyThreshold2       50           - Second threshold for
    //                                       hysteresis procedure in Canny()
    // cannyApertureSize     3            - Aperturesize for the sobel operator in Canny().
    //                                      If zero, Canny() is not applied and the input
    //                                      image is taken as an edge image.
    // doMerge               false        - If true, incremental merging of segments
    //                                      will be performed
    int lengthThreshold{30};
    double distanceThreshold{1.41431356};
    double cannyThreshold1{50.0};
    double cannyThreshold2{50.0};
    int cannyApertureSize{3};
    bool doMerge{false};
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

class LinesTracker
{
public:
    LinesTracker();
    ~LinesTracker();

    // delete copy assignment and copy constructor
    LinesTracker(const LinesTracker&) = delete;
    LinesTracker operator=(const LinesTracker&) = delete;
    LinesTracker(LinesTracker&& ) = delete;

    bool trackLines(std::shared_ptr<PinHoleCamera> camera,
                    const cv::Mat& prevImg,
                    const cv::Mat& currImg,
                    TrackedLines2D& trackedLines);
private:
    template <typename T>
    void reduceLinesVector(const std::vector<uchar> status,
                           std::vector<T>& linesV)
    {
        std::size_t jdx{0};
        for (std::size_t idx = 0; idx < status.size(); idx+=2)
        {
            if (static_cast<int>(status[idx]) && static_cast<int>(status[idx+1]))
            {
                linesV[jdx++] = linesV[idx/2]; // places active features in the front of the vector
            }
        }

        linesV.resize(jdx); // remove the last inactive features
    }

    void setMask(std::shared_ptr<PinHoleCamera> camera);

    cv::Ptr<cv::ximgproc::FastLineDetector> m_fld{nullptr};
    FLDParameters m_fldParams;
    long long int m_featureID{0};

    cv::Mat m_mask;
    std::vector<cv::Vec4f> m_detectedLines;
    std::vector<Line2D> m_forwardedLines, m_prevLines, m_newLines;

    std::vector<long long int> m_trackedIDs;
    std::vector<long long int> m_trackCount;

    // TODO move these hard-coded parameters as configurable parameters
    int m_trackingPixelErrThreshold{3}; // pixel error used for checking bi-directional optical flow

    int m_minDistanceBetweenFeatures{30}; // choose this size (in pixels) such that mask applied does not introduce spuriously detected lines, preferably above 15
    int m_borderSize{1}; // image border size to check for interior tracked features in pixels

    // tracker parameters
    int m_maxPyramidLevel{4}; // 0-based levels of sub-image pyramids for KLT tracker
    cv::Size m_searchWindowSize{cv::Size(15, 15)}; // window size in pixels for KLT tracker
};

} // namespace Perception
} // namespace KinDynVIO

#endif // KINDYNVIO_PERECEPTION_FEATURES_LINES_TRACKER_H
