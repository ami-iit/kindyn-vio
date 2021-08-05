/**
 * @file ImageProcessor.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <KinDynVIO/Perception/Features/ImageProcessor.h>

using namespace KinDynVIO::Perception;
using namespace BipedalLocomotion::ParametersHandler;

enum class TrackerType
{
    POINTS,
    LINES,
    POINTS_AND_LINES
};

class ImageProcessor::Impl
{
public:
    bool trackPoints();
    bool trackLines();

    void drawPoints(const cv::Mat& img,
                    const TrackedPoints2D& points,
                    const bool& printMetaDataconst,
                    const int& windowSize = 10,
                    const int& radius = 2,
                    const int& thickness = 2,
                    const double& fontScale = 0.35);
    void drawLines(const cv::Mat& img,
                   const TrackedLines2D& trackedLines,
                   const bool& printMetaData,
                   const int& windowSize = 10,
                   const int& thickness = 2,
                   const double& fontScale = 0.35);

    bool initialized{false};
    bool debug{true};
    bool equalizeImg{true};
    TrackerType trackerType{TrackerType::POINTS};

    // CLAHE parameters
    cv::Ptr<cv::CLAHE> clahe;
    double claheClipLimit{3.0}; // CLAHE threshold for contrast limiting
    cv::Size claheTileSize{cv::Size(8, 8)};

    // drawing parameters
    int drawnTrackedFeatureWindowSize{10}; // size required to interpolate feature color from B/G to
                                           // R
    int drawnFeatureRadius{2}; // radius for drawn point features in pixels
    int drawnFeatureThickness{2}; // thickness of drawn features in pixels
    double drawnFontScale{0.35}; // font scale for accompanying text

    TimeStampedImg currImg, prevImg;

    PointsTracker ptsTracker;
    LinesTracker linesTracker;
    TrackedFeatures features;

    std::shared_ptr<PinHoleCamera> camera{nullptr};
};

ImageProcessor::ImageProcessor()
    : m_pimpl(std::make_unique<ImageProcessor::Impl>())
{
}

ImageProcessor::~ImageProcessor()
{
}

bool ImageProcessor::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    const std::string printPrefix{"[ImageProcessor::initialize]"};
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        BipedalLocomotion::log()->error("{} The parameter handler has expired. "
                                        "Please check its scope.",
                                        printPrefix);
        return false;
    }

    std::string tracker;
    if (handle->getParameter("tracker_type", tracker))
    {
        std::vector<std::string> options{"points", "lines", "points_and_lines"};
        if (tracker == options[0])
        {
            m_pimpl->trackerType = TrackerType::POINTS;
        } else if (tracker == options[1])
        {
            m_pimpl->trackerType = TrackerType::LINES;
        } else if (tracker == options[2])
        {
            m_pimpl->trackerType = TrackerType::POINTS_AND_LINES;
        } else
        {
            std::cerr << printPrefix << "Setting the tracker type to default as \"points\"."
                      << std::endl;
        }
    }

    handle->getParameter("equalize_image", m_pimpl->equalizeImg);
    m_pimpl->clahe = cv::createCLAHE(m_pimpl->claheClipLimit, m_pimpl->claheTileSize);

    handle->getParameter("debug", m_pimpl->debug);
    handle->getParameter("drawn_tracked_feature_window_size",
                         m_pimpl->drawnTrackedFeatureWindowSize);
    handle->getParameter("drawn_feature_radius", m_pimpl->drawnFeatureRadius);
    handle->getParameter("drawn_feature_thickness", m_pimpl->drawnFeatureThickness);
    handle->getParameter("drawn_font_scale", m_pimpl->drawnFontScale);

    m_pimpl->initialized = true;
    return true;
}

bool ImageProcessor::setCameraModel(std::shared_ptr<PinHoleCamera> camera)
{
    const std::string printPrefix{"[ImageProcessor::setCameraModel]"};
    if (camera == nullptr)
    {
        BipedalLocomotion::log()->error("{} Invalid pointer to camera model.", printPrefix);
        return false;
    }

    m_pimpl->camera = camera;

    return true;
}

bool ImageProcessor::setImage(const cv::Mat& img, const double& receiveTimeInSeconds)
{
    std::string printPrefix{"[ImageProcessor::setImage]"};
    if (!m_pimpl->initialized)
    {
        BipedalLocomotion::log()->error("{} Please call initialize first", printPrefix);
        return false;
    }

    if (img.channels() == 3)
    {
        cv::cvtColor(img, m_pimpl->currImg.img, cv::COLOR_RGB2GRAY);
    } else if (img.channels() == 4)
    {
        cv::cvtColor(img, m_pimpl->currImg.img, cv::COLOR_RGBA2GRAY);
    } else
    {
        m_pimpl->currImg.img = img;
    }

    m_pimpl->currImg.ts = receiveTimeInSeconds;

    auto begin = BipedalLocomotion::clock().now();
    if (m_pimpl->equalizeImg)
    {
        m_pimpl->clahe->apply(m_pimpl->currImg.img, m_pimpl->currImg.img);
        if (m_pimpl->debug)
        {
            auto end = BipedalLocomotion::clock().now();
            BipedalLocomotion::log()->info("{} CLAHE took {} seconds.",
                                           printPrefix,
                                           (end - begin).count());
        }
    }

    return true;
}

bool ImageProcessor::setInput(const TimeStampedImg& stampedImg)
{
    return this->setImage(stampedImg.img, stampedImg.ts);
}

bool ImageProcessor::advance()
{
    std::string printPrefix{"[ImageProcessor::advance]"};
    if (!m_pimpl->initialized)
    {
        BipedalLocomotion::log()->error("{} Please call initialize first", printPrefix);
        return false;
    }

    if (m_pimpl->currImg.img.empty())
    {
        BipedalLocomotion::log()->error("{} Please set image first", printPrefix);
        return false;
    }

    // if first frame - just detect images
    if (m_pimpl->prevImg.img.empty())
    {
        // this is the first image
        m_pimpl->prevImg.img
            = cv::Mat(m_pimpl->camera->rows(), m_pimpl->camera->cols(), CV_8UC1, cv::Scalar(0));
    }

    if (m_pimpl->trackerType == TrackerType::POINTS
        || m_pimpl->trackerType == TrackerType::POINTS_AND_LINES)
    {
        if (!m_pimpl->trackPoints())
        {
            BipedalLocomotion::log()->error("{} Failed to track point features.", printPrefix);
            return false;
        }
    }

    if (m_pimpl->trackerType == TrackerType::LINES
        || m_pimpl->trackerType == TrackerType::POINTS_AND_LINES)
    {
        if (!m_pimpl->trackLines())
        {
            BipedalLocomotion::log()->error("{} Failed to track line features.", printPrefix);
            return false;
        }
    }

    m_pimpl->currImg.img.copyTo(m_pimpl->prevImg.img);
    m_pimpl->prevImg.ts = m_pimpl->currImg.ts;

    return true;
}

bool ImageProcessor::Impl::trackPoints()
{
    std::string printPrefix{"[ImageProcessor::Impl::trackPoints]"};
    auto begin = BipedalLocomotion::clock().now();

    if (!ptsTracker.trackPoints(camera, prevImg.img, currImg.img, features.points))
    {
        return false;
    }

    if (debug)
    {
        auto end = BipedalLocomotion::clock().now();
        BipedalLocomotion::log()->info("{} Tracking point features took {} seconds.",
                                       printPrefix,
                                       (end - begin).count());
    }
    return true;
}

bool ImageProcessor::Impl::trackLines()
{
    std::string printPrefix{"[ImageProcessor::Impl::trackLines]"};
    auto begin = BipedalLocomotion::clock().now();
    if (!linesTracker.trackLines(camera, prevImg.img, currImg.img, features.lines))
    {
        return false;
    }

    if (debug)
    {
        auto end = BipedalLocomotion::clock().now();
        BipedalLocomotion::log()->info("{} Tracking line features took {} seconds.",
                                       printPrefix,
                                       (end - begin).count());
    }
    return true;
}

const TrackedFeatures& ImageProcessor::getOutput() const
{
    return m_pimpl->features;
}

bool ImageProcessor::isOutputValid() const
{
    return m_pimpl->initialized;
}

bool ImageProcessor::getImageWithDetectedFeatures(cv::Mat& outImg)
{
    cv::cvtColor(m_pimpl->currImg.img, outImg, cv::COLOR_GRAY2RGB);
    if (m_pimpl->trackerType == TrackerType::POINTS
        || m_pimpl->trackerType == TrackerType::POINTS_AND_LINES)
    {
        m_pimpl->drawPoints(outImg,
                            m_pimpl->features.points,
                            m_pimpl->debug,
                            m_pimpl->drawnTrackedFeatureWindowSize,
                            m_pimpl->drawnFeatureRadius,
                            m_pimpl->drawnFeatureThickness,
                            m_pimpl->drawnFontScale);
    }

    if (m_pimpl->trackerType == TrackerType::LINES
        || m_pimpl->trackerType == TrackerType::POINTS_AND_LINES)
    {
        m_pimpl->drawLines(outImg,
                           m_pimpl->features.lines,
                           m_pimpl->debug,
                           m_pimpl->drawnTrackedFeatureWindowSize,
                           m_pimpl->drawnFeatureThickness,
                           m_pimpl->drawnFontScale);
    }

    return true;
}

void ImageProcessor::Impl::drawPoints(const cv::Mat& img,
                                      const TrackedPoints2D& points,
                                      const bool& printMetaData = true,
                                      const int& windowSize,
                                      const int& radius,
                                      const int& thickness,
                                      const double& fontScale)
{
    for (std::size_t jdx = 0; jdx < points.uvs.size(); jdx++)
    {
        double len{std::min(1.0, 1.0 * points.counts[jdx] / windowSize)};
        auto blue2red = cv::Scalar(255 * (1 - len), 0, 255 * len);
        cv::circle(img, points.uvs[jdx], radius, blue2red, thickness);

        if (printMetaData)
        {
            // add text - feature ID and track count
            auto font = cv::FONT_HERSHEY_SIMPLEX;
            std::string id{"P" + std::to_string(points.ids[jdx]) + ", "
                           + std::to_string(points.counts[jdx])};

            cv::putText(img, id, points.uvs[jdx] + cv::Point2f(2, 0), font, fontScale, blue2red);
        }
    }
}

void ImageProcessor::Impl::drawLines(const cv::Mat& img,
                                     const TrackedLines2D& trackedLines,
                                     const bool& printMetaData,
                                     const int& windowSize,
                                     const int& thickness,
                                     const double& fontScale)
{
    const auto& nrLines = trackedLines.lines.size();
    for (std::size_t jdx = 0; jdx < nrLines; jdx++)
    {
        const auto& line = trackedLines.lines[jdx];
        double len{std::min(1.0, 1.0 * trackedLines.counts[jdx] / windowSize)};
        auto green2red = cv::Scalar(0, 255 * (1 - len), 255 * (len));
        cv::line(img, line.startPixelCoord, line.endPixelCoord, green2red, thickness);

        if (printMetaData)
        {
            // add text - feature ID and track count
            auto font = cv::FONT_HERSHEY_SIMPLEX;
            std::string id{"L" + std::to_string(trackedLines.ids[jdx]) + ", "
                           + std::to_string(trackedLines.counts[jdx])};

            cv::putText(img,
                        id,
                        line.startPixelCoord - cv::Point2f(10, 2),
                        font,
                        fontScale,
                        green2red);
        }
    }
}
