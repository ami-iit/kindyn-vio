/**
 * @file DataTypes.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNVIO_PERECEPTION_FEATURES_DATA_TYPES_H
#define KINDYNVIO_PERECEPTION_FEATURES_DATA_TYPES_H

#include <opencv2/opencv.hpp>

namespace KinDynVIO
{
namespace Perception
{

struct TimeStampedImg
{
    double ts{-1.0};
    cv::Mat img;
};

} // namespace Perception
} // namespace KinDynVIO

#endif // KINDYNVIO_PERECEPTION_FEATURES_DATA_TYPES_H
