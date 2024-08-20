/**
 * @file DataTypes.h
 * @authors Prashanth Ramadoss
 * @copyright Fondazione Istituto Italiano di Tecnologia (IIT). SPDX-License-Identifier: BSD-3-Clause
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
