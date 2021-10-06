/**
 * @file Utils.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNVIO_PERECEPTION_FEATURES_UTILS_H
#define KINDYNVIO_PERECEPTION_FEATURES_UTILS_H

#include <vector>

namespace KinDynVIO
{
namespace Perception
{

template <typename T>
void reducePointsVector(const std::vector<uchar>& status,
                        std::vector<T>& v)
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

template <typename T>
void reduceLinesVector(const std::vector<uchar>& status,
                        std::vector<T>& linesV)
{
    std::size_t jdx{0};
    for (std::size_t idx = 0; idx < status.size(); idx+=2)
    {
        if (static_cast<int>(status[idx]) &&
            static_cast<int>(status[idx+1]))
        {
            linesV[jdx++] = linesV[idx/2]; // places active features in the front of the vector
        }
    }

    linesV.resize(jdx); // remove the last inactive features
}

} // namespace Perception
} // namespace KinDynVIO

#endif // KINDYNVIO_PERECEPTION_FEATURES_UTILS_H
