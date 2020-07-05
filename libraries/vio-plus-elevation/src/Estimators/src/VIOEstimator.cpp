#include "Estimators/VIOEstimator.h"

namespace Viper
{
namespace Estimators
{
VIOEstimator::VIOEstimator()
{
    m_gravity.zero();
    m_gravity(2) = -9.80665;
}

}
}
