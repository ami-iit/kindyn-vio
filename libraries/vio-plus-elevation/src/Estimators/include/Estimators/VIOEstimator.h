
#ifndef VIO_ESTIMATOR_H
#define VIO_ESTIMATOR_H

#include "Estimators/VIOState.h"
#include "Estimators/VIOObservation.h"
#include <iostream>

namespace Viper
{
    namespace Estimators
    {
        class VIOEstimatorOptions
        {
            size_t Ncam_max{5};
        };

        class VIOEstimator
        {
        public:
            VIOEstimator();
        private:
            bool predictCameraStates(const IMUWithBiasState& imu_state, CameraState& cam_state);

            VIOState m_X;
            VIOEstimatorOptions m_opt;

            iDynTree::Vector3 m_gravity;
        };
    }
}

#endif
