
#ifndef VIO_STATE_H
#define VIO_STATE_H

#include <iDynTree/Core/Transform.h>
#include <unordered_map>

#include "Estimators/VIOObservation.h"

namespace Viper
{
    namespace Estimators
    {
        class IMUWithBiasState
        {
        public:
            iDynTree::Rotation A_R_IMU;
            iDynTree::Position A_p_IMU;
            iDynTree::LinVelocity A_v_IMU;
            iDynTree::Vector3 bias_acc;
            iDynTree::Vector3 bias_gyro;

            void init();
            bool propagate(const IMUObservation& obs, const iDynTree::Vector3& g);
            void print();
        };

        class CameraState
        {
        public:
            iDynTree::Rotation A_R_cam;
            iDynTree::Position A_p_cam;

            void init()
            {
                A_R_cam = iDynTree::Rotation::Identity();
                A_p_cam.Zero();
            }
        };

        class VIOState
        {
        public:
            IMUWithBiasState X_imu;
            std::unordered_map<size_t, CameraState> X_cam;
        };
    }
}

#endif

