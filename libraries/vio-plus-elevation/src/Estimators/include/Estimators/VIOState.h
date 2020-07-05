
#ifndef VIO_STATE_H
#define VIO_STATE_H

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/MatrixDynSize.h>

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
            iDynTree::LinVelocity A_v_IMU;
            iDynTree::Position A_p_IMU;
            iDynTree::Vector3 bias_gyro;
            iDynTree::Vector3 bias_acc;

            void init();
            bool propagate(const IMUObservation& obs, const iDynTree::Vector3& g);

            // discrete error propagation matrices
            void calcPhik(const iDynTree::Vector3& g, const double& dt,
                          const bool& estimate_bias, iDynTree::MatrixDynSize&Phik);
            bool calcQk(const IMUSensorStdDev& std_dev, const iDynTree::MatrixDynSize& Phik,
                        const double& dt, const bool& estimate_bias, iDynTree::MatrixDynSize& Qk);

            void print();
        private:
            void calcAdjointSE_2_3(iDynTree::MatrixFixSize<9, 9>& AdjX_wo_bias);
            void stddev2diagMat(const IMUSensorStdDev& std_dev,
                                const bool& estimate_bias, iDynTree::MatrixDynSize& Cov_w);

            void calcFc(const iDynTree::Vector3& g, const bool& estimate_bias,
                        iDynTree::MatrixDynSize& Fc);
            void calcLc(const bool& estimate_bias, iDynTree::MatrixDynSize& Lc);
            void calcQc(const IMUSensorStdDev& std_dev, const bool& estimate_bias,
                              iDynTree::MatrixDynSize& Qc);

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

