
#ifndef VIO_OBSERVATION_H
#define VIO_OBSERVATION_H

#include <iDynTree/Core/VectorFixSize.h>

#include <iostream>

namespace Viper
{
    namespace Estimators
    {
        class IMUObservation
        {
        public:
            iDynTree::LinearMotionVector3 acc_meas;
            iDynTree::AngVelocity gyro_meas;
            double dt{0.0}; // sampling time

            void print()
            {
                std::cout << "Accelerometer measurement: " << acc_meas.toString() << std::endl;
                std::cout << "Gyroscope measurement: " << gyro_meas.toString() << std::endl;
                std::cout << "Sampling Time: " << dt << " s." << std::endl;
            }
        };

        class IMUSensorStdDev
        {
        public:
            iDynTree::Vector3 sigma_a; // accelerometer Gaussian white noise deviation
            iDynTree::Vector3 sigma_g; // gyroscope Gaussian white noise deviation
            iDynTree::Vector3 sigma_ba; // accelerometer bias random walk noise deviation
            iDynTree::Vector3 sigma_bg; // gyroscope bias random walk noise deviation
        };

    }
}

#endif

