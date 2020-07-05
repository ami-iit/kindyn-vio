
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

    }
}

#endif

