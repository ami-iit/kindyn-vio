#ifndef VIO_ESTIMATOR_DEVICE_H
#define VIO_ESTIMATOR_DEVICE_H

#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/Wrapper.h>

#include <YARPRobotsHelper/RobotSensorBridge.h>

namespace wbe
{
    namespace dev
    {
        class VIOEstimatorDevice : public yarp::dev::DeviceDriver,
                                   public yarp::dev::IMultipleWrapper,
                                   public yarp::os::PeriodicThread
        {
        public:
            explicit VIOEstimatorDevice(double period, yarp::os::ShouldUseSystemClock useSystemClock = yarp::os::ShouldUseSystemClock::No);
            VIOEstimatorDevice();
            ~VIOEstimatorDevice();

            virtual bool open(yarp::os::Searchable& config);
            virtual bool close();
            virtual bool attachAll(const yarp::dev::PolyDriverList & poly);
            virtual bool detachAll();
            virtual void run();
        };
    }
}

#endif

