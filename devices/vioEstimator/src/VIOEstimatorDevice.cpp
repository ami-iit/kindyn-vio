#include "VIOEstimatorDevice.h"

wbe::dev::VIOEstimatorDevice::VIOEstimatorDevice(double period,
                                                  yarp::os::ShouldUseSystemClock useSystemClock): yarp::os::PeriodicThread(period, useSystemClock)
{
}

wbe::dev::VIOEstimatorDevice::VIOEstimatorDevice() : yarp::os::PeriodicThread(0.01, yarp::os::ShouldUseSystemClock::No)
{
}

wbe::dev::VIOEstimatorDevice::~VIOEstimatorDevice()
{
}


bool wbe::dev::VIOEstimatorDevice::open(yarp::os::Searchable& config)
{
    return true;
}

bool wbe::dev::VIOEstimatorDevice::attachAll(const yarp::dev::PolyDriverList& poly)
{
    return true;
}

void wbe::dev::VIOEstimatorDevice::run()
{
}

bool wbe::dev::VIOEstimatorDevice::detachAll()
{
    return true;
}

bool wbe::dev::VIOEstimatorDevice::close()
{
    return true;
}

