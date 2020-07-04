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
    yarp::os::Bottle sensor_bridge_config;
    if (!YarpHelper::extractGroupFromYarpConfig(config, "RobotSensorBridge", sensor_bridge_config))
    {
        yError() << "[VIOEstimatorDevice]: Could not find \"RobotSensorBridge\" group in configuration file. Cannot load estimator";
        return false;
    }

    m_robot_sensor_bridge = std::make_unique<YarpHelper::RobotSensorBridge>();
    if (!m_robot_sensor_bridge->configureRobot(sensor_bridge_config))
    {
        return false;
    }

    return true;
}

bool wbe::dev::VIOEstimatorDevice::attachAll(const yarp::dev::PolyDriverList& poly)
{
    if (!m_robot_sensor_bridge->attachInterfaces(poly))
    {
        yError() << "[VIOEstimatorDevice]: Could not attach to other devices.";
        return false;
    }

    this->start();
    return true;
}

void wbe::dev::VIOEstimatorDevice::run()
{
}

bool wbe::dev::VIOEstimatorDevice::detachAll()
{
    std::lock_guard<std::mutex> guard(m_device_mutex);
    this->stop();
    return true;
}

bool wbe::dev::VIOEstimatorDevice::close()
{
    return true;
}

