#include "VIOEstimatorDevice.h"
#include <iDynTree/Core/EigenHelpers.h>

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

    this->setPeriod(0.01);
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
    std::lock_guard<std::mutex> guard(m_device_mutex);
    this->updateInertialBuffers();
}

bool wbe::dev::VIOEstimatorDevice::updateInertialBuffers()
{
    auto ok{true};
    // these reads are pretty slow because underlying realsensewithIMU driver is performing a blocking read
    // This needs to be optiized. Profiling was done to check if RobotSensorBridge is adding a overload,
    // but since the number of sensors in the map data structure is less, it does not add any footprint
    ok = ok && m_robot_sensor_bridge->getGyroscopeMeasure(m_inertial_buffer.d435i_gyro_name,
                                                          m_inertial_buffer.d435i_gyro_vec.data(),
                                                          m_inertial_buffer.imu_recv_time_in_s);

    ok = ok && m_robot_sensor_bridge->getLinearAccelerometerMeasure(m_inertial_buffer.d435i_acc_name,
                                                                    m_inertial_buffer.d435i_acc_vec.data(),
                                                                    m_inertial_buffer.imu_recv_time_in_s);

    return true;
}

bool wbe::dev::VIOEstimatorDevice::updateImageBuffers()
{
     auto ok{true};
    yarp::sig::FlexImage realsense_out;
    ok = ok && m_robot_sensor_bridge->getRgbImage(m_visual_buffer.d435i_cam_name, realsense_out, m_visual_buffer.rgb_recv_time_in_s);
    return true;
}


bool wbe::dev::VIOEstimatorDevice::detachAll()
{
    std::lock_guard<std::mutex> guard(m_device_mutex);
    this->stop();
    return true;
}

bool wbe::dev::VIOEstimatorDevice::close()
{
    std::lock_guard<std::mutex> guard(m_device_mutex);
    return true;
}

