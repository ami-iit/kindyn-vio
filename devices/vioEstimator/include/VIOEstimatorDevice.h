#ifndef VIO_ESTIMATOR_DEVICE_H
#define VIO_ESTIMATOR_DEVICE_H

#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/Wrapper.h>

#include <yarp/sig/Image.h>
#include <yarp/cv/Cv.h>

#include <YARPRobotsHelper/RobotSensorBridge.h>
#include <YARPRobotsHelper/YarpHelper.hpp>

#include <iDynTree/Core/VectorFixSize.h>

#include <mutex>
#include <opencv2/opencv.hpp>

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

        private:
            bool updateInertialBuffers();
            bool updateImageBuffers();

            struct
            {
                std::string d435i_gyro_name{"/gyro_sensor"};
                std::string d435i_acc_name{"/accelerations_sensor"};
                iDynTree::Vector3 d435i_gyro_vec;
                iDynTree::Vector3 d435i_acc_vec;
                double imu_recv_time_in_s{-1.0};
            } m_inertial_buffer;

            struct
            {
                std::string d435i_cam_name{"d435i_depthcam"};
                cv::Mat rgb_image;
                cv::Mat depth_image;
                double rgb_recv_time_in_s{-1.0};
                double depth_recv_time_in_s{-1.0};
            }m_visual_buffer;

            std::unique_ptr<YarpHelper::RobotSensorBridge> m_robot_sensor_bridge;
            std::mutex m_device_mutex;


        };
    }
}

#endif

