#include <Estimators/VIOState.h>
#include <Estimators/VIOObservation.h>

#include <iostream>

bool testIMUPropagationModel()
{
    Viper::Estimators::IMUWithBiasState imu_state;
    Viper::Estimators::IMUObservation imu_obs;

    // We are in the Earth
    iDynTree::Vector3 g;
    g.zero();
    g(2) = -9.80665;

    // stationary IMU with Z-axis pointing upwards
    imu_state.init();

    std::cout << "======Stationary IMU with Z-axis pointing upwards====" << std::endl;
    imu_state.print();
    std::cout << "======================================================" << std::endl;

    // set IMU observation measuring only acceleration due to gravity
    imu_obs.dt = 0.01;
    imu_obs.acc_meas.zero();
    imu_obs.acc_meas(2) = -g(2);
    imu_obs.gyro_meas.zero();

    std::cout << "======IMU observation measuring only acceleration due to gravity====" << std::endl;
    imu_obs.print();
    std::cout << "======================================================" << std::endl;

    auto imu_prev_state = imu_state;
    imu_state.propagate(imu_obs, g);

    std::cout << "======IMU state propagation====" << std::endl;
    imu_state.print();
    std::cout << "======================================================" << std::endl;

    return true;
}

int main(int argc, char** argv)
{
    if (!testIMUPropagationModel())
    {
        std::cerr << "VIO testIMUPropagationModel Failed." << std::endl;
    }

    return 0;
}
