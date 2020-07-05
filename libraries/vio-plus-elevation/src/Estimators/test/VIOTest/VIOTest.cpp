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

bool testIMUJacobians()
{
    Viper::Estimators::IMUWithBiasState imu_state;

    // We are in the Earth
    iDynTree::Vector3 g;
    g.zero();
    g(2) = -9.80665;

    // sensor noise parameters
    Viper::Estimators::IMUSensorStdDev std_dev;
    std_dev.sigma_g(0) = 1; std_dev.sigma_g(1) = 1; std_dev.sigma_g(2) = 1;
    std_dev.sigma_a(0) = 1; std_dev.sigma_a(1) = 1; std_dev.sigma_a(2) = 1;
    std_dev.sigma_bg(0) = 1; std_dev.sigma_bg(1) = 1; std_dev.sigma_bg(2) = 1;
    std_dev.sigma_ba(0) = 1; std_dev.sigma_ba(1) = 1; std_dev.sigma_ba(2) = 1;

    double dt{0.01};

    // stationary IMU with Z-axis pointing upwards
    imu_state.init();

    std::cout << "======Stationary IMU with Z-axis pointing upwards====" << std::endl;
    imu_state.print();
    std::cout << "======================================================" << std::endl;

    std::cout << "======IMU Phik without bias====" << std::endl;
    iDynTree::MatrixDynSize Fk;
    imu_state.calcPhik(g, dt, /*estimate_bias=*/ false,Fk);
    std::cout << Fk.toString() << std::endl;
    std::cout << "======================================================" << std::endl;

    std::cout << "======IMU Phik with bias====" << std::endl;
    iDynTree::MatrixDynSize Fk_b;
    imu_state.calcPhik(g, dt, /*estimate_bias=*/ true, Fk_b);
    std::cout << Fk_b.toString() << std::endl;
    std::cout << "======================================================" << std::endl;

    std::cout << "======Discrete system noise covariance without bias====" << std::endl;
    iDynTree::MatrixDynSize Qk;
    imu_state.calcQk(std_dev, Fk, dt, /*estimate_bias=*/ false,  Qk);
    std::cout << Qk.toString() << std::endl;
    std::cout << "======================================================" << std::endl;

    std::cout << "======Discrete system noise covariance with bias====" << std::endl;
    iDynTree::MatrixDynSize Qk_b;
    imu_state.calcQk(std_dev, Fk_b, dt, /*estimate_bias=*/ true,  Qk_b);
    std::cout << Qk_b.toString() << std::endl;
    std::cout << "======================================================" << std::endl;

    return true;
}

int main(int argc, char** argv)
{
    if (!testIMUPropagationModel())
    {
        std::cerr << "VIO testIMUPropagationModel Failed." << std::endl;
    }

    if (!testIMUJacobians())
    {
        std::cerr << "VIO testIMUPropagationModel Failed." << std::endl;
    }

    return 0;
}
