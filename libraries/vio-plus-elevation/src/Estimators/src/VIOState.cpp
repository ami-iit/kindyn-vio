
#include "Estimators/VIOState.h"
#include <iDynTree/Core/EigenHelpers.h>

namespace Viper
{
namespace Estimators
{

void IMUWithBiasState::init()
{
    A_R_IMU = iDynTree::Rotation::Identity();
    A_p_IMU.zero();
    A_v_IMU.zero();
    bias_acc.zero();
    bias_gyro.zero();
}

bool IMUWithBiasState::propagate(const IMUObservation& obs,
                                 const iDynTree::Vector3& g)
{
    using iDynTree::toEigen;
    iDynTree::LinearMotionVector3 acc_unbiased;
    iDynTree::AngVelocity omega_unbiased;
    toEigen(acc_unbiased) = toEigen(obs.acc_meas) - toEigen(bias_acc);
    toEigen(omega_unbiased) = toEigen(obs.gyro_meas) - toEigen(bias_gyro);

    iDynTree::Vector3 body_acc;
    toEigen(body_acc) = (toEigen(A_R_IMU)*toEigen(acc_unbiased)) + toEigen(g);
    std::cout << "Body acc: " << body_acc.toString() << std::endl;
    A_R_IMU = A_R_IMU*(omega_unbiased.exp());
    toEigen(A_p_IMU) = toEigen(A_p_IMU) + (toEigen(A_v_IMU)*obs.dt) + (toEigen(body_acc)*obs.dt*obs.dt*0.5);
    toEigen(A_v_IMU) = toEigen(A_v_IMU) + (toEigen(body_acc)*obs.dt);

    // bias remains the same
    return true;
}

void IMUWithBiasState::print()
{
    std::cout << "IMU Orientation RPY (rad): " << A_R_IMU.asRPY().toString() << std::endl;
    std::cout << "IMU Position (m): " << A_p_IMU.toString() << std::endl;
    std::cout << "IMU Velocity (m/s): " << A_v_IMU.toString() << std::endl;
    std::cout << "IMU Accelerometer bias (m/s^2): " << bias_acc.toString() << std::endl;
    std::cout << "IMU Gyroscope bias (rad/s): " << bias_gyro.toString() << std::endl;
}

} // end namespace Estimators
} // end namespace Viper



