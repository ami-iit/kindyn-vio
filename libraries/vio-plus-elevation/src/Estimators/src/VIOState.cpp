
#include "Estimators/VIOState.h"
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/Utils.h>

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

void IMUWithBiasState::calcPhik(const iDynTree::Vector3& g,
                                const double& dt,
                                const bool& estimate_bias,
                                iDynTree::MatrixDynSize& Phik)
{
    iDynTree::MatrixDynSize Fc;
    calcFc(g, estimate_bias, Fc);
    auto vdim = Fc.rows();
    Phik.resize(vdim, vdim);
    toEigen(Phik) = Eigen::MatrixXd::Identity(vdim, vdim) + (toEigen(Fc)*dt);
}

bool IMUWithBiasState::calcQk(const IMUSensorStdDev& std_dev, const iDynTree::MatrixDynSize& Phik,
                                          const double& dt, const bool& estimate_bias,
                                          iDynTree::MatrixDynSize& Qk)
{
    if (estimate_bias)
    {
        if (!(Phik.rows() == 15 && Phik.cols() == 15))
        {
            std::cerr << "[Error] System state Jacobian size mismatch" << std::endl;
            return false;
        }
    }
    else
    {
        if (!(Phik.rows() == 9 && Phik.cols() == 9))
        {
            std::cerr << "[Error] System state Jacobian size  mismatch" << std::endl;
            return false;
        }
    }

    iDynTree::MatrixDynSize Qc;
    calcQc(std_dev, estimate_bias, Qc);
        std::cout << Phik.toString() << std::endl;

    Qk.resize(Qc.rows(), Qc.cols());
    toEigen(Qk) = dt*toEigen(Phik)*toEigen(Qc)*(toEigen(Phik).transpose());
}

void IMUWithBiasState::calcFc(const iDynTree::Vector3& g, const bool& estimate_bias,
                              iDynTree::MatrixDynSize& Fc)
{
    using iDynTree::toEigen;

    estimate_bias ? Fc.resize(15, 15) : Fc.resize(9, 9) ;
    auto F{toEigen(Fc)};

    // epsilon = (rot, vel, pos, bias_acc, bias_gyro)
    // dfrot_depsilon
    F.block(0, 0, 3, 9).setZero();

    // dfvel_depsilon
    F.block(3, 0, 3, 3) = iDynTree::skew(toEigen(g));
    F.block(3, 3, 3, 6).setZero();

    // dfpos_depsilon
    F.block(6, 0, 3, 3).setZero();
    F.block(6, 3, 3, 3) = Eigen::Matrix3d::Identity();
    F.block(6, 6, 3, 3).setZero();

    if (estimate_bias)
    {
        // dfrot_dbias
        F.block(0, 9, 3, 3) = -toEigen(A_R_IMU);
        F.block(0, 12, 3, 3).setZero();

        // dfvel_dbias
        F.block(3, 9, 3, 3) = -iDynTree::skew(toEigen(A_v_IMU))*toEigen(A_R_IMU);
        F.block(3, 12, 3, 3) = -toEigen(A_R_IMU);

        // dfpos_dbias
        F.block(6, 9, 3, 3) = -iDynTree::skew(toEigen(A_p_IMU))*toEigen(A_R_IMU);
        F.block(6, 12, 3, 3).setZero();

        // dfbias_depsilon
        F.block(9, 0, 6, 15).setZero();
    }
}


void IMUWithBiasState::calcAdjointSE_2_3(iDynTree::MatrixFixSize<9, 9>& AdjX_wo_bias)
{
    using iDynTree::toEigen;
    auto Ad{toEigen(AdjX_wo_bias)};

    // rot
    Ad.block(0, 0, 3, 3) = toEigen(A_R_IMU);
    Ad.block(0, 3, 3, 6).setZero();

    // vel
    Ad.block(3, 0, 3, 3) = iDynTree::skew(toEigen(A_v_IMU))*toEigen(A_R_IMU);
    Ad.block(3, 3, 3, 3) = toEigen(A_R_IMU);
    Ad.block(3, 6, 3, 3).setZero();

    // pos
    Ad.block(6, 0, 3, 3) = iDynTree::skew(toEigen(A_p_IMU))*toEigen(A_R_IMU);
    Ad.block(6, 3, 3, 3).setZero();
    Ad.block(6, 6, 3, 3) = toEigen(A_R_IMU);
}

void IMUWithBiasState::calcLc(const bool& estimate_bias,
                              iDynTree::MatrixDynSize& Lc)
{
    using iDynTree::toEigen;
    estimate_bias ? Lc.resize(15, 15) : Lc.resize(9, 9) ;
    auto Lc_eig{toEigen(Lc)};

    iDynTree::MatrixFixSize<9, 9> Lc_wo_bias;
    calcAdjointSE_2_3(Lc_wo_bias);
    Lc_eig.block(0, 0, 9, 9) = toEigen((Lc_wo_bias));

    if (estimate_bias)
    {
        Lc_eig.block(0, 0, 9, 9) = toEigen((Lc_wo_bias));

        Lc_eig.block(0, 9, 9, 6).setZero();
        Lc_eig.block(9, 0, 6, 9).setZero();

        Lc_eig.block(9, 9, 3, 3) = Eigen::Matrix3d::Identity();
        Lc_eig.block(12, 12, 3, 3) = Eigen::Matrix3d::Identity();
    }
}

void IMUWithBiasState::calcQc(const IMUSensorStdDev& std_dev, const bool& estimate_bias,
                              iDynTree::MatrixDynSize& Qc)
{
    using iDynTree::toEigen;

    iDynTree::MatrixDynSize Lc;
    calcLc(estimate_bias, Lc);

    iDynTree::MatrixDynSize Cov_w;
    stddev2diagMat(std_dev, estimate_bias, Cov_w);

    estimate_bias ? Qc.resize(15, 15) : Qc.resize(9, 9);
    toEigen(Qc) = toEigen(Lc) * toEigen(Cov_w) * (toEigen(Lc).transpose());
}

void IMUWithBiasState::stddev2diagMat(const IMUSensorStdDev& std_dev,
                                      const bool& estimate_bias,
                                      iDynTree::MatrixDynSize& Cov_w)
{
    using iDynTree::toEigen;
    estimate_bias ? Cov_w.resize(15, 15) : Cov_w.resize(9, 9) ;

    auto W{toEigen(Cov_w)};
    // rot
    W.block(0, 0, 3, 3) = toEigen(std_dev.sigma_g).asDiagonal();
    W.block(0, 3, 3, 6).setZero();

    // vel
    W.block(3, 0, 3, 3).setZero();
    W.block(3, 3, 3, 3) = toEigen(std_dev.sigma_a).asDiagonal();
    W.block(3, 6, 3, 3).setZero();

    // pos
    W.block(6, 0, 3, 9).setZero();

    if (estimate_bias)
    {
        W.block(0, 9, 9, 6).setZero();
        W.block(9, 0, 6, 9).setZero();

        W.block(9, 9, 3, 3) = toEigen(std_dev.sigma_bg).asDiagonal();
        W.block(12, 12, 3, 3) = toEigen(std_dev.sigma_ba).asDiagonal();
    }
}

} // end namespace Estimators
} // end namespace Viper



