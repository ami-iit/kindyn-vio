/**
 * @file CentroidalDynamicsPreintegrationBase.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNVIO_FACTORS_CENTROIDAL_DYNAMICS_PREINTEGRATION_BASE_H
#define KINDYNVIO_FACTORS_CENTROIDAL_DYNAMICS_PREINTEGRATION_BASE_H

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>

#include <KinDynVIO/Factors/CentroidalDynamicsMeasurementBias.h>
#include <memory>


namespace KinDynVIO
{
namespace Factors
{
/**
 * If you are using the factor, please cite:
 * Fourmy, M., Flayols, T., Mansard, N. and Solà, J., 2021, May.
 * Contact forces pre-integration for the whole body estimation of legged robots.
 * In 2021 IEEE International Conference on Robotics and Automation-ICRA.
 */
struct PreintegrationCentroidalDynamicsParams
{
    // continous-time measurement covariances in local frames
    gtsam::Matrix3 gyroscopeCovariance;
    gtsam::Matrix3 contactForceCovariance, contactTorqueCovariance;
    gtsam::Matrix3 comPositionCovariance; // in base link

    gtsam::Pose3 base_H_imu; ///< pose of the IMU in the base link frame

    PreintegrationCentroidalDynamicsParams() : gyroscopeCovariance(gtsam::I_3x3),
    contactForceCovariance(gtsam::I_3x3),
    contactTorqueCovariance(gtsam::I_3x3),
    comPositionCovariance(gtsam::I_3x3)
    {
    }

    void print(const std::string& s) const;
    bool equals(const PreintegrationCentroidalDynamicsParams& other, double tol=1e-9) const;

    void setGyroscopeCovariance(const gtsam::Matrix3& cov)     { gyroscopeCovariance = cov;     }
    void setContactForceCovariance(const gtsam::Matrix3& cov)  { contactForceCovariance = cov;  }
    void setContactTorqueCovariance(const gtsam::Matrix3& cov) { gyroscopeCovariance = cov;     }
    void setCOMPositionCovariance(const gtsam::Matrix3& cov)   { contactTorqueCovariance = cov; }
    void setBaseHIMU(const gtsam::Pose3& pose)                 { base_H_imu = pose; }

    const gtsam::Matrix3& getGyroscopeCovariance()     const { return gyroscopeCovariance; }
    const gtsam::Matrix3& getContactForceCovariance()  const { return contactForceCovariance;  }
    const gtsam::Matrix3& getContactTorqueCovariance() const { return contactTorqueCovariance; }
    const gtsam::Matrix3& getCOMPositionCovariance()   const { return comPositionCovariance; }
    const gtsam::Pose3&   getBaseHIMU()                const { return base_H_imu; }
};

class PreintegratedCentroidalDynamicsMeasurements
{
public:
    using Bias = gtsam::CentroidalDynamicsMeasurementBias;
    using Params = PreintegrationCentroidalDynamicsParams;
protected:
    std::shared_ptr<Params> p_;
    gtsam::Vector3 gravity_;
    const int residualDim{24};

    // Gyro, net wrench and com position bias used for preintegration
    Bias biasHat_;

    /// Time interval from i to j
    double deltaTij_;

    // Bias update Jacobians
    gtsam::Matrix3 delRdelBiasGyro_;
    gtsam::Matrix3 delCdotdelBiasGyro_, delCdotdelBiasNetForce_;
    gtsam::Matrix3 delCdelBiasGyro_, delCdelBiasNetForce_;
    gtsam::Matrix3 delHadelBiasGyro_, delHadelBiasNetForce_;
    gtsam::Matrix3 delHadelBiasNetTorque_, delHadelBiasCOMPosition_;

    /**
     * Covariance matrix of preintegrated measurements
     * COVARIANCE OF: [PreintRotation PreintCOMVELOCITY PreintCOMPOSITION PreintANGULARMOMENTUM
     *                 BiasGyro BiasNetContactForceInBase BiasNetContactTorqueInBase BiasComPositionInBase]
     * (first-order propagation from *measurementCovariance*).
     * PreintegratedCombinedMeasurements also include the biases and keep the correlation
     * between the preintegrated measurements and the biases
     */
    Eigen::Matrix<double, 24, 24> preintMeasCov_;

    gtsam::Vector12 computeError(const gtsam::Rot3& R_i, const gtsam::Vector3& cdot_i, const gtsam::Vector3& c_i, const gtsam::Vector3& ha_i,
                                 const gtsam::Rot3& R_j, const gtsam::Vector3& cdot_j, const gtsam::Vector3& c_j, const gtsam::Vector3& ha_j,
                                 const gtsam::CentroidalDynamicsMeasurementBias& bias_i, const gtsam::CentroidalDynamicsMeasurementBias& bias_j) const;

public:
    /// Default constructor for serialization
    PreintegratedCentroidalDynamicsMeasurements() : gravity_(gtsam::Vector3(0., 0., -9.80665)) {}

    PreintegratedCentroidalDynamicsMeasurements(const std::shared_ptr<Params>& p,
        const Bias& biasHat = Bias());

    void resetIntegration();
    void resetIntegrationAndSetBias(const Bias& biasHat);

    /// check parameters equality: checks whether shared pointer points to same Params object.
    bool matchesParamsWith(const PreintegratedCentroidalDynamicsMeasurements& other) const
    {
        return p_.get() == other.p_.get();
    }

    const std::shared_ptr<Params>& params() const { return p_; }
    Params& p() const { return *p_; }

    const Bias& biasHat() const { return biasHat_; }
    gtsam::Vector12 biasHatVector() const { return biasHat_.vector(); }
    double deltaTij() const { return deltaTij_; }

    // TODO
    gtsam::Rot3 deltaRij() const {return gtsam::Rot3(); } // Base Rotation delta
    gtsam::Vector3 deltaCdotij() const {return gtsam::Vector3(); } // COM Velocity delta
    gtsam::Vector3 deltaCij() const {return gtsam::Vector3(); } // COM Position delta
    gtsam::Vector3 deltaHaij() const  {return gtsam::Vector3(); } // Angular momentum delta

    gtsam::Vector12 computeErrorAndJacobians(const gtsam::Rot3& R_i, const gtsam::Vector3& cdot_i, const gtsam::Vector3& c_i, const gtsam::Vector3& ha_i,
                                const gtsam::Rot3& R_j, const gtsam::Vector3& cdot_j, const gtsam::Vector3& c_j, const gtsam::Vector3& ha_j,
                                const gtsam::CentroidalDynamicsMeasurementBias& bias_i, const gtsam::CentroidalDynamicsMeasurementBias& bias_j,
                                gtsam::OptionalJacobian<24, 3> H1 = boost::none,
                                gtsam::OptionalJacobian<24, 3> H2 = boost::none,
                                gtsam::OptionalJacobian<24, 3> H3 = boost::none,
                                gtsam::OptionalJacobian<24, 3> H4 = boost::none,
                                gtsam::OptionalJacobian<24, 3> H5 = boost::none,
                                gtsam::OptionalJacobian<24, 3> H6 = boost::none,
                                gtsam::OptionalJacobian<24, 3> H7 = boost::none,
                                gtsam::OptionalJacobian<24, 3> H8 = boost::none,
                                gtsam::OptionalJacobian<24, 12> H9 = boost::none,
                                gtsam::OptionalJacobian<24, 12> H10 = boost::none) const;

    void print(const std::string& s) const;
    bool equals(const PreintegratedCentroidalDynamicsMeasurements& other, double tol=1e-9) const;
};


} // namespace Factors
} // namespace KinDynVIO

#endif  // KINDYNVIO_FACTORS_CENTROIDAL_DYNAMICS_PREINTEGRATION_BASE_H
