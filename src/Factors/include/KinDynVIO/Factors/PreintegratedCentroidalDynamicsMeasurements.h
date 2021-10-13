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
#include <iDynTree/KinDynComputations.h>
#include <map>
#include <memory>

namespace KinDynVIO
{
namespace Factors
{

// use ordered map in order to handle
// the dynamic application of
// different wrenches properly
// across multiple quantities
using LocalContactWrenchesMap = std::map<iDynTree::FrameIndex, gtsam::Vector6>;

struct PreintegrationCentroidalDynamicsParams
{
    // continous-time measurement covariances in local frames
    gtsam::Matrix3 gyroscopeCovariance;
    gtsam::Matrix3 contactForceCovariance, contactTorqueCovariance;
    gtsam::Matrix3 comPositionCovariance; // in base link

    // biases
    gtsam::Matrix3 gyroscopeBiasCovariance;
    gtsam::Matrix3 contactForceBiasCovariance, contactTorqueBiasCovariance;
    gtsam::Matrix3 comPositionBiasCovariance; // in base link

    gtsam::Pose3 baseHImu; ///< pose of the IMU in the base link frame
    std::string baseLinkName;
    std::string baseImuName;

    PreintegrationCentroidalDynamicsParams() : gyroscopeCovariance(gtsam::I_3x3),
    contactForceCovariance(gtsam::I_3x3),
    contactTorqueCovariance(gtsam::I_3x3),
    comPositionCovariance(gtsam::I_3x3),
    gyroscopeBiasCovariance(gtsam::I_3x3),
    contactForceBiasCovariance(gtsam::I_3x3),
    contactTorqueBiasCovariance(gtsam::I_3x3),
    comPositionBiasCovariance(gtsam::I_3x3)
    {
    }

    void print(const std::string& s) const;
    bool equals(const PreintegrationCentroidalDynamicsParams& other, double tol=1e-9) const;

    void setGyroscopeCovariance(const gtsam::Matrix3& cov)     { gyroscopeCovariance = cov;      }
    void setContactForceCovariance(const gtsam::Matrix3& cov)  { contactForceCovariance = cov;   }
    void setContactTorqueCovariance(const gtsam::Matrix3& cov) { contactTorqueCovariance = cov;  }
    void setCOMPositionCovariance(const gtsam::Matrix3& cov)   { comPositionCovariance = cov;    }
    void setGyroscopeBiasCovariance(const gtsam::Matrix3& cov)     { gyroscopeBiasCovariance = cov;     }
    void setContactForceBiasCovariance(const gtsam::Matrix3& cov)  { contactForceBiasCovariance = cov;  }
    void setContactTorqueBiasCovariance(const gtsam::Matrix3& cov) { contactTorqueBiasCovariance = cov; }
    void setCOMPositionBiasCovariance(const gtsam::Matrix3& cov)   { comPositionBiasCovariance = cov;   }
    void setBaseHIMU(const gtsam::Pose3& pose)               { baseHImu = pose;     }
    void setBaseLinkName(const std::string& name)            { baseLinkName = name; }
    void setBaseImuName(const std::string& name)             { baseImuName = name;  }

    const gtsam::Matrix3& getGyroscopeCovariance()       const { return gyroscopeCovariance;     }
    const gtsam::Matrix3& getContactForceCovariance()    const { return contactForceCovariance;  }
    const gtsam::Matrix3& getContactTorqueCovariance()   const { return contactTorqueCovariance; }
    const gtsam::Matrix3& getCOMPositionCovariance()     const { return comPositionCovariance;   }
    const gtsam::Matrix3& getGyroscopeBiasCovariance()       const { return gyroscopeBiasCovariance;     }
    const gtsam::Matrix3& getContactForceBiasCovariance()    const { return contactForceBiasCovariance;  }
    const gtsam::Matrix3& getContactTorqueBiasCovariance()   const { return contactTorqueBiasCovariance; }
    const gtsam::Matrix3& getCOMPositionBiasCovariance()     const { return comPositionBiasCovariance;   }
    const gtsam::Pose3&   getBaseHIMU()                  const { return baseHImu;                }
    const std::string& getBaseLinkName()                 const { return baseLinkName;            }
    const std::string& getBaseImuName()                  const { return baseImuName;             }
};

/**
 * This is an improved version of the contact force preintegration considered
 * in below referenced paper.
 * The difference from the original paper is that, in this implementation
 * we consider additional bias terms on the cumulative external wrenches acting on the base link
 * If you are using the factor, please cite:
 * Fourmy, M., Flayols, T., Mansard, N. and Solà, J., 2021, May.
 * Contact forces pre-integration for the whole body estimation of legged robots.
 * In 2021 IEEE International Conference on Robotics and Automation-ICRA.
 */
template <typename BiasType>
class PreintegratedCentroidalDynamicsMeasurements
{
public:
    using Bias = BiasType;
    using Params = PreintegrationCentroidalDynamicsParams;
protected:
    std::shared_ptr<Params> m_p;
    gtsam::Vector3 m_gravity;

    // Gyro, net wrench and com position bias used for preintegration
    Bias m_biasHat;

    gtsam::Matrix m_preintMeasCov;

    /// Time interval from i to j
    double m_deltaTij;
    gtsam::Rot3 m_deltaRij;
    gtsam::Vector3 m_deltaCdotij, m_deltaCij, m_deltaHaij;

    gtsam::Vector3 correctGyroMeasurementBySensorPose(const gtsam::Vector3& unbiasedGyro)
    {
        return m_p->getBaseHIMU().rotation().matrix()*unbiasedGyro;
    }

    // Delta(b_hat) = Delta(b_bar) + J^Delta_b deltab
    virtual void biasCorrectedDeltas(const BiasType& bias) = 0;

public:
    /**
     * Assumes that the kinDyn state is already set with the
     * encoder measurements from the current time step
     * and that the local contact wrenches map consists of contact wrenches
     * measures at the links whose LinkIndices are available in
     * the kinDyn loaded robot model.
     * KinDynComputations will be used to obtain COM position measurement in base,
     * relative kinematics between base and the links in contact
     *
     * @warning not thread-safe
     */
    virtual bool update(const std::shared_ptr<iDynTree::KinDynComputations>& kinDyn,
                        const gtsam::Vector3& localGyroMeas,
                        const LocalContactWrenchesMap& localContactWrenches,
                        const double& dt) = 0;
    virtual void resetIntegration() = 0;
    virtual void resetIntegrationAndSetBias(const Bias& biasHat) = 0;

    /// check parameters equality: checks whether shared pointer points to same Params object.
    bool matchesParamsWith(const PreintegratedCentroidalDynamicsMeasurements<BiasType>& other) const
    {
        return m_p.get() == other.m_p.get();
    }

    const std::shared_ptr<Params>& params() const { return m_p;  }
    Params& p()                             const { return *m_p; }

    const Bias& biasHat()          const { return m_biasHat;          }
    gtsam::Vector biasHatVector()  const { return m_biasHat.vector(); }
    double deltaTij()              const { return m_deltaTij;         }

    gtsam::Rot3    deltaRij()      const { return m_deltaRij;      } // Base Rotation delta
    gtsam::Vector3 deltaCdotij()   const { return m_deltaCdotij;   } // COM Velocity delta
    gtsam::Vector3 deltaCij()      const { return m_deltaCij;      } // COM Position delta
    gtsam::Vector3 deltaHaij()     const { return m_deltaHaij;     } // Angular momentum delta
    gtsam::Matrix  preintMeasCov() const { return m_preintMeasCov; }

    virtual gtsam::Vector computeErrorAndJacobians(const gtsam::Pose3& H_i,
                                                   const gtsam::Vector3& cdot_i,
                                                   const gtsam::Vector3& c_i,
                                                   const gtsam::Vector3& ha_i,
                                                   const gtsam::Pose3& H_j,
                                                   const gtsam::Vector3& cdot_j,
                                                   const gtsam::Vector3& c_j,
                                                   const gtsam::Vector3& ha_j,
                                                   const Bias& bias_i,
                                                   const Bias& bias_j,
                                                   gtsam::OptionalJacobian<24, 6> H1 = boost::none,
                                                   gtsam::OptionalJacobian<24, 3> H2 = boost::none,
                                                   gtsam::OptionalJacobian<24, 3> H3 = boost::none,
                                                   gtsam::OptionalJacobian<24, 3> H4 = boost::none,
                                                   gtsam::OptionalJacobian<24, 6> H5 = boost::none,
                                                   gtsam::OptionalJacobian<24, 3> H6 = boost::none,
                                                   gtsam::OptionalJacobian<24, 3> H7 = boost::none,
                                                   gtsam::OptionalJacobian<24, 3> H8 = boost::none,
                                                   gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H9 = boost::none,
                                                   gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H10 = boost::none) = 0;
};

struct CDMModelComputations
{
    double mass{0.0};
    std::size_t nrExtWrenches{0};
    gtsam::Vector3 netExtForceInBase;
    gtsam::Vector3 netExtTorqueInBase;
    gtsam::Vector3 comInBase;

    gtsam::Vector3 netUnbiasedExtForceInBase;
    gtsam::Vector3 netUnbiasedExtTorqueInBase;
    gtsam::Vector3 unBiasedCoMInBase;

    gtsam::Matrix3 SBfnet, SBtaunet; // skew-symmetric matrices

    std::map<iDynTree::LinkIndex, gtsam::Vector3> contactFrameLeverArmsInBase;
    std::map<iDynTree::LinkIndex, gtsam::Matrix3> contactFrameRotationsInBase;
};

struct CDMGyroComputations
{
    gtsam::Vector3 unbiasedGyro;
    gtsam::Rot3 DRkkplusone, DRik;
    gtsam::Matrix3 Jr;
};

class PreintegratedCDMCumulativeBias :
        public PreintegratedCentroidalDynamicsMeasurements<gtsam::CDMBiasCumulative>
{
protected:
    const std::size_t m_residualDim{24};
    // Bias update Jacobians
    gtsam::Matrix3 m_delRdelBiasGyro;
    gtsam::Matrix3 m_delCdotdelBiasGyro, m_delCdotdelBiasNetForce;
    gtsam::Matrix3 m_delCdelBiasGyro, m_delCdelBiasNetForce;
    gtsam::Matrix3 m_delHadelBiasGyro, m_delHadelBiasNetTorque, m_delHadelBiasCOMPosition;
    gtsam::Matrix m_G, m_A, m_B, m_Sigma_n;
    CDMModelComputations m_currModelComp;
    CDMGyroComputations m_currGyroComp;

    /**
     * Covariance matrix of preintegrated measurements
     * COVARIANCE OF: [PreintRotation PreintCOMVELOCITY PreintCOMPOSITION PreintANGULARMOMENTUM
     *                 BiasGyro BiasComPositionInBase BiasNetContactForceInBase BiasNetContactTorqueInBase]
     * (first-order propagation from *measurementCovariance*).
     * PreintegratedCombinedMeasurements also include the biases and keep the correlation
     * between the preintegrated measurements and the biases
     *
     * Sigma_ii = 0
     * Sigma_ik+1 = A Sigma_ik A.T + G
     * where, G = B Sigma_n B.T
     */
    void propagatePreintegrationCovariance(const CDMModelComputations& modelComp,
                                           const CDMGyroComputations& gyroComp,
                                           const double& dt);
    void updatePreintegratedMeasurements(const CDMModelComputations& modelComp,
                                         const CDMGyroComputations& gyroComp,
                                         const double& dt);
    bool updateCDMModelComputations(const std::shared_ptr<iDynTree::KinDynComputations>& kinDyn,
                                    const LocalContactWrenchesMap& localContactWrenches);
    void updateCDMGyroComputations(const gtsam::Vector3& localGyroMeas,
                                   const double& dt);
    void updateBiasJacobians(const CDMModelComputations& modelComp,
                             const CDMGyroComputations& gyroComp,
                             const double& dt);
    void prepareA(const CDMModelComputations& modelComp,
                  const CDMGyroComputations& gyroComp,
                  const double& dt);
    void prepareG(const CDMModelComputations& modelComp,
                  const CDMGyroComputations& gyroComp,
                  const double& dt);

    // r_Delta = [r_DeltaRij, r_DeltaCdotij, r_DeltaCij, r_DetlaHaij]
    gtsam::Vector computeError(const gtsam::Rot3& dR,
                               const gtsam::Vector3& dcDot,
                               const gtsam::Vector3& dc,
                               const gtsam::Vector3& dha,
                               const Bias& dBias,
                               const Bias& bias_i);
    void biasCorrectedDeltas(const Bias& bias) override;
public:
    PreintegratedCDMCumulativeBias();
    PreintegratedCDMCumulativeBias(const std::shared_ptr<Params>& p,
                                   const Bias& biasHat = Bias());

    void resetIntegration() override;
    void resetIntegrationAndSetBias(const Bias& biasHat) override;

    bool update(const std::shared_ptr<iDynTree::KinDynComputations>& kinDyn,
                const gtsam::Vector3& localGyroMeas,
                const LocalContactWrenchesMap& localContactWrenches,
                const double& dt) override;

    gtsam::Vector computeErrorAndJacobians(const gtsam::Pose3& H_i,
                                           const gtsam::Vector3& cdot_i,
                                           const gtsam::Vector3& c_i,
                                           const gtsam::Vector3& ha_i,
                                           const gtsam::Pose3& H_j,
                                           const gtsam::Vector3& cdot_j,
                                           const gtsam::Vector3& c_j,
                                           const gtsam::Vector3& ha_j,
                                           const Bias& bias_i,
                                           const Bias& bias_j,
                                           gtsam::OptionalJacobian<24, 6> H1 = boost::none,
                                           gtsam::OptionalJacobian<24, 3> H2 = boost::none,
                                           gtsam::OptionalJacobian<24, 3> H3 = boost::none,
                                           gtsam::OptionalJacobian<24, 3> H4 = boost::none,
                                           gtsam::OptionalJacobian<24, 6> H5 = boost::none,
                                           gtsam::OptionalJacobian<24, 3> H6 = boost::none,
                                           gtsam::OptionalJacobian<24, 3> H7 = boost::none,
                                           gtsam::OptionalJacobian<24, 3> H8 = boost::none,
                                           gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H9 = boost::none,
                                           gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H10 = boost::none) override;

    void print(const std::string& s) const;
    bool equals(const PreintegratedCDMCumulativeBias& other, double tol=1e-9) const;
};

} // namespace Factors
} // namespace KinDynVIO

#endif  // KINDYNVIO_FACTORS_CENTROIDAL_DYNAMICS_PREINTEGRATION_BASE_H
