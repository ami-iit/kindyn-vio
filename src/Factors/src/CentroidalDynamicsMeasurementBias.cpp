/**
 * @file CentroidalDynamicsMeasurementBias.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynVIO/Factors/CentroidalDynamicsMeasurementBias.h>
#include <gtsam/geometry/Point3.h>
#include <iostream>

using namespace gtsam;

CentroidalDynamicsMeasurementBias::CentroidalDynamicsMeasurementBias()
    : biasGyro(0., 0., 0.)
    , biasNetContactForceInBase(0., 0., 0.)
    , biasNetContactTorqueInBase(0., 0., 0.)
    , biasCOMPositionInBase(0., 0., 0.)
{
}

CentroidalDynamicsMeasurementBias::CentroidalDynamicsMeasurementBias(
    const gtsam::Vector3& bGyro,
    const gtsam::Vector3& bNetForceInBase,
    const gtsam::Vector3& bNetTorqueInBase,
    const gtsam::Vector3& bCOMPositionInBase)
    : biasGyro(bGyro)
    , biasNetContactForceInBase(bNetForceInBase)
    , biasNetContactTorqueInBase(bNetTorqueInBase)
    , biasCOMPositionInBase(bCOMPositionInBase)
{
}

CentroidalDynamicsMeasurementBias::CentroidalDynamicsMeasurementBias(const gtsam::Vector12& v)
    : biasGyro(v.head<3>())
    , biasNetContactForceInBase(v.segment<3>(3))
    , biasNetContactTorqueInBase(v.segment<3>(6))
    , biasCOMPositionInBase(v.tail<3>())
{
}

gtsam::Vector12 CentroidalDynamicsMeasurementBias::vector() const
{
    gtsam::Vector12 v;
    v << biasGyro, biasNetContactForceInBase, biasNetContactTorqueInBase, biasCOMPositionInBase;
    return v;
}

const gtsam::Vector3& CentroidalDynamicsMeasurementBias::gyroscope() const
{
    return biasGyro;
}
const gtsam::Vector3& CentroidalDynamicsMeasurementBias::netContactForceInBase() const
{
    return biasNetContactForceInBase;
}
const gtsam::Vector3& CentroidalDynamicsMeasurementBias::netContactTorqueInBase() const
{
    return biasNetContactTorqueInBase;
}
const gtsam::Vector3& CentroidalDynamicsMeasurementBias::comPositionInBase() const
{
    return biasCOMPositionInBase;
}

gtsam::Vector3
CentroidalDynamicsMeasurementBias::correctGyroscope(const gtsam::Vector3& measurement,
                                                    gtsam::OptionalJacobian<3, 12> H1,
                                                    gtsam::OptionalJacobian<3, 3> H2) const
{
    using namespace gtsam;
    if (H1)
        (*H1) << -I_3x3, Z_3x3, Z_3x3, Z_3x3;
    if (H2)
        (*H2) << I_3x3;
    return measurement - biasGyro;
}

gtsam::Vector3 CentroidalDynamicsMeasurementBias::correctNetContactForceInBase(
    const gtsam::Vector3& measurement,
    gtsam::OptionalJacobian<3, 12> H1,
    gtsam::OptionalJacobian<3, 3> H2) const
{
    using namespace gtsam;
    if (H1)
        (*H1) << Z_3x3, -I_3x3, Z_3x3, Z_3x3;
    if (H2)
        (*H2) << I_3x3;
    return measurement - biasNetContactForceInBase;
}

gtsam::Vector3 CentroidalDynamicsMeasurementBias::correctNetContactTorqueInBase(
    const gtsam::Vector3& measurement,
    gtsam::OptionalJacobian<3, 12> H1,
    gtsam::OptionalJacobian<3, 3> H2) const
{
    using namespace gtsam;
    if (H1)
        (*H1) << Z_3x3, Z_3x3, -I_3x3, Z_3x3;
    if (H2)
        (*H2) << I_3x3;
    return measurement - biasNetContactTorqueInBase;
}

gtsam::Vector3
CentroidalDynamicsMeasurementBias::correctCOMPositionInBase(const gtsam::Vector3& measurement,
                                                            gtsam::OptionalJacobian<3, 12> H1,
                                                            gtsam::OptionalJacobian<3, 3> H2) const
{
    using namespace gtsam;
    if (H1)
        (*H1) << Z_3x3, Z_3x3, Z_3x3, -I_3x3;
    if (H2)
        (*H2) << I_3x3;
    return measurement - biasCOMPositionInBase;
}

void CentroidalDynamicsMeasurementBias::print(const std::string& s) const
{
    std::cout << s << " gyro = " << gtsam::Point3(biasGyro)
              << " net contact force in base = " << gtsam::Point3(biasNetContactForceInBase)
              << " net contact torque in base = " << gtsam::Point3(biasNetContactTorqueInBase)
              << " com position in base = " << gtsam::Point3(biasCOMPositionInBase) << std::endl;
}

bool CentroidalDynamicsMeasurementBias::equals(const CentroidalDynamicsMeasurementBias& expected,
                                               double tol) const
{
    return gtsam::equal_with_abs_tol(biasGyro, expected.biasGyro, tol)
           && gtsam::equal_with_abs_tol(biasNetContactForceInBase,
                                        expected.biasNetContactForceInBase,
                                        tol)
           && gtsam::equal_with_abs_tol(biasNetContactTorqueInBase,
                                        expected.biasNetContactTorqueInBase,
                                        tol)
           && gtsam::equal_with_abs_tol(biasCOMPositionInBase, expected.biasCOMPositionInBase, tol);
}

CentroidalDynamicsMeasurementBias CentroidalDynamicsMeasurementBias::operator-() const
{
    return CentroidalDynamicsMeasurementBias(-biasGyro,
                                             -biasNetContactForceInBase,
                                             -biasNetContactTorqueInBase,
                                             -biasCOMPositionInBase);
}

CentroidalDynamicsMeasurementBias
CentroidalDynamicsMeasurementBias::operator+(const gtsam::Vector12& v) const
{
    return CentroidalDynamicsMeasurementBias(biasGyro + v.head<3>(),
                                             biasNetContactForceInBase + v.segment<3>(3),
                                             biasNetContactTorqueInBase + v.segment<3>(6),
                                             biasCOMPositionInBase + v.tail<3>());
}

CentroidalDynamicsMeasurementBias
CentroidalDynamicsMeasurementBias::operator+(const CentroidalDynamicsMeasurementBias& b) const
{
    return CentroidalDynamicsMeasurementBias(biasGyro + b.biasGyro,
                                             biasNetContactForceInBase
                                                 + b.biasNetContactForceInBase,
                                             biasNetContactTorqueInBase
                                                 + b.biasNetContactTorqueInBase,
                                             biasCOMPositionInBase + b.biasCOMPositionInBase);
}

CentroidalDynamicsMeasurementBias
CentroidalDynamicsMeasurementBias::operator-(const CentroidalDynamicsMeasurementBias& b) const
{
    return CentroidalDynamicsMeasurementBias(biasGyro - b.biasGyro,
                                             biasNetContactForceInBase
                                                 - b.biasNetContactForceInBase,
                                             biasNetContactTorqueInBase
                                                 - b.biasNetContactTorqueInBase,
                                             biasCOMPositionInBase - b.biasCOMPositionInBase);
}
