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

CDMBiasCumulative::CDMBiasCumulative()
    : m_biasGyro(0., 0., 0.)
    , m_biasCOMPositionInBase(0., 0., 0.)
    , m_biasNetExternalForceInBase(0., 0., 0.)
    , m_biasNetExternalTorqueInBase(0., 0., 0.)
{
}

CDMBiasCumulative::CDMBiasCumulative(
    const gtsam::Vector3& bGyro,
    const gtsam::Vector3& bCOMPositionInBase,
    const gtsam::Vector3& bNetForceInBase,
    const gtsam::Vector3& bNetTorqueInBase)
    : m_biasGyro(bGyro)
    , m_biasCOMPositionInBase(bCOMPositionInBase)
    , m_biasNetExternalForceInBase(bNetForceInBase)
    , m_biasNetExternalTorqueInBase(bNetTorqueInBase)
{
}

CDMBiasCumulative::CDMBiasCumulative(const gtsam::Vector12& v)
    : m_biasGyro(v.head<3>())
    , m_biasCOMPositionInBase(v.segment<3>(3))
    , m_biasNetExternalForceInBase(v.segment<3>(6))
    , m_biasNetExternalTorqueInBase(v.tail<3>())
{
}

gtsam::Vector12 CDMBiasCumulative::vector() const
{
    gtsam::Vector12 v;
    v << m_biasGyro, m_biasCOMPositionInBase, m_biasNetExternalForceInBase, m_biasNetExternalTorqueInBase;
    return v;
}

const gtsam::Vector3& CDMBiasCumulative::gyroscope() const
{
    return m_biasGyro;
}
const gtsam::Vector3& CDMBiasCumulative::comPositionInBase() const
{
    return m_biasCOMPositionInBase;
}
const gtsam::Vector3& CDMBiasCumulative::netExternalForceInBase() const
{
    return m_biasNetExternalForceInBase;
}
const gtsam::Vector3& CDMBiasCumulative::netExternalTorqueInBase() const
{
    return m_biasNetExternalTorqueInBase;
}

gtsam::Vector3
CDMBiasCumulative::correctGyroscope(const gtsam::Vector3& measurement,
                                    gtsam::OptionalJacobian<3, 12> H1,
                                    gtsam::OptionalJacobian<3, 3> H2) const
{
    using namespace gtsam;
    if (H1)
        (*H1) << -I_3x3, Z_3x3, Z_3x3, Z_3x3;
    if (H2)
        (*H2) << I_3x3;
    return measurement - m_biasGyro;
}

gtsam::Vector3
CDMBiasCumulative::correctCOMPositionInBase(const gtsam::Vector3& measurement,
                                            gtsam::OptionalJacobian<3, 12> H1,
                                            gtsam::OptionalJacobian<3, 3> H2) const
{
    using namespace gtsam;
    if (H1)
        (*H1) << Z_3x3, -I_3x3, Z_3x3, Z_3x3;
    if (H2)
        (*H2) << I_3x3;
    return measurement - m_biasCOMPositionInBase;
}

gtsam::Vector3 CDMBiasCumulative::correctNetExternalForceInBase(
    const gtsam::Vector3& measurement,
    gtsam::OptionalJacobian<3, 12> H1,
    gtsam::OptionalJacobian<3, 3> H2) const
{
    using namespace gtsam;
    if (H1)
        (*H1) << Z_3x3, Z_3x3, -I_3x3, Z_3x3;
    if (H2)
        (*H2) << I_3x3;
    return measurement - m_biasNetExternalForceInBase;
}

gtsam::Vector3 CDMBiasCumulative::correctNetExternalTorqueInBase(
    const gtsam::Vector3& measurement,
    gtsam::OptionalJacobian<3, 12> H1,
    gtsam::OptionalJacobian<3, 3> H2) const
{
    using namespace gtsam;
    if (H1)
        (*H1) << Z_3x3, Z_3x3, Z_3x3, -I_3x3;
    if (H2)
        (*H2) << I_3x3;
    return measurement - m_biasNetExternalTorqueInBase;
}


void CDMBiasCumulative::print(const std::string& s) const
{
    std::cout << s << " gyro = " << gtsam::Point3(m_biasGyro)
                   << " com position in base = " << gtsam::Point3(m_biasCOMPositionInBase)
                   << " net contact force in base = " << gtsam::Point3(m_biasNetExternalForceInBase)
                   << " net contact torque in base = " << gtsam::Point3(m_biasNetExternalTorqueInBase)
              << std::endl;
}

bool CDMBiasCumulative::equals(const CDMBiasCumulative& other,
                               double tol) const
{
    return gtsam::equal_with_abs_tol(m_biasGyro, other.m_biasGyro, tol)
        && gtsam::equal_with_abs_tol(m_biasCOMPositionInBase, other.m_biasCOMPositionInBase, tol)
        && gtsam::equal_with_abs_tol(m_biasNetExternalForceInBase,
                                     other.m_biasNetExternalForceInBase,
                                     tol)
        && gtsam::equal_with_abs_tol(m_biasNetExternalTorqueInBase,
                                     other.m_biasNetExternalTorqueInBase,
                                     tol);
}

CDMBiasCumulative CDMBiasCumulative::operator-() const
{
    return CDMBiasCumulative(-m_biasGyro,
                             -m_biasCOMPositionInBase,
                             -m_biasNetExternalForceInBase,
                             -m_biasNetExternalTorqueInBase);
}

CDMBiasCumulative
CDMBiasCumulative::operator+(const gtsam::Vector12& v) const
{
    return CDMBiasCumulative(m_biasGyro + v.head<3>(),
                             m_biasCOMPositionInBase + v.segment<3>(3),
                             m_biasNetExternalForceInBase + v.segment<3>(6),
                             m_biasNetExternalTorqueInBase + v.tail<3>());
}

CDMBiasCumulative
CDMBiasCumulative::operator+(const CDMBiasCumulative& other) const
{
    return CDMBiasCumulative(m_biasGyro + other.m_biasGyro,
                             m_biasCOMPositionInBase + other.m_biasCOMPositionInBase,
                             m_biasNetExternalForceInBase + other.m_biasNetExternalForceInBase,
                             m_biasNetExternalTorqueInBase + other.m_biasNetExternalTorqueInBase);
}

CDMBiasCumulative
CDMBiasCumulative::operator-(const CDMBiasCumulative& other) const
{
    return CDMBiasCumulative(m_biasGyro - other.m_biasGyro,
                             m_biasCOMPositionInBase - other.m_biasCOMPositionInBase,
                             m_biasNetExternalForceInBase - other.m_biasNetExternalForceInBase,
                             m_biasNetExternalTorqueInBase - other.m_biasNetExternalTorqueInBase);
}
