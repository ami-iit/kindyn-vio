/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <memory>

#include <KinDynVIO/Factors/CentroidalDynamicsMeasurementBias.h>
#include <KinDynVIO/Factors/PreintegratedCentroidalDynamicsMeasurements.h>
#include <KinDynVIO/Factors/CentroidalDynamicsFactor.h>

#include <KinDynVIO/TestData/EstimatorTestUtils.h>

using namespace KinDynVIO::Factors;

TEST_CASE("Cumulative CentroidalDynamics Measurement Bias Test")
{
    Eigen::Matrix<double, 9, 1> v1 = Eigen::Matrix<double, 9, 1>::Random();
    Eigen::Matrix<double, 9, 1> v2 = Eigen::Matrix<double, 9, 1>::Random();
    gtsam::CDMBiasCumulative b1(v1);
    gtsam::CDMBiasCumulative b2(gtsam::Vector3(v2.head<3>()),
                                gtsam::Vector3(v2.segment<3>(3)),
                                gtsam::Vector3(v2.tail<3>()));

    auto bplusb = b1 + b2;
    auto b1minusb2 = b1 - b2;
    auto minusb = -b1;

    REQUIRE(b1.vector().isApprox(v1));
    REQUIRE(b2.vector().isApprox(v2));
    REQUIRE(minusb.vector().isApprox(-v1));
    REQUIRE(bplusb.vector().isApprox(v1 + v2));
    REQUIRE(b1minusb2.vector().isApprox(v1 - v2));
    REQUIRE(b1.equals(gtsam::CDMBiasCumulative(b1)));
    REQUIRE(bplusb.netExternalForceInBase().isApprox(b1.netExternalForceInBase()
                                                    + b2.netExternalForceInBase()));
}

TEST_CASE("Preintegrated Centroidal Dynamics Measurement Cumulative Bias Test")
{
    PreintegratedCDMCumulativeBias pim;
}
