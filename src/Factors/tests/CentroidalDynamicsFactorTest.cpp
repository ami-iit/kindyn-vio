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

#include <KinDynVIO/Factors/CentroidalDynamicsFactor.h>
#include <KinDynVIO/Factors/CentroidalDynamicsMeasurementBias.h>
#include <KinDynVIO/Factors/PreintegratedCentroidalDynamicsMeasurements.h>

using namespace KinDynVIO::Factors;

TEST_CASE("CentroidalDynamics Measurement Bias Test")
{
    Eigen::Matrix<double, 12, 1> v1 = Eigen::Matrix<double, 12, 1>::Random();
    Eigen::Matrix<double, 12, 1> v2 = Eigen::Matrix<double, 12, 1>::Random();
    gtsam::CentroidalDynamicsMeasurementBias b1(v1);
    gtsam::CentroidalDynamicsMeasurementBias b2(gtsam::Vector3(v2.head<3>()),
                                                gtsam::Vector3(v2.segment<3>(3)),
                                                gtsam::Vector3(v2.segment<3>(6)),
                                                gtsam::Vector3(v2.tail<3>()));

    auto bplusb = b1 + b2;
    auto b1minusb2 = b1 - b2;
    auto minusb = -b1;

    REQUIRE(b1.vector().isApprox(v1));
    REQUIRE(b2.vector().isApprox(v2));
    REQUIRE(minusb.vector().isApprox(-v1));
    REQUIRE(bplusb.vector().isApprox(v1 + v2));
    REQUIRE(b1minusb2.vector().isApprox(v1 - v2));
    REQUIRE(b1.equals(gtsam::CentroidalDynamicsMeasurementBias(b1)));
    REQUIRE(bplusb.netContactForceInBase().isApprox(b1.netContactForceInBase()
                                                    + b2.netContactForceInBase()));
}

TEST_CASE("Centroidal Dynamics Factor Test")
{
    CentroidalDynamicsFactor factor;
}
