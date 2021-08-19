/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include <catch2/catch.hpp>
#include <memory>

#include <KinDynVIO/Estimators/IMUPreintegrator.h>
#include <KinDynVIO/Estimators/States.h>

#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <gtsam/geometry/Rot3.h>

using namespace KinDynVIO::Estimators;
using namespace BipedalLocomotion::ParametersHandler;

TEST_CASE("IMUPreintegration Test")
{
    bool debug{false};
    const double dt{0.01};
    Eigen::Vector3d g;
    g << 0, 0, -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    Eigen::Vector3d pDoubleDot, omegaB;
    pDoubleDot << 1.0, 0.0, 0.0;
    omegaB << 0.0, 0.01, 0.0; // in body coordinates

    Eigen::Vector3d p, v, p0, v0;
    gtsam::Rot3 R, R0;
    R.identity();
    p.setZero();
    v.setZero();
    R0 = R;
    p0 = p;
    v0 = v;

    IMUPreintegratorInput input;
    input.linAcc.setZero();
    input.gyro.setZero();
    auto bias = gtsam::imuBias::ConstantBias();

    IMUState Xi(gtsam::Pose3(R, p), v, bias);
    IMUState Xj;

    auto imuPreintegrator = std::make_unique<ForsterIMUPreintegrator>();
    std::shared_ptr<IParametersHandler> parameterHandler = std::make_shared<StdImplementation>();
    parameterHandler->setParameter("sigma_acc", 1e-5);
    parameterHandler->setParameter("sigma_gyro", 1e-5);
    parameterHandler->setParameter("sigma_b_acc", 1e-5);
    parameterHandler->setParameter("sigma_b_gyro", 1e-5);
    parameterHandler->setParameter("sigma_pos_integration", 0.0);
    parameterHandler->setParameter("error_bias", 0.0);
    parameterHandler->setParameter("initial_bias", bias.vector());
    REQUIRE(imuPreintegrator->initialize(parameterHandler));

    double t = dt;
    for (; t < 1.0; t += dt)
    {
        // simulated unbiased, noiseless IMU measurements
        input.gyro = omegaB;
        input.linAcc = R.transpose() * (pDoubleDot - g);
        input.ts = t;

        // relevant variable keys are always associated
        // to ith state (Xi) and jth state (Xj), and IMU measurements (o)
        // are integrated between these states
        //
        // |  o  o  o  o  o  o  o  o  o ... o  o  o |
        // Xi                                       Xj
        // vi                                       vi
        // bi                                       bj
        //
        // constrained into one preintegration factor
        //
        // |-------------------[]-------------------|
        // Xi                                       Xj
        // vi                                       vi
        // bi                                       bj
        //
        // we set random keys now
        // however they are required to be associated
        // to relevant variables add to the factor graph
        // to get the combined IMU factor as output
        input.posei = 1;
        input.vi = 2;
        input.bi = 3;
        input.posei = 4;
        input.vi = 5;
        input.bi = 6;

        imuPreintegrator->setInput(input);
        imuPreintegrator->advance();

        // simple Euler integration
        p += v * dt + (0.5 * pDoubleDot * dt * dt);
        v += pDoubleDot * dt;
        R = R * gtsam::Rot3::Expmap(omegaB * dt);
    }

    // we can make use of the methods related to prediction
    // and output factor once, the entire loop for these
    // intermediate IMU integration has been carried out
    imuPreintegrator->getPredictedState(Xi, Xj);
    auto factor = imuPreintegrator->getOutput();

    if (debug)
    {
        std::cout << "=============================================" << std::endl;
        std::cout << "Pos: " << p.transpose() << " Vel: " << v.transpose() << " RPY: " << R.roll()
                  << " " << R.pitch() << " " << R.yaw() << std::endl;
        std::cout << "IMU acc: " << input.linAcc.transpose() << " gyro: " << input.gyro.transpose()
                  << std::endl;

        std::cout << "Predicted Pos: " << Xj.p().transpose() << " Vel: " << Xj.v().transpose()
                  << " RPY: " << Xj.rpy().transpose() << std::endl;
    }

    REQUIRE(p.isApprox(Xj.p()));
    REQUIRE(v.isApprox(Xj.v()));
    REQUIRE(R.matrix().isApprox(Xj.pose().rotation().matrix()));

    // check IMU deltas
    // See Eqn 26. in the paper
    // "IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori
    // Estimation", Christian Forster, Luca Carlone, Frank Dellaert, Davide Scaramuzza
    double deltaTij = t - dt;
    Eigen::Vector3d deltaRij = gtsam::Rot3::Logmap(R0.inverse() * R);
    Eigen::Vector3d deltaVij = R0.inverse() * (v - v0 - (g * deltaTij));
    Eigen::Vector3d deltaPij
        = R0.inverse() * (p - p0 - (v0 * deltaTij) - (0.5 * g * deltaTij * deltaTij));

    Eigen::Vector3d factorDeltaRij
        = gtsam::Rot3::Logmap(factor.preintegratedMeasurements().deltaRij());
    Eigen::Vector3d factorDeltaPij = factor.preintegratedMeasurements().deltaPij();
    Eigen::Vector3d factorDeltaVij = factor.preintegratedMeasurements().deltaVij();

    if (debug)
    {
        std::cout << "Actual deltas Rij: " << deltaRij.transpose() << std::endl;
        std::cout << "Factor deltas Rij: " << factorDeltaRij.transpose() << std::endl;

        std::cout << "Actual deltas Pij: " << deltaPij.transpose() << std::endl;
        std::cout << "Factor deltas Pij: " << factorDeltaPij.transpose() << std::endl;

        std::cout << "Actual deltas Vij: " << deltaVij.transpose() << std::endl;
        std::cout << "Factor deltas Vij: " << factorDeltaVij.transpose() << std::endl;
    }

    REQUIRE(deltaRij.isApprox(factorDeltaRij));
    REQUIRE(deltaPij.isApprox(factorDeltaPij));
    REQUIRE(deltaVij.isApprox(factorDeltaVij));
}
