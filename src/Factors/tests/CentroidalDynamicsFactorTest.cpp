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
#include <KinDynVIO/Estimators/CentroidalDynamicsPreintegrator.h>
#include <KinDynVIO/Estimators/GraphManager.h>
#include <KinDynVIO/Estimators/IMUPreintegrator.h>
#include "ResourceFolderPath.h"
#include <KinDynVIO/TestData/EstimatorTestUtils.h>

#include <BipedalLocomotion/System/Clock.h>

#include <iDynTree/KinDynComputations.h>

using namespace KinDynVIO::Factors;
using namespace KinDynVIO::Estimators;

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
    bool debug{true};
    bool enableLogging{true};
    Loggable log;
    RobotMatData rmd;
    REQUIRE(readRobotMatFile(getWalkingArucoDataset(), rmd));

    auto kinDyn = getKinDyn(getiCubGenova04ModelPath(), rmd.jointsList);
    REQUIRE(kinDyn->isValid());

    ProprioceptiveInput cdmIn;
    CentroidalDynamicsCumulativeBiasPreintegrator cdmPreInt;
    REQUIRE(cdmPreInt.initialize(getCentroidalDynPreintConfig(),
                                 kinDyn->getRobotModel()));
    // before starting, preintegration status should be idle
    auto cdmOut = cdmPreInt.getOutput();
    REQUIRE(cdmOut.status == PreintegratorStatus::IDLE);

    ForsterIMUPreintegrator imuPreint;
    IMUPreintegratorInput imuIn;
    REQUIRE(imuPreint.setBaseLinkIMUExtrinsics(getBaseHIMU()));
    REQUIRE(imuPreint.initialize(getIMUPreintConfig()));
    // before starting, preintegration status should be idle
    auto imuOut = imuPreint.getOutput();
    REQUIRE(imuOut.status == PreintegratorStatus::IDLE);

    gtsam::Pose3 groundTruthPose, estimatedPose, smootherPose;
    gtsam::Vector3 smootherVel;
    gtsam::Vector3 smootherCOM, smootherCOMVel;
    gtsam::Vector3 smootherAngMomentum;
    gtsam::CDMBiasCumulative smootherCDMBias;
    IMUBias smootherIMUBias;

    double timeNow{0.0}, prevTime{0.0};
    double dt{0.01};
    Eigen::Vector3d g = getGravity();
    Eigen::Matrix<double, 6, 1> baseVel = Eigen::Matrix<double, 6, 1>::Identity();

    Eigen::Vector3d c_i, cdot_i, c_j, cdot_j, ha_i, ha_j;
    Eigen::Matrix3d R_i;
    Eigen::Vector3d p_i, v_i, R_j, p_j, v_j;

    auto w_p_b = iDynTree::Position(rmd.sbasePos.row(0)(0),
                                    rmd.sbasePos.row(0)(1),
                                    rmd.sbasePos.row(0)(2));
    auto w_R_b = iDynTree::Rotation::RPY(rmd.sbaseRot.row(0)(0),
                                         rmd.sbaseRot.row(0)(1),
                                         rmd.sbaseRot.row(0)(2));
    auto w_H_b = iDynTree::Transform(w_R_b, w_p_b);
    groundTruthPose = gtsam::Pose3(iDynTree::toEigen(w_H_b.asHomogeneousTransform()));
    baseVel = rmd.sbaseVel.row(0);
    smootherVel = baseVel.head<3>();
    smootherPose = groundTruthPose;

    kinDyn->setRobotState(iDynTree::toEigen(w_H_b.asHomogeneousTransform()),
                          rmd.jpos.row(0),
                          baseVel,
                          rmd.jvel.row(0),
                          g);

    Eigen::Vector3d com0 = iDynTree::toEigen(kinDyn->getCenterOfMassPosition());
    auto dcom0 = iDynTree::toEigen(kinDyn->getCenterOfMassVelocity());
    auto ha0 = iDynTree::toEigen(kinDyn->getCentroidalTotalMomentum().getAngularVec3());

    smootherCOM = com0;
    smootherCOMVel = dcom0;
    smootherAngMomentum = ha0;

    GraphManager gMgr;
    BaseStateDev priorDev;
    priorDev.imuPosition << 1e-6, 1e-6, 1e-6;
    priorDev.imuOrientation << deg2rad(0.1), deg2rad(0.1), deg2rad(0.1);
    priorDev.imuLinearVelocity << 1e-5, 1e-5, 1e-5;
    priorDev.accelerometerBias << 1e-5, 1e-5, 1e-5;
    priorDev.gyroscopeBias << 1e-5, 1e-5, 1e-5;
    gMgr.addBaseStatePriorAtCurrentKey(timeNow, groundTruthPose, gtsam::Vector3(), priorDev);

    CentroidalStateStdDev priorCentroidalDev;
    priorCentroidalDev.com << 1e-2, 1e-2, 1e-2;
    priorCentroidalDev.dcom << 1e-3, 1e-3, 1e-3;
    priorCentroidalDev.ha << 1e-2, 1e-2, 1e-2;
    priorCentroidalDev.forceBias << 1e1, 1e1, 1e1;
    priorCentroidalDev.torqueBias << 1e-1, 1e-1, 1e-1;
    priorCentroidalDev.comPositionBias << 1e-2, 1e-2, 1e-2;
//     gMgr.addCentroidalStatePriorAtCurrentKey(timeNow, com0, dcom0, ha0, priorCentroidalDev);

    int spawnStateCounter{0};
//     for (std::size_t iter = 1; iter < rmd.time.size(); iter++)
    for (std::size_t iter = 1; iter < 1500; iter++)
    {
        timeNow = rmd.time[iter] - rmd.time[0];
        dt = timeNow - prevTime;

        std::cout << "============Time Now: " << timeNow << ", Iter: " << iter
                  << " ============ dt: " << dt << std::endl;

        // get ground truth pose
        auto w_p_b = iDynTree::Position(rmd.sbasePos.row(iter)(0),
                                        rmd.sbasePos.row(iter)(1),
                                        rmd.sbasePos.row(iter)(2));
        auto w_R_b = iDynTree::Rotation::RPY(rmd.sbaseRot.row(iter)(0),
                                             rmd.sbaseRot.row(iter)(1),
                                             rmd.sbaseRot.row(iter)(2));
        auto w_H_b = iDynTree::Transform(w_R_b, w_p_b);
        groundTruthPose = gtsam::Pose3(iDynTree::toEigen(w_H_b.asHomogeneousTransform()));
        baseVel = rmd.sbaseVel.row(iter);

        kinDyn->setRobotState(iDynTree::toEigen(w_H_b.asHomogeneousTransform()),
                              rmd.jpos.row(iter),
                              baseVel,
                              rmd.jvel.row(iter),
                              g);

        auto comDyn = kinDyn->getCenterOfMassPosition();
        auto dcomDyn = kinDyn->getCenterOfMassVelocity();
        auto haDyn = kinDyn->getCentroidalTotalMomentum().getAngularVec3();

        if (cdmOut.status == PreintegratorStatus::IDLE)
        {
            c_i = iDynTree::toEigen(comDyn);
            cdot_i = iDynTree::toEigen(dcomDyn);
            ha_i = iDynTree::toEigen(haDyn);

            R_i = iDynTree::toEigen(w_R_b);
            cdmPreInt.startPreintegration(prevTime);

        }

        auto start = BipedalLocomotion::clock().now();
        cdmIn = getProprioceptiveInputFromRobotData(iter, rmd);
        cdmPreInt.setInput(cdmIn);
        REQUIRE(cdmPreInt.advance());
        auto end = BipedalLocomotion::clock().now();
//         BipedalLocomotion::log()->info("Time taken to advance "
//                                        "centroidal dynamics preintegration measurement: {}",
//                                        (end-start).count());


        if (imuOut.status == PreintegratorStatus::IDLE)
        {
            p_i = iDynTree::toEigen(w_p_b);
            v_i = baseVel.head<3>();
            imuPreint.startPreintegration(prevTime);
        }

        imuIn = getIMUPreintegratorInputFromRobotData(iter, rmd);
        imuPreint.setInput(imuIn);
        imuPreint.advance();

        if (iter % 50 == 0 ||
            iter == rmd.time.size() - 1)
        {
            cdmPreInt.stopPreintegration();
            cdmOut = cdmPreInt.getOutput();

            imuPreint.stopPreintegration();
            imuOut = imuPreint.getOutput();

            double dTij = cdmOut.preInt.deltaTij();
            double dTijSq = dTij*dTij;

            double dTijIMU = imuOut.preInt.deltaTij();
            double dTijIMUSq = dTijIMU*dTijIMU;

            c_j = iDynTree::toEigen(comDyn);
            cdot_j = iDynTree::toEigen(dcomDyn);
            ha_j = iDynTree::toEigen(haDyn);

            p_j = iDynTree::toEigen(w_p_b);
            v_j = baseVel.head<3>();

            Eigen::Vector3d Dcij_meas = R_i.transpose()*(c_j - c_i - (cdot_i*dTij) - (0.5*g*dTijSq));
            Eigen::Vector3d Dcij_preint = cdmOut.preInt.deltaCij();

            Eigen::Vector3d Dcdotij_meas = R_i.transpose()*(cdot_j - cdot_i - (g*dTij));
            Eigen::Vector3d Dcdotij_preint = cdmOut.preInt.deltaCdotij();

            Eigen::Vector3d DHaij_meas = R_i.transpose()*(ha_j - ha_i);
            Eigen::Vector3d DHaij_preint = cdmOut.preInt.deltaHaij();

            Eigen::Vector3d Dpij_meas = R_i.transpose()*(p_j - p_i - (v_i*dTijIMU) - (0.5*g*dTijIMUSq));
            Eigen::Vector3d Dpij_preint = imuOut.preInt.deltaPij();

            Eigen::Vector3d Dvij_meas = R_i.transpose()*(v_j - v_i - (g*dTijIMU));
            Eigen::Vector3d Dvij_preint = imuOut.preInt.deltaVij();

            std::cout << "DTij (CDM): \n" << dTij << std::endl;
            std::cout << "Dcij (KinDyn): \n" << Dcij_meas << std::endl;
            std::cout << "Dcij (preintegrated): \n" << Dcij_preint << std::endl;
            std::cout << "Dcdotij (KinDyn): \n" << Dcdotij_meas << std::endl;
            std::cout << "Dcdotij (preintegrated): \n" << Dcdotij_preint << std::endl;
            std::cout << "DHaij (KinDyn): \n" << DHaij_meas << std::endl;
            std::cout << "DHaij (preintegrated): \n" << DHaij_preint << std::endl;

            std::cout << "DTij (IMU): \n" << dTijIMU << std::endl;
            std::cout << "Dpij (Sim): \n" << Dpij_meas << std::endl;
            std::cout << "Dpij (preintegrated): \n" << Dpij_preint << std::endl;
            std::cout << "Dvij (Sim): \n" << Dvij_meas << std::endl;
            std::cout << "Dvij (preintegrated): \n" << Dvij_preint << std::endl;

            if (cdmOut.status == PreintegratorStatus::PREINTEGRATED &&
                imuOut.status == PreintegratorStatus::PREINTEGRATED)
            {
                gMgr.spawnNewState(timeNow);
                spawnStateCounter++;
                BipedalLocomotion::log()->info("GMgr: Spawned new state {} at {} seconds", spawnStateCounter, timeNow);
                gMgr.setInitialGuessForCurrentBaseStates(smootherPose,
                                                         smootherVel,
                                                         smootherIMUBias);
//                 gMgr.setInitialGuessForCurrentCentroidalStates(smootherCOM,
//                                                                smootherCOMVel,
//                                                                smootherAngMomentum,
//                                                                smootherCDMBias);
//                 gMgr.setInitialGuessForCurrentCentroidalStates(c_i,
//                                                                cdot_i,
//                                                                ha_i);

                gMgr.processAbsolutePoseMeasurement(groundTruthPose, 1e-3, 1e-3);
//                 gMgr.processPreintegratedCDM(cdmOut.preInt);
                gMgr.processPreintegratedIMUMeasurements(imuOut.preInt);

                auto start = BipedalLocomotion::clock().now();
                REQUIRE(gMgr.optimize());
                auto end = BipedalLocomotion::clock().now();
                BipedalLocomotion::log()->info("Time taken to optimize: {}", (end-start).count());

                BipedalLocomotion::log()->warn("Preintegration complete.");
                cdmPreInt.resetIntegration();
                imuPreint.resetIMUIntegration();

                smootherPose = gMgr.getEstimatedBasePose();
                smootherVel = gMgr.getEstimatedBaseLinearVelocity();
                smootherIMUBias = gMgr.getEstimatedIMUBias();
//                 smootherCOM = gMgr.getEstimatedCOMPosition();
//                 smootherCOMVel = gMgr.getEstimatedCOMVelocity();
//                 smootherCDMBias = gMgr.getEstimatedCDMBias();
//                 smootherAngMomentum = gMgr.getEstimatedCentroidalAngularMomentum();

                updateLogData(timeNow, log.smootherTime);
                updateLogData(smootherPose.translation(), log.smootherPosition);
                updateLogData(smootherPose.rotation().ypr(), log.smootherRotation);
                updateLogData(smootherVel, log.smootherLinearVelocity);
                updateLogData(smootherCOM, log.smootherCOM);
                updateLogData(smootherCOMVel, log.smootherCOMVel);
                updateLogData(smootherAngMomentum, log.smootherAngMomentum);
                updateLogData(smootherCDMBias.vector(), log.smootherCDMBias);
            }

        }

        // update log
        updateLogData(timeNow, log.simTime);
        updateLogData(groundTruthPose.translation(), log.simPosition);
        updateLogData(groundTruthPose.rotation().ypr(), log.simRotation);
        updateLogData(baseVel, log.simVelocity);

        updateLogData(iDynTree::toEigen(comDyn), log.simCOM);
        updateLogData(iDynTree::toEigen(dcomDyn), log.simCOMVel);
        updateLogData(iDynTree::toEigen(haDyn), log.simAngMomentum);

        prevTime = timeNow;
        imuOut = imuPreint.getOutput();
        cdmOut = cdmPreInt.getOutput();
    }

    if (enableLogging)
    {
        bool writeOk = writeLoggableToFile(log, "out.mat");
        REQUIRE(writeOk);
    }
}
