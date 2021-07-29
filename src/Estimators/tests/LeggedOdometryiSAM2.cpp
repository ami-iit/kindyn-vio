/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include <catch2/catch.hpp>

#include <ResourceFolderPath.h>

#include <Eigen/Dense>
#include <manif/manif.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/ModelIO/ModelLoader.h>

#include <BipedalLocomotion/ContactDetectors/SchmittTriggerDetector.h>
#include <BipedalLocomotion/Conversions/matioCppConversions.h>
#include <BipedalLocomotion/FloatingBaseEstimators/LeggedOdometry.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

#include <matioCpp/matioCpp.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <ratio>
#include <memory>

using namespace std::chrono_literals;

using namespace BipedalLocomotion::Conversions;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion::Estimators;
using namespace BipedalLocomotion::Contacts;

double deg2rad(const double& ang)
{
    return ang * (M_PI / 180);
}

bool populateSchmittParams(std::weak_ptr<IParametersHandler> handler)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        return false;
    }

    handle->setParameter("contacts", std::vector<std::string>{"l_sole", "r_sole"});
    handle->setParameter("contact_make_thresholds", std::vector<double>{150.0, 150.0});
    handle->setParameter("contact_break_thresholds", std::vector<double>{120.0, 120.0});
    handle->setParameter("contact_make_switch_times", std::vector<double>{0.01, 0.01});
    handle->setParameter("contact_break_switch_times", std::vector<double>{0.01, 0.01});
    return true;
}

bool populateEstimatorConfig(std::weak_ptr<IParametersHandler> handler,
                             const int& nrJoints,
                             const iDynTree::Rotation& R0,
                             const iDynTree::Position& p0,
                             const iDynTree::Rotation& Rlf0,
                             const iDynTree::Position& plf0,
                             const iDynTree::Rotation& Rrf0,
                             const iDynTree::Position& prf0)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        return false;
    }
    handle->setParameter("use_model_info", true);
    handle->setParameter("sampling_period_in_s", 0.01);

    auto modelInfoGroup = std::make_shared<StdImplementation>();
    handle->setGroup("ModelInfo", modelInfoGroup);
    modelInfoGroup->setParameter("base_link", "root_link");
    modelInfoGroup->setParameter("base_link_imu", "root_link_imu_acc");
    modelInfoGroup->setParameter("left_foot_contact_frame", "l_sole");
    modelInfoGroup->setParameter("right_foot_contact_frame", "r_sole");

    auto optionsGroup = std::make_shared<StdImplementation>();
    handle->setGroup("Options", optionsGroup);
    optionsGroup->setParameter("enable_imu_bias_estimation", false);
    optionsGroup->setParameter("enable_ekf_update", true);
    optionsGroup->setParameter("use_kinematics_measure", true);
    optionsGroup->setParameter("use_static_ldmks_pose_measure", false);

    auto loGroup = std::make_shared<StdImplementation>();
    handle->setGroup("LeggedOdom", loGroup);
    loGroup->setParameter("initial_fixed_frame", "l_sole");
    loGroup->setParameter("initial_ref_frame_for_world", "root_link");


    auto quat = R0.asQuaternion();

    loGroup->setParameter("initial_world_orientation_in_ref_frame", std::vector<double>{quat(0), quat(1), quat(2), quat(3)});
    loGroup->setParameter("initial_world_position_in_ref_frame", std::vector<double>{p0(0), p0(1),p0(2)});

    auto switching_pattern_for_lo{"lastActive"};
    loGroup->setParameter("switching_pattern", switching_pattern_for_lo);

    return true;
}

TEST_CASE("Legged Odometry + iSAM2 Fixed Lag Smoothing")
{
    std::string modelFilePath{getiCubURDFModelPath()};
    std::string matFilePath{getTestDatasetPath()};

    std::cout << "Model path: " << modelFilePath << std::endl;
    std::cout << "Matfile path: " << matFilePath << std::endl;

    matioCpp::File input(matFilePath);
    REQUIRE(input.isOpen());

    std::cout << "Opening mat file successful." << std::endl;

    ////////////////////
    // read the mat file
    ////////////////////
    auto time = input.read("estimatorJointsTime").asVector<double>();
    auto jointsPos = input.read("estimatorJointsPos").asMultiDimensionalArray<double>();
    auto jointsVel = input.read("estimatorJointsVel").asMultiDimensionalArray<double>();
    auto simBasePos = input.read("linkBasePos").asMultiDimensionalArray<double>();
    auto simBaseRot = input.read("linkBaseRot").asMultiDimensionalArray<double>();
    auto nrJoints = static_cast<int>(input.read("nr_joints_est").asElement<double>());

    auto lfForceZ = input.read("lfForceZ").asMultiDimensionalArray<double>();
    auto rfForceZ = input.read("rfForceZ").asMultiDimensionalArray<double>();
    auto imuAcc = input.read("imuAcc").asMultiDimensionalArray<double>();
    auto imuOmega = input.read("imuOmega").asMultiDimensionalArray<double>();

    std::vector<std::string> jointsList
        = {"neck_pitch",     "neck_roll",   "neck_yaw",         "torso_pitch",
           "torso_roll",     "torso_yaw",   "l_shoulder_pitch", "l_shoulder_roll",
           "l_shoulder_yaw", "l_elbow",     "r_shoulder_pitch", "r_shoulder_roll",
           "r_shoulder_yaw", "r_elbow",     "l_hip_pitch",      "l_hip_roll",
           "l_hip_yaw",      "l_knee",      "l_ankle_pitch",    "l_ankle_roll",
           "r_hip_pitch",    "r_hip_roll",  "r_hip_yaw",        "r_knee",
           "r_ankle_pitch",  "r_ankle_roll"};

    Eigen::VectorXd timeEig = toEigen(time);
    Eigen::MatrixXd jposEig = toEigen(jointsPos);
    Eigen::MatrixXd jvelEig = toEigen(jointsVel);
    Eigen::MatrixXd sbasePosEig = toEigen(simBasePos);
    Eigen::MatrixXd sbaseRotEig = toEigen(simBaseRot);
    Eigen::MatrixXd lfZEig = toEigen(lfForceZ);
    Eigen::MatrixXd rfZEig = toEigen(rfForceZ);
    Eigen::MatrixXd imuAccEig = toEigen(imuAcc);
    Eigen::MatrixXd imuOmegaEig = toEigen(imuOmega);

    input.close();

    auto dt{0.01};
    auto nrIters{timeEig.rows()};

    /////////////////////////
    // Prepare the model
    /////////////////////////
    // load model
    iDynTree::ModelLoader modelLoader;
    REQUIRE(modelLoader.loadReducedModelFromFile(modelFilePath, jointsList));
    auto model = modelLoader.model();
    std::cout << "Loading model successful." << std::endl;

    /////////////////////////
    // Setup the estimator
    /////////////////////////
    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    kinDyn->loadRobotModel(model);

    // get initial ground truth pose to initialize estimator
    auto w_p_b0
        = iDynTree::Position(sbasePosEig.row(0)(0), sbasePosEig.row(0)(1), sbasePosEig.row(0)(2));
    auto w_R_b0 = iDynTree::Rotation::RPY(sbaseRotEig.row(0)(0),
                                          sbaseRotEig.row(0)(1),
                                          sbaseRotEig.row(0)(2));
    auto w_H_b0 = iDynTree::Transform(w_R_b0, w_p_b0);
    Eigen::Matrix<double, 6, 1> zeroTwist;
    zeroTwist.setZero();
    Eigen::Vector3d g;
    g << 0, 0, -9.80665;

    kinDyn->setRobotState(iDynTree::toEigen(w_H_b0.asHomogeneousTransform()),
                          iDynTree::make_span(jposEig.row(0).data(), jposEig.row(0).size()),
                          zeroTwist,
                          iDynTree::make_span(jvelEig.row(0).data(), jvelEig.row(0).size()),
                          g);

    auto w_H_imu0 = kinDyn->getWorldTransform("root_link_imu_acc");
    auto w_H_lsole0 = kinDyn->getWorldTransform("l_sole");
    auto w_H_rsole0 = kinDyn->getWorldTransform("r_sole");

    // configure estimator internal parameters for configuration setup
    std::shared_ptr<StdImplementation> originalHandler = std::make_shared<StdImplementation>();
    IParametersHandler::shared_ptr parameterHandler = originalHandler;

    populateEstimatorConfig(parameterHandler,
                            nrJoints,
                            w_H_imu0.getRotation(),
                            w_H_imu0.getPosition(),
                            w_H_lsole0.getRotation(),
                            w_H_lsole0.getPosition(),
                            w_H_rsole0.getRotation(),
                            w_H_rsole0.getPosition());
    std::cout << "r_sole position in the world: " << w_H_rsole0.getPosition()(2) << std::endl;
    std::cout << "l_sole position in the world: " << w_H_lsole0.getPosition()(2) << std::endl;
    // create and initialize estimator
    auto filter = std::make_unique<LeggedOdometry>();
    REQUIRE(filter->initialize(parameterHandler, kinDyn));
    std::cout << "Configured estimator successful" << std::endl;

    /////////////////////////
    // Setup the contact detector
    /////////////////////////
    std::shared_ptr<StdImplementation> contactHandler = std::make_shared<StdImplementation>();

    IParametersHandler::shared_ptr schmittParameterHandler = contactHandler;
    populateSchmittParams(schmittParameterHandler);
    SchmittTriggerDetector contactDetector;

    REQUIRE(contactDetector.initialize(schmittParameterHandler));
    std::cout << "Configured contact detector successful." << std::endl;
    contactDetector.resetState("l_sole", true);
    contactDetector.resetState("r_sole", true);

    //////////////////////////////////////////////////
    // Initialize buffers for saving data to mat file
    //////////////////////////////////////////////////

    // blf outputs outputs
    Eigen::MatrixXd estBasePos, estBaseRot, estBaseLinVel, estBaseAngVel;
    estBasePos.resize(nrIters, sbasePosEig.cols());
    estBaseRot.resize(nrIters, sbaseRotEig.cols());
    estBaseLinVel.resize(nrIters, sbaseRotEig.cols());
    estBaseAngVel.resize(nrIters, sbaseRotEig.cols());

    Eigen::VectorXd lfContact(nrIters), rfContact(nrIters);

    bool first = true;
    FloatingBaseEstimators::Output out;
    //////////////////////////////////
    //////////////////////////////////
    ////////// Run estimator//////////
    //////////////////////////////////
    //////////////////////////////////
    for (std::size_t iter = 0; iter < nrIters; iter++)
    {
        auto timeNow = timeEig(iter);
        Eigen::VectorXd s = jposEig.row(iter);
        Eigen::VectorXd sdot = jvelEig.row(iter);
        Eigen::Vector3d acc = imuAccEig.row(iter);
        Eigen::Vector3d gyro = imuOmegaEig.row(iter);
        double lfz = lfZEig(iter);
        double rfz = rfZEig(iter);

        // set Timed contact pairs
        contactDetector.setTimedTriggerInput("l_sole", timeNow, lfz);
        contactDetector.setTimedTriggerInput("r_sole", timeNow, rfz);
        REQUIRE(contactDetector.advance());

        auto contactMap = contactDetector.getOutput();

        filter->setIMUMeasurement(acc, gyro);
        filter->setKinematics(s, sdot);
        filter->setContactStatus("l_sole",
                                contactMap.at("l_sole").isActive,
                                contactMap.at("l_sole").switchTime,
                                timeNow);
        filter->setContactStatus("r_sole",
                                contactMap.at("r_sole").isActive,
                                contactMap.at("r_sole").switchTime,
                                timeNow);

        lfContact(iter) = 300 * static_cast<int>(contactMap.at("l_sole").isActive);
        rfContact(iter) = 300 * static_cast<int>(contactMap.at("r_sole").isActive);

        REQUIRE(filter->advance());
                        auto xx = filter->getOutput();
        //                 out = filter.getOutput();

        std::cout << "Iter: " << iter << std::endl;

        //         std::cout <<  "==============> "<< filter.isOutputValid() << std::endl;
        //         out = filter.getOutput();
        //         Eigen::Vector3d pEst = out.basePose.translation();
        //         Eigen::Vector3d rpyEst = out.basePose.rotation().eulerAngles(2, 1, 0).reverse();
        //         estBasePos.row(iter) << pEst(0), pEst(1), pEst(2);
        //         estBaseRot.row(iter) << rpyEst(0), rpyEst(1), rpyEst(2);
        //         estBaseLinVel.row(iter) << out.baseTwist(0), out.baseTwist(1), out.baseTwist(2);
        //         estBaseAngVel.row(iter) << out.baseTwist(3), out.baseTwist(4), out.baseTwist(5);

        //         Eigen::MatrixXd pose = out.basePose.transform();

        std::cout << "============" << std::endl;

        first = false;

    } // end run loop

    //     matioCpp::File file = matioCpp::File::Create("out.mat");
    //     matioCpp::MultiDimensionalArray<double> outEstPos{"estBasePos",
    //                                                     {static_cast<std::size_t>(estBasePos.rows()),
    //                                                     static_cast<std::size_t>(estBasePos.cols())},
    //                                                     estBasePos.data()};
    //     matioCpp::MultiDimensionalArray<double> outEstRot{"estBaseRot",
    //                                                     {static_cast<std::size_t>(estBaseRot.rows()),
    //                                                     static_cast<std::size_t>(estBaseRot.cols())},
    //                                                     estBaseRot.data()};
    //     matioCpp::MultiDimensionalArray<double> outEstLinVel{"estBaseLinVel",
    //                                                     {static_cast<std::size_t>(estBaseLinVel.rows()),
    //                                                     static_cast<std::size_t>(estBaseLinVel.cols())},
    //                                                     estBaseLinVel.data()};
    //     matioCpp::MultiDimensionalArray<double> outEstAngVel{"estBaseAngVel",
    //                                                     {static_cast<std::size_t>(estBaseAngVel.rows()),
    //                                                     static_cast<std::size_t>(estBaseAngVel.cols())},
    //                                                     estBaseAngVel.data()};
    //     auto outContactlf = tomatioCpp(lfContact, "estLFContact");
    //     auto outContactrf = tomatioCpp(rfContact, "estRFContact");
    //
    //     bool write_ok{true};
    //     write_ok = write_ok && file.write(outEstPos);
    //     write_ok = write_ok && file.write(outEstRot);
    //     write_ok = write_ok && file.write(outEstLinVel);
    //     write_ok = write_ok && file.write(outEstAngVel);
    //     write_ok = write_ok && file.write(outContactlf);
    //     write_ok = write_ok && file.write(outContactrf);
    //
    //     REQUIRE(write_ok);

    //     std::cout << "Write to file successful" << std::endl;
}
