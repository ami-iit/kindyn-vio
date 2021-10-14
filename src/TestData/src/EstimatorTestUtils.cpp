/**
 * @file EstimatorTestUtils.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <KinDynVIO/TestData/EstimatorTestUtils.h>

using KinDynPtr = std::shared_ptr<iDynTree::KinDynComputations>;
using ParamsHandler = std::shared_ptr<BipedalLocomotion::ParametersHandler::StdImplementation>;

double deg2rad(const double& ang)
{
    return ang * (M_PI / 180);
}

bool readRobotMatFile(const std::string& matFilePath, RobotMatData& rmd)
{
    matioCpp::File input(matFilePath);
    if (!input.isOpen())
    {
        BipedalLocomotion::log()->error("Failed to open matfile.");
        return false;
    }

    auto time = input.read("estimatorJointsTime").asVector<double>();
    auto imgTime = input.read("imgTime").asVector<double>();
    auto jointsPos = input.read("estimatorJointsPos").asMultiDimensionalArray<double>();
    auto jointsVel = input.read("estimatorJointsVel").asMultiDimensionalArray<double>();
    auto simBasePos = input.read("linkBasePos").asMultiDimensionalArray<double>();
    auto simBaseRot = input.read("linkBaseRot").asMultiDimensionalArray<double>();

    auto lfForceZ = input.read("lfForceZ").asMultiDimensionalArray<double>();
    auto rfForceZ = input.read("rfForceZ").asMultiDimensionalArray<double>();
    auto lfWrench = input.read("lfWrench").asMultiDimensionalArray<double>();
    auto rfWrench = input.read("rfWrench").asMultiDimensionalArray<double>();
    auto imuAcc = input.read("imuAcc").asMultiDimensionalArray<double>();
    auto imuOmega = input.read("imuOmega").asMultiDimensionalArray<double>();

    rmd.nrJoints = static_cast<int>(input.read("nr_joints_est").asElement<double>());
    rmd.time = BipedalLocomotion::Conversions::toEigen(time);
    rmd.imgTime = BipedalLocomotion::Conversions::toEigen(imgTime);
    rmd.jpos = BipedalLocomotion::Conversions::toEigen(jointsPos);
    rmd.jvel = BipedalLocomotion::Conversions::toEigen(jointsVel);
    rmd.sbasePos = BipedalLocomotion::Conversions::toEigen(simBasePos);
    rmd.sbaseRot = BipedalLocomotion::Conversions::toEigen(simBaseRot);
    rmd.lfZ = BipedalLocomotion::Conversions::toEigen(lfForceZ);
    rmd.rfZ = BipedalLocomotion::Conversions::toEigen(rfForceZ);
    rmd.lfWrench = BipedalLocomotion::Conversions::toEigen(lfWrench);
    rmd.rfWrench = BipedalLocomotion::Conversions::toEigen(rfWrench);
    rmd.imuAcc = BipedalLocomotion::Conversions::toEigen(imuAcc);
    rmd.imuOmega = BipedalLocomotion::Conversions::toEigen(imuOmega);

    input.close();

    rmd.nrImgs = rmd.imgTime.size();
    rmd.jointsList = {"neck_pitch",     "neck_roll",   "neck_yaw",         "torso_pitch",
                      "torso_roll",     "torso_yaw",   "l_shoulder_pitch", "l_shoulder_roll",
                      "l_shoulder_yaw", "l_elbow",     "r_shoulder_pitch", "r_shoulder_roll",
                      "r_shoulder_yaw", "r_elbow",     "l_hip_pitch",      "l_hip_roll",
                      "l_hip_yaw",      "l_knee",      "l_ankle_pitch",    "l_ankle_roll",
                      "r_hip_pitch",    "r_hip_roll",  "r_hip_yaw",        "r_knee",
                      "r_ankle_pitch",  "r_ankle_roll"};

    return true;
}

bool getAllArucoIMUImageNamesFromDirectory(const std::string& dirPath,
                                           std::vector<std::string>& vec,
                                           std::string extension)
{
    auto dirIter = std::filesystem::directory_iterator(dirPath);
    int filecount = 0;
    for (auto& entry : dirIter)
    {
        if (std::filesystem::path(entry).extension() == extension)
        {
            vec.emplace_back(dirPath + std::string(std::filesystem::path(entry).filename()));
            filecount++;
        }
    }
    std::sort(vec.begin(), vec.end());
    return true;
}

bool getArucoIMUImageNames(const std::string& dirPath,
                           const std::string& textFilePath,
                           std::vector<std::string>& vec)
{
    vec.clear();
    std::ifstream fileOut;
    fileOut.open(textFilePath);

    std::string imgFileName;
    while (std::getline(fileOut, imgFileName))
    {
        vec.emplace_back(dirPath + imgFileName);
    }
    return true;
}

bool writeLoggableToFile(const Loggable& log, const std::string& fileName)
{
    matioCpp::File file = matioCpp::File::Create(fileName);
    bool writeOk{true};

    if (log.estTime.size() > 0)
    {
        matioCpp::MultiDimensionalArray<double> outEstPos{"estPosition",
                                                          {static_cast<std::size_t>(
                                                               log.estPosition.rows()),
                                                           static_cast<std::size_t>(
                                                               log.estPosition.cols())},
                                                          log.estPosition.data()};
        matioCpp::MultiDimensionalArray<double> outEstRot{"estRotation",
                                                          {static_cast<std::size_t>(
                                                               log.estRotation.rows()),
                                                           static_cast<std::size_t>(
                                                               log.estRotation.cols())},
                                                          log.estRotation.data()};
        matioCpp::MultiDimensionalArray<double> outEstTime{"estTime",
                                                           {static_cast<std::size_t>(
                                                                log.estTime.rows()),
                                                            static_cast<std::size_t>(
                                                                log.estTime.cols())},
                                                           log.estTime.data()};
        writeOk = writeOk && file.write(outEstPos);
        writeOk = writeOk && file.write(outEstRot);
        writeOk = writeOk && file.write(outEstTime);
    }
    matioCpp::MultiDimensionalArray<double> outSimPos{"simPosition",
                                                      {static_cast<std::size_t>(
                                                           log.simPosition.rows()),
                                                       static_cast<std::size_t>(
                                                           log.simPosition.cols())},
                                                      log.simPosition.data()};
    matioCpp::MultiDimensionalArray<double> outSimRot{"simRotation",
                                                      {static_cast<std::size_t>(
                                                           log.simRotation.rows()),
                                                       static_cast<std::size_t>(
                                                           log.simRotation.cols())},
                                                      log.simRotation.data()};
    matioCpp::MultiDimensionalArray<double> outSimTime{"simTime",
                                                       {static_cast<std::size_t>(log.simTime.rows()),
                                                        static_cast<std::size_t>(
                                                            log.simTime.cols())},
                                                       log.simTime.data()};

    if (log.smootherTime.size() > 0)
    {
        matioCpp::MultiDimensionalArray<double> outSmoothPos{"smootherPosition",
                                                             {static_cast<std::size_t>(
                                                                  log.smootherPosition.rows()),
                                                              static_cast<std::size_t>(
                                                                  log.smootherPosition.cols())},
                                                             log.smootherPosition.data()};
        matioCpp::MultiDimensionalArray<double> outSmoothRot{"smootherRotation",
                                                             {static_cast<std::size_t>(
                                                                  log.smootherRotation.rows()),
                                                              static_cast<std::size_t>(
                                                                  log.smootherRotation.cols())},
                                                             log.smootherRotation.data()};
        matioCpp::MultiDimensionalArray<double> outSmoothTime{"smootherTime",
                                                              {static_cast<std::size_t>(
                                                                   log.smootherTime.rows()),
                                                               static_cast<std::size_t>(
                                                                   log.smootherTime.cols())},
                                                              log.smootherTime.data()};
        writeOk = writeOk && file.write(outSmoothPos);
        writeOk = writeOk && file.write(outSmoothRot);
        writeOk = writeOk && file.write(outSmoothTime);
    }

    writeOk = writeOk && file.write(outSimPos);
    writeOk = writeOk && file.write(outSimRot);
    writeOk = writeOk && file.write(outSimTime);

    if (!writeOk)
    {
        std::cerr << "Could not write to file" << std::endl;
    }

    return writeOk;
}

KinDynPtr getKinDyn(const std::string& modelFilePath, const std::vector<std::string>& jointsList)
{
    iDynTree::ModelLoader modelLoader;
    modelLoader.loadReducedModelFromFile(modelFilePath, jointsList);
    auto model = modelLoader.model();

    auto kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    kinDyn->loadRobotModel(model);
    return kinDyn;
}

ParamsHandler getSchmittParamsConfig()
{
    auto handle = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();

    handle->setParameter("contacts", std::vector<std::string>{"l_sole", "r_sole"});
    handle->setParameter("contact_make_thresholds", std::vector<double>{150.0, 150.0});
    handle->setParameter("contact_break_thresholds", std::vector<double>{120.0, 120.0});
    handle->setParameter("contact_make_switch_times", std::vector<double>{0.01, 0.01});
    handle->setParameter("contact_break_switch_times", std::vector<double>{0.01, 0.01});
    return handle;
}

ParamsHandler getEstimatorConfig(const EstimatorType& type,
                                 const int& nrJoints,
                                 const iDynTree::Rotation& Rimu0,
                                 const iDynTree::Position& pimu0,
                                 const iDynTree::Rotation& Rlf0,
                                 const iDynTree::Position& plf0,
                                 const iDynTree::Rotation& Rrf0,
                                 const iDynTree::Position& prf0)
{
    auto handle = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();

    handle->setParameter("use_model_info", true);
    handle->setParameter("sampling_period_in_s", 0.01);

    auto modelInfoGroup
        = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    handle->setGroup("ModelInfo", modelInfoGroup);
    modelInfoGroup->setParameter("base_link", "root_link");
    modelInfoGroup->setParameter("base_link_imu", "root_link_imu_acc");
    modelInfoGroup->setParameter("left_foot_contact_frame", "l_sole");
    modelInfoGroup->setParameter("right_foot_contact_frame", "r_sole");

    auto optionsGroup = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    handle->setGroup("Options", optionsGroup);
    optionsGroup->setParameter("enable_imu_bias_estimation", false);
    optionsGroup->setParameter("enable_ekf_update", true);
    optionsGroup->setParameter("use_kinematics_measure", true);
    optionsGroup->setParameter("use_static_ldmks_pose_measure", false);

    if (type == EstimatorType::LeggedOdom)
    {
        auto loGroup = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
        handle->setGroup("LeggedOdom", loGroup);
        loGroup->setParameter("initial_fixed_frame", "r_sole");
        loGroup->setParameter("initial_ref_frame_for_world", "root_link_imu_acc");

        auto refH = iDynTree::Transform(Rimu0, pimu0).inverse();
        auto quat = refH.getRotation().asQuaternion();
        auto p0 = refH.getPosition();

        loGroup->setParameter("initial_world_orientation_in_ref_frame",
                              std::vector<double>{quat(0), quat(1), quat(2), quat(3)});
        loGroup->setParameter("initial_world_position_in_ref_frame",
                              std::vector<double>{p0(0), p0(1), p0(2)});

        auto switching_pattern_for_lo{"lastActive"};
        loGroup->setParameter("switching_pattern", switching_pattern_for_lo);
    }

    if (type == EstimatorType::InvEKF)
    {
        auto sensorsdevGroup
            = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
        handle->setGroup("SensorsStdDev", sensorsdevGroup);
        BipedalLocomotion::ParametersHandler::IParametersHandler::shared_ptr sensdev_handler
            = sensorsdevGroup;
        sensdev_handler->setParameter("accelerometer_measurement_noise_std_dev",
                                      std::vector<double>{0.1, 0.1, 0.1});
        sensdev_handler->setParameter("gyroscope_measurement_noise_std_dev",
                                      std::vector<double>{0.01, 0.01, 0.01});
        sensdev_handler->setParameter("contact_foot_linear_velocity_noise_std_dev",
                                      std::vector<double>{9e-3, 9e-3, 9e-3});
        sensdev_handler->setParameter("contact_foot_angular_velocity_noise_std_dev",
                                      std::vector<double>{4e-3, 4e-3, 4e-3});
        sensdev_handler->setParameter("swing_foot_linear_velocity_noise_std_dev",
                                      std::vector<double>{0.5, 0.5, 0.5});
        sensdev_handler->setParameter("swing_foot_angular_velocity_noise_std_dev",
                                      std::vector<double>{0.05, 0.05, 0.05});
        sensdev_handler->setParameter("forward_kinematic_measurement_noise_std_dev",
                                      std::vector<double>{1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6});
        std::vector<double> encoder_noise(nrJoints);
        for (auto& v : encoder_noise)
        {
            v = deg2rad(1e-5);
        }
        sensdev_handler->setParameter("encoders_measurement_noise_std_dev", encoder_noise);

        auto initStateGroup
            = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
        handle->setGroup("InitialStates", initStateGroup);
        BipedalLocomotion::ParametersHandler::IParametersHandler::shared_ptr initstate_handler
            = initStateGroup;
        initstate_handler->setParameter("imu_orientation_quaternion_wxyz",
                                        std::vector<double>{Rimu0.asQuaternion()(0),
                                                            Rimu0.asQuaternion()(1),
                                                            Rimu0.asQuaternion()(2),
                                                            Rimu0.asQuaternion()(3)});
        initstate_handler->setParameter("imu_position_xyz",
                                        std::vector<double>{pimu0(0), pimu0(1), pimu0(2)});
        initstate_handler->setParameter("imu_linear_velocity_xyz", std::vector<double>{0, 0, 0});
        initstate_handler->setParameter("accelerometer_bias", std::vector<double>{0, 0, 0});
        initstate_handler->setParameter("gyroscope_bias", std::vector<double>{0, 0, 0});
        initstate_handler->setParameter("l_contact_frame_orientation_quaternion_wxyz",
                                        std::vector<double>{Rlf0.asQuaternion()(0),
                                                            Rlf0.asQuaternion()(1),
                                                            Rlf0.asQuaternion()(2),
                                                            Rlf0.asQuaternion()(3)});
        initstate_handler->setParameter("r_contact_frame_orientation_quaternion_wxyz",
                                        std::vector<double>{Rrf0.asQuaternion()(0),
                                                            Rrf0.asQuaternion()(1),
                                                            Rrf0.asQuaternion()(2),
                                                            Rrf0.asQuaternion()(3)});
        initstate_handler->setParameter("l_contact_frame_position_xyz",
                                        std::vector<double>{plf0(0), plf0(1), plf0(2)});
        initstate_handler->setParameter("r_contact_frame_position_xyz",
                                        std::vector<double>{prf0(0), prf0(1), prf0(2)});

        auto priordevGroup
            = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
        handle->setGroup("PriorsStdDev", priordevGroup);
        BipedalLocomotion::ParametersHandler::IParametersHandler::shared_ptr prior_handler
            = priordevGroup;
        prior_handler->setParameter("imu_orientation",
                                    std::vector<double>{deg2rad(1), deg2rad(1), deg2rad(1)});
        prior_handler->setParameter("imu_position", std::vector<double>{1e-6, 1e-6, 1e-6});
        prior_handler->setParameter("imu_linear_velocity", std::vector<double>{1e-6, 1e-6, 1e-6});
        prior_handler->setParameter("accelerometer_bias", std::vector<double>{1e-1, 1e-1, 1e-1});
        prior_handler->setParameter("gyroscope_bias", std::vector<double>{1e-1, 1e-1, 1e-1});
        prior_handler->setParameter("r_contact_frame_orientation",
                                    std::vector<double>{deg2rad(1), deg2rad(1), deg2rad(1)});
        prior_handler->setParameter("l_contact_frame_orientation",
                                    std::vector<double>{deg2rad(1), deg2rad(1), deg2rad(1)});
        prior_handler->setParameter("l_contact_frame_position",
                                    std::vector<double>{1e-6, 1e-6, 1e-6});
        prior_handler->setParameter("r_contact_frame_position",
                                    std::vector<double>{1e-6, 1e-6, 1e-6});
    }

    return handle;
}

ParamsHandler getKinematicInertialWrapperConfig(const EstimatorType& type,
                                                const RobotMatData& rmd,
                                                KinDynPtr kinDyn)
{

    // get initial ground truth pose to initialize estimator
    auto w_p_b0 = iDynTree::Position(rmd.sbasePos.row(0)(0),
                                     rmd.sbasePos.row(0)(1),
                                     rmd.sbasePos.row(0)(2));
    auto w_R_b0 = iDynTree::Rotation::RPY(rmd.sbaseRot.row(0)(0),
                                          rmd.sbaseRot.row(0)(1),
                                          rmd.sbaseRot.row(0)(2));
    auto w_H_b0 = iDynTree::Transform(w_R_b0, w_p_b0);
    Eigen::Matrix<double, 6, 1> zeroTwist;
    zeroTwist.setZero();
    Eigen::Vector3d g;
    g << 0, 0, -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    kinDyn->setRobotState(iDynTree::toEigen(w_H_b0.asHomogeneousTransform()),
                          iDynTree::make_span(rmd.jpos.row(0).data(), rmd.jpos.row(0).size()),
                          zeroTwist,
                          iDynTree::make_span(rmd.jvel.row(0).data(), rmd.jvel.row(0).size()),
                          g);

    auto w_H_imu0 = kinDyn->getWorldTransform("root_link_imu_acc");
    auto Rimu0 = w_H_imu0.getRotation();
    auto pimu0 = w_H_imu0.getPosition();
    auto w_H_lsole0 = kinDyn->getWorldTransform("l_sole");
    auto Rlf0 = w_H_lsole0.getRotation();
    auto plf0 = w_H_lsole0.getPosition();
    auto w_H_rsole0 = kinDyn->getWorldTransform("r_sole");
    auto Rrf0 = w_H_rsole0.getRotation();
    auto prf0 = w_H_rsole0.getPosition();

    auto handle = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    handle->setGroup("BASE_ESTIMATOR",
                     getEstimatorConfig(type, rmd.nrJoints, Rimu0, pimu0, Rlf0, plf0, Rrf0, prf0));
    handle->setGroup("CONTACT_DETECTOR", getSchmittParamsConfig());
    return handle;
}

KinDynVIO::Estimators::ProprioceptiveInput
getProprioceptiveInputFromRobotData(const int& iter, const RobotMatData& rmd)
{
    KinDynVIO::Estimators::ProprioceptiveInput input;
    input.ts = rmd.time(iter) - rmd.time(0);
    input.meas.encoders = rmd.jpos.row(iter);
    input.meas.encodersSpeed = rmd.jvel.row(iter);
    input.meas.acc = rmd.imuAcc.row(iter);
    input.meas.gyro = rmd.imuOmega.row(iter);
    input.contactWrenches["l_sole"] = rmd.lfWrench.row(iter);
    input.contactWrenches["r_sole"] = rmd.rfWrench.row(iter);
    input.lfContact = "l_sole";
    input.rfContact = "r_sole";
    return input;
}

KinDynVIO::Estimators::IMUPreintegratorInput
getIMUPreintegratorInputFromRobotData(const int& iter, const RobotMatData& rmd)
{
    KinDynVIO::Estimators::IMUPreintegratorInput input;
    input.ts = rmd.time(iter) - rmd.time(0);
    input.linAcc = rmd.imuAcc.row(iter);
    input.gyro = rmd.imuOmega.row(iter);

    return input;
}

gtsam::Pose3 getBaseHCam30Degrees()
{
    // obtained from CAD
    Eigen::Matrix4d H;
    H.row(0) << 0., 0.5, -0.866025, -0.082086;
    H.row(1) << 1., 0., 0., -0.0175;
    H.row(2) << 0., -0.866025, -0.5, -0.0621506;
    H.row(3) << 0., 0., 0., 1;

    return gtsam::Pose3(H);
}

gtsam::Pose3 getBaseHIMU()
{
    // obtained from URDF
    Eigen::Matrix4d H;
    H.row(0) << 0., -0.5, 0.866025, 0.0851554;
    H.row(1) << -1., 0., 0., -0.011;
    H.row(2) << 0., -0.866025, -0.5, -0.112309;
    H.row(3) << 0., 0., 0., 1;
    return gtsam::Pose3(H);
}

void getCameraIntrinsics(std::vector<double>& K, std::vector<double>& gamma)
{
    // Obtained from the realsense camera used for capturing the images
    K = {618.967163085938, 0., 323.237915039062, 0., 619.085632324219, 238.748596191406, 0., 0., 1.};
    gamma = {0.0, 0.0, 0.0, 0.0, 0.0};
}

ParamsHandler getArucoConfig()
{
    std::vector<double> K, gamma;
    getCameraIntrinsics(K, gamma);

    auto parameterHandler
        = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    parameterHandler->setParameter("marker_dictionary", "4X4_50");
    parameterHandler->setParameter("marker_length", 0.075);
    parameterHandler->setParameter("camera_matrix", K);
    parameterHandler->setParameter("distortion_coefficients", gamma);
    return parameterHandler;
}

ParamsHandler getIMUPreintConfig()
{
    auto parameterHandler
        = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    parameterHandler->setParameter("sigma_acc", 0.1);
    parameterHandler->setParameter("sigma_gyro", 0.01);
    parameterHandler->setParameter("sigma_b_acc", 1e-6);
    parameterHandler->setParameter("sigma_b_gyro", 1e-6);
    parameterHandler->setParameter("sigma_pos_integration", 1e6);
    parameterHandler->setParameter("error_bias", 1e-4);
    parameterHandler->setParameter("initial_bias", std::vector<double>{0., 0., 0., 0., 0., 0.});

    return parameterHandler;
}

ParamsHandler getVisionFrontEndConfig()
{
    auto handle = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();

    // Obtained from the realsense camera used for capturing the images
    std::vector<double>
        K{616.873107910156, 0., 314.136962890625, 0., 617.2548828125, 244.331726074219, 0., 0., 1.};
    std::vector<double> gamma{0.0, 0.0, 0.0, 0.0, 0.0};

    auto camModelGroup = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    handle->setGroup("CAMERA_MODEL", camModelGroup);
    camModelGroup->setParameter("width", 640);
    camModelGroup->setParameter("height", 480);
    camModelGroup->setParameter("camera_matrix", K);
    camModelGroup->setParameter("distortion_coefficients", gamma);

    auto imgProcGroup = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    handle->setGroup("IMAGE_PROCESSOR", imgProcGroup);
    imgProcGroup->setParameter("tracker_type", "points");
    imgProcGroup->setParameter("equalize_image", false);
    imgProcGroup->setParameter("debug", true);
    imgProcGroup->setParameter("force_features_from_aruco", false);

    auto arucoDetGroup = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    handle->setGroup("ARUCO_DETECTOR", arucoDetGroup);
    arucoDetGroup->setParameter("marker_dictionary", "4X4_50");
    arucoDetGroup->setParameter("marker_length", 0.075);
    arucoDetGroup->setParameter("camera_matrix", K);
    arucoDetGroup->setParameter("distortion_coefficients", gamma);

    auto fMgrGroup = std::make_shared<BipedalLocomotion::ParametersHandler::StdImplementation>();
    handle->setGroup("FEATURE_MANAGER", fMgrGroup);
    fMgrGroup->setParameter("wait_frames_for_zero_motion", 10);
    fMgrGroup->setParameter("zero_motion_threshold", 5.0);
    fMgrGroup->setParameter("disparity_threshold_for_new_keyframe", 0.5);
    fMgrGroup->setParameter("wait_frames_for_pruning", 20);
    fMgrGroup->setParameter("keyframe_history_size", 5);

    return handle;
}

