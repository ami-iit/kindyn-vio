/**
 * @file EstimatorUtils.h
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef KINDYNVIO_TEST_DATA_ESTIMATOR_UTILS_H
#define KINDYNVIO_TEST_DATA_ESTIMATOR_UTILS_H

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/Conversions/matioCppConversions.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/Math/Constants.h>

#include <KinDynVIO/Estimators/IO.h>

#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>

#include <gtsam/geometry/Pose3.h>

#include <matioCpp/matioCpp.h>
#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <vector>

using KinDynPtr = std::shared_ptr<iDynTree::KinDynComputations>;
using ParamsHandler = std::shared_ptr<BipedalLocomotion::ParametersHandler::StdImplementation>;

enum class EstimatorType
{
    LeggedOdom,
    InvEKF
};

struct Loggable
{
    Eigen::VectorXd simTime;
    Eigen::MatrixXd simPosition;
    Eigen::MatrixXd simRotation;

    Eigen::VectorXd estTime;
    Eigen::MatrixXd estPosition;
    Eigen::MatrixXd estRotation;

    Eigen::VectorXd smootherTime;
    Eigen::MatrixXd smootherPosition;
    Eigen::MatrixXd smootherRotation;
};

struct RobotMatData
{
    Eigen::VectorXd time;
    Eigen::MatrixXd jpos;
    Eigen::MatrixXd jvel;
    Eigen::MatrixXd sbasePos;
    Eigen::MatrixXd sbaseRot;
    Eigen::MatrixXd lfZ;
    Eigen::MatrixXd rfZ;
    Eigen::MatrixXd lfWrench;
    Eigen::MatrixXd rfWrench;
    Eigen::MatrixXd imuAcc;
    Eigen::MatrixXd imuOmega;
    Eigen::VectorXd imgTime;
    int nrJoints;
    int nrImgs;
    std::vector<std::string> jointsList;
};

double deg2rad(const double& ang);

bool readRobotMatFile(const std::string& matFilePath, RobotMatData& rmd);
bool getAllArucoIMUImageNamesFromDirectory(const std::string& dirPath, std::vector<std::string>& vec, std::string extension = ".jpg");
bool getArucoIMUImageNames(const std::string& dirPath, const std::string& textFilePath, std::vector<std::string>& vec);


bool writeLoggableToFile(const Loggable& log, const std::string& fileName);

KinDynPtr getKinDyn(const std::string& modelFilePath,
                    const std::vector<std::string>& jointsList);
ParamsHandler getSchmittParamsConfig();
ParamsHandler getEstimatorConfig(const EstimatorType& type,
                                 const int& nrJoints,
                                 const iDynTree::Rotation& Rimu0,
                                 const iDynTree::Position& pimu0,
                                 const iDynTree::Rotation& Rlf0,
                                 const iDynTree::Position& plf0,
                                 const iDynTree::Rotation& Rrf0,
                                 const iDynTree::Position& prf0);
ParamsHandler getKinematicInertialWrapperConfig(const EstimatorType& type,
                                                const RobotMatData& rmd,
                                                KinDynPtr kinDyn);
ParamsHandler getArucoConfig();
ParamsHandler getIMUPreintConfig();
ParamsHandler getVisionFrontEndConfig();

KinDynVIO::Estimators::ProprioceptiveInput
getProprioceptiveInputFromRobotData(const int& iter,
                                    const RobotMatData& rmd);
KinDynVIO::Estimators::IMUPreintegratorInput
getIMUPreintegratorInputFromRobotData(const int& iter,
                                      const RobotMatData& rmd);


gtsam::Pose3 getBaseHCam30Degrees();
gtsam::Pose3 getBaseHIMU();
void getCameraIntrinsics(std::vector<double>& K,
                         std::vector<double>& gamma);


#endif // KINDYNVIO_TEST_DATA_ESTIMATOR_UTILS_H
