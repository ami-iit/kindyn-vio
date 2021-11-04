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
    Eigen::MatrixXd simVelocity;
    Eigen::MatrixXd simPosition;
    Eigen::MatrixXd simRotation;
    Eigen::MatrixXd simCOM;
    Eigen::MatrixXd simCOMVel;
    Eigen::MatrixXd simAngMomentum;

    Eigen::VectorXd estTime;
    Eigen::MatrixXd estVelocity;
    Eigen::MatrixXd estPosition;
    Eigen::MatrixXd estRotation;

    Eigen::VectorXd smootherTime;
    Eigen::MatrixXd smootherLinearVelocity;
    Eigen::MatrixXd smootherPosition;
    Eigen::MatrixXd smootherRotation;

    Eigen::MatrixXd smootherCOM;
    Eigen::MatrixXd smootherCOMVel;
    Eigen::MatrixXd smootherAngMomentum;
    Eigen::MatrixXd smootherCDMBias;
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

    // update internal
    Eigen::MatrixXd sbaseVel;
};

double deg2rad(const double& ang);

bool readRobotMatFile(const std::string& matFilePath, RobotMatData& rmd);
bool getAllArucoIMUImageNamesFromDirectory(const std::string& dirPath, std::vector<std::string>& vec, std::string extension = ".jpg");
bool getArucoIMUImageNames(const std::string& dirPath, const std::string& textFilePath, std::vector<std::string>& vec);

template <typename EigenType>
bool checkAndWriteToFile(const std::string& varName,
                         const EigenType& data,
                         matioCpp::File& file)
{
    if (std::is_same_v<EigenType, Eigen::VectorXd>)
    {
        if (data.size() < 1)
        {
            // skip writing
            std::cout << "Skipped writing "
                      << varName << " to file."
                      << std::endl;
            return true;
        }
    }
    else if (std::is_same_v<EigenType, Eigen::MatrixXd>)
    {
        if (data.rows() < 1)
        {
            // skip writing
            std::cout << "Skipped writing "
                      << varName << " to file."
                      << std::endl;
            return true;
        }
    }

    matioCpp::MultiDimensionalArray<double> out{varName,
                                            {static_cast<std::size_t>(data.rows()),
                                             static_cast<std::size_t>(data.cols())},
                                            data.data()};
    return file.write(out);
}

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
ParamsHandler getCentroidalDynPreintConfig();

KinDynVIO::Estimators::ProprioceptiveInput
getProprioceptiveInputFromRobotData(const int& iter,
                                    const RobotMatData& rmd);
KinDynVIO::Estimators::IMUPreintegratorInput
getIMUPreintegratorInputFromRobotData(const int& iter,
                                      const RobotMatData& rmd);

Eigen::Vector3d getGravity();
gtsam::Pose3 getBaseHCam30Degrees();
gtsam::Pose3 getBaseHIMU();
void getCameraIntrinsics(std::vector<double>& K,
                         std::vector<double>& gamma);

void updateRobotMatBaseVelocity(RobotMatData& rmd);

template <typename ArrayType, typename DataType>
void updateLogData(const DataType& data, ArrayType& log)
{
    std::size_t tail;
    if constexpr (std::is_same_v<ArrayType, Eigen::VectorXd> &&
        std::is_same_v<DataType, double>)
    {
        tail = log.size() + 1;
        log.conservativeResize(tail);
        log(tail - 1) = data;
    }
    else if constexpr (std::is_same_v<ArrayType, Eigen::MatrixXd>)
    {
        // expects data type to be a Eigen::VectorXd or Eigen::Ref
        tail = log.rows() + 1;
        log.conservativeResize(tail, data.size());
        log.row(tail - 1) = data;
    }
}

#endif // KINDYNVIO_TEST_DATA_ESTIMATOR_UTILS_H
