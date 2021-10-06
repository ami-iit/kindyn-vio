/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include <catch2/catch.hpp>
#include <opencv2/opencv.hpp>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/Perception/Features/ArucoDetector.h>
#include <Eigen/Dense>

#include <cmath>
#include <iostream>

#include "ResourceFolderPath.h"

#include <KinDynVIO/Perception/CameraModels/PinHoleCamera.h>
#include <KinDynVIO/Perception/Features/ImageProcessor.h>
#include <KinDynVIO/Perception/Features/FeaturesManager.h>

using namespace BipedalLocomotion::Perception;
using namespace KinDynVIO::Perception;
using namespace BipedalLocomotion::ParametersHandler;

constexpr double dt{0.066};

std::shared_ptr<PinHoleCamera> getCamera(int row, int col)
{
    std::shared_ptr<IParametersHandler> parameterHandler = std::make_shared<StdImplementation>();
    // Obtained from the realsense camera used for capturing the images
    std::vector<double>
        K{616.873107910156, 0., 314.136962890625, 0., 617.2548828125, 244.331726074219, 0., 0., 1.};
    std::vector<double> gamma{0.0, 0.0, 0.0, 0.0, 0.0};

    parameterHandler->setParameter("width", col);
    parameterHandler->setParameter("height", row);
    parameterHandler->setParameter("camera_matrix", K);
    parameterHandler->setParameter("distortion_coefficients", gamma);

    auto camera = std::make_shared<PinHoleCamera>();
    if (!camera->initialize(parameterHandler))
    {
        return nullptr;
    }

    return camera;
}

std::shared_ptr<ArucoDetector> getArucoDetector()
{
    std::shared_ptr<IParametersHandler> parameterHandler = std::make_shared<StdImplementation>();
    // Obtained from the realsense camera used for capturing the images
    // Obtained from the realsense camera used for capturing the images
    std::vector<double>
        K{616.873107910156, 0., 314.136962890625, 0., 617.2548828125, 244.331726074219, 0., 0., 1.};
    std::vector<double> gamma{0.0, 0.0, 0.0, 0.0, 0.0};

    parameterHandler->setParameter("marker_dictionary", "4X4_50");
    parameterHandler->setParameter("marker_length", 0.075);
    parameterHandler->setParameter("camera_matrix", K);
    parameterHandler->setParameter("distortion_coefficients", gamma);

    auto detector = std::make_shared<ArucoDetector>();
    if (!detector->initialize(parameterHandler))
    {
        return nullptr;
    }

    return detector;
}

TEST_CASE("Aruco Lines Tracker with Feature Manager Unit Test")
{
    auto imgsPath = getMinimumArucoImagesDirectoryPath();
    std::string frame1Path{imgsPath + "Frame1.jpg"};
    auto frame1 = cv::imread(frame1Path);

    std::string frame2Path{imgsPath + "Frame2.jpg"};
    auto frame2 = cv::imread(frame2Path);

    std::string frame3Path{imgsPath + "Frame3.jpg"};
    auto frame3 = cv::imread(frame3Path);

    auto camera = getCamera(frame1.rows, frame1.cols);
    auto arucoDetector = getArucoDetector();

    std::shared_ptr<IParametersHandler> parameterHandler = std::make_shared<StdImplementation>();
    parameterHandler->setParameter("tracker_type", "lines");
    parameterHandler->setParameter("equalize_image", false);
    parameterHandler->setParameter("debug", true);
    parameterHandler->setParameter("force_features_from_aruco", true);

    ImageProcessor imgProc;
    REQUIRE(imgProc.initialize(parameterHandler));
    REQUIRE(imgProc.setCameraModel(camera));
    imgProc.setImage(frame1, dt);
    REQUIRE(imgProc.advance());
    REQUIRE(imgProc.setArucoDetector(arucoDetector));

    FeatureManager fMgr;

    cv::Mat outImg;
    for (auto idx = 0; idx < 5; idx++)
    {
        //std::cout << "========== Iter : "<< idx << "========= " << std::endl;
        if (idx % 2 == 0)
        {
            imgProc.setImage(frame1, idx * dt);
        } else
        {
            imgProc.setImage(frame2, idx * dt);
        }

        REQUIRE(imgProc.advance());
        imgProc.getImageWithDetectedFeatures(outImg);


        auto trackedFeatures = imgProc.getOutput();

        fMgr.setCurrentFrameFeatures(trackedFeatures);
        fMgr.advance();

        //cv::imshow("processed Frame", outImg);
        //cv::waitKey();
        //std::cout << "=================== " << std::endl;

    }

    for (auto idx = 5; idx < 10; idx++)
    {
        //std::cout << "========== Iter : "<< idx << "========= " << std::endl;
        imgProc.setImage(frame3, idx * dt);
        REQUIRE(imgProc.advance());
        imgProc.getImageWithDetectedFeatures(outImg);

        auto trackedFeatures = imgProc.getOutput();
        fMgr.setCurrentFrameFeatures(trackedFeatures);
        fMgr.advance();

        //cv::imshow("processed Frame", outImg);
        //cv::waitKey();
        //std::cout << "=================== " << std::endl;
    }
}

