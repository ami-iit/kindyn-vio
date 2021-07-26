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
#include <iDynTree/Core/EigenHelpers.h>

#include <cmath>
#include <iostream>

#include <PerceptionFeaturesResourceFolderPath.h>

#include <KinDynVIO/Perception/CameraModels/PinHoleCamera.h>
#include <KinDynVIO/Perception/Features/ImageProcessor.h>

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

TEST_CASE("Point Tracker Unit Test")
{
    std::string frame1Path{getCameraFrame1()};
    auto frame1 = cv::imread(frame1Path);

    std::string frame2Path{getCameraFrame2()};
    auto frame2 = cv::imread(frame2Path);

    std::string frame3Path{getCameraFrame3()};
    auto frame3 = cv::imread(frame3Path);

    auto camera = getCamera(frame1.rows, frame1.cols);

    std::shared_ptr<IParametersHandler> parameterHandler = std::make_shared<StdImplementation>();
    parameterHandler->setParameter("tracker_type", "points");
    parameterHandler->setParameter("equalize_image", false);
    parameterHandler->setParameter("debug", true);

    ImageProcessor imgProc;
    REQUIRE(imgProc.initialize(parameterHandler));
    REQUIRE(imgProc.setCameraModel(camera));

    cv::Mat outImg;
    for (auto idx = 0; idx < 10; idx++)
    {
        if (idx % 2 == 0)
        {
            imgProc.setImage(frame1, idx * dt);
        } else
        {
            imgProc.setImage(frame2, idx * dt);
        }
        REQUIRE(imgProc.advance());
        imgProc.getImageWithDetectedFeatures(outImg);
        //cv::imshow("processed Frame", outImg);
        //cv::waitKey(1000);
    }

    imgProc.setImage(frame3, 10 * dt);
    REQUIRE(imgProc.advance());
    imgProc.getImageWithDetectedFeatures(outImg);
    //cv::imshow("processed Frame", outImg);
    //cv::waitKey();
}
