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
#include <KinDynVIO/Perception/Pipelines/VisionFrontEnd.h>
#include <KinDynVIO/Perception/Features/DataTypes.h>

#include <iostream>
#include <cmath>

#include "ResourceFolderPath.h"

using namespace BipedalLocomotion::Perception;
using namespace KinDynVIO::Perception;
using namespace BipedalLocomotion::ParametersHandler;

constexpr double dt{0.066};

std::shared_ptr<StdImplementation> getVisionFrontEndConfiguration(int row, int col)
{
    auto handle = std::make_shared<StdImplementation>();

    // Obtained from the realsense camera used for capturing the images
    std::vector<double>
        K{616.873107910156, 0., 314.136962890625, 0., 617.2548828125, 244.331726074219, 0., 0., 1.};
    std::vector<double> gamma{0.0, 0.0, 0.0, 0.0, 0.0};

    auto camModelGroup = std::make_shared<StdImplementation>();
    handle->setGroup("CAMERA_MODEL", camModelGroup);
    camModelGroup->setParameter("width", col);
    camModelGroup->setParameter("height", row);
    camModelGroup->setParameter("camera_matrix", K);
    camModelGroup->setParameter("distortion_coefficients", gamma);

    auto imgProcGroup = std::make_shared<StdImplementation>();
    handle->setGroup("IMAGE_PROCESSOR", imgProcGroup);
    imgProcGroup->setParameter("tracker_type", "points_and_lines");
    imgProcGroup->setParameter("equalize_image", false);
    imgProcGroup->setParameter("debug", true);
    imgProcGroup->setParameter("force_features_from_aruco", true);

    auto arucoDetGroup = std::make_shared<StdImplementation>();
    handle->setGroup("ARUCO_DETECTOR", arucoDetGroup);
    arucoDetGroup->setParameter("marker_dictionary", "4X4_50");
    arucoDetGroup->setParameter("marker_length", 0.075);
    arucoDetGroup->setParameter("camera_matrix", K);
    arucoDetGroup->setParameter("distortion_coefficients", gamma);

    auto fMgrGroup = std::make_shared<StdImplementation>();
    handle->setGroup("FEATURE_MANAGER", fMgrGroup);
    fMgrGroup->setParameter("wait_frames_for_zero_motion", 10);
    fMgrGroup->setParameter("zero_motion_threshold", 0.5);
    fMgrGroup->setParameter("disparity_threshold_for_new_keyframe", 20.0);
    fMgrGroup->setParameter("wait_frames_for_pruning", 20);
    fMgrGroup->setParameter("keyframe_history_size", 5);

    return handle;
}


TEST_CASE("Vision Front End Unit Test")
{
    auto imgsPath = getMinimumArucoImagesDirectoryPath();
    std::string frame1Path{imgsPath + "Frame1.jpg"};
    auto frame1 = cv::imread(frame1Path);

    std::string frame2Path{imgsPath + "Frame2.jpg"};
    auto frame2 = cv::imread(frame2Path);

    std::string frame3Path{imgsPath + "Frame3.jpg"};
    auto frame3 = cv::imread(frame3Path);

    VisionFrontEnd visio;
    auto parameterHandler = getVisionFrontEndConfiguration(frame1.rows, frame1.cols);
    REQUIRE(visio.initialize(parameterHandler));

    cv::Mat outImg;
    for (auto idx = 0; idx < 5; idx++)
    {
        std::cout << "========== Iter : "<< idx << "========= " << std::endl;
        if (idx % 2 == 0)
        {
            visio.setInput({frame1, idx * dt});
        } else
        {
            visio.setInput({frame2, idx * dt});
        }

        REQUIRE(visio.advance());
        visio.getImageWithDetectedFeatures(outImg);

        auto kfFeature = visio.getOutput();
        auto kfFeatureHistory = visio.featureManager().getKeyFramesHistory();

        //visio.featureManager().printFeatureTracks();
        //cv::imshow("processed Frame", outImg);
        //cv::waitKey();
        //std::cout << "=================== " << std::endl;
    }

    for (auto idx = 5; idx < 10; idx++)
    {
        std::cout << "========== Iter : "<< idx << "========= " << std::endl;
        visio.setInput({frame3, idx * dt});
        REQUIRE(visio.advance());
        visio.getImageWithDetectedFeatures(outImg);
        auto kfFeature = visio.getOutput();
        auto kfFeatureHistory = visio.featureManager().getKeyFramesHistory();

        //visio.featureManager().printFeatureTracks();
        //cv::imshow("processed Frame", outImg);
        //cv::waitKey();
        //std::cout << "=================== " << std::endl;
    }
}

