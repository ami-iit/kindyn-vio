/*
 * Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include <catch2/catch.hpp>
#include <memory>

#include <KinDynVIO/Estimators/ArucoWrapper.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>

#include "ResourceFolderPath.h"

using namespace KinDynVIO::Estimators;
using namespace BipedalLocomotion::ParametersHandler;

TEST_CASE("Aruco Wrapper Test")
{
    bool debug{false};

    auto imgsPath = getMinimumArucoImagesDirectoryPath();
    std::string frame1Path{imgsPath + "Frame1.jpg"};
    auto frame1 = cv::imread(frame1Path);

    std::string frame2Path{imgsPath + "Frame2.jpg"};
    auto frame2 = cv::imread(frame2Path);


    // Initialize the detector
    std::shared_ptr<IParametersHandler> parameterHandler = std::make_shared<StdImplementation>();
    // Obtained from the realsense camera used for capturing the images
    std::vector<double>
        K{616.873107910156, 0., 314.136962890625, 0., 617.2548828125, 244.331726074219, 0., 0., 1.};
    std::vector<double> gamma{0.0, 0.0, 0.0, 0.0, 0.0};

    parameterHandler->setParameter("marker_dictionary", "4X4_50");
    parameterHandler->setParameter("marker_length", 0.075);
    parameterHandler->setParameter("camera_matrix", K);
    parameterHandler->setParameter("distortion_coefficients", gamma);

    ArucoWrapper wrapper;
    REQUIRE(wrapper.setBaseLinkCameraExtrinsics(gtsam::Pose3()));
    REQUIRE(wrapper.initialize(parameterHandler));

    ArucoWrapperInput input;
    input.Xhat.identity();
    frame1.copyTo(input.imgTs.img);
    input.imgTs.ts = 0.01;

    REQUIRE(wrapper.setInput(input));
    REQUIRE(wrapper.advance());
    auto out = wrapper.getOutput();

    auto nrDetMarkers = out.detectorOut.markers.size();
    if (nrDetMarkers > 0)
    {
        REQUIRE(out.isKeyFrame);
        REQUIRE(out.markerPriors.size() == nrDetMarkers);
        REQUIRE(out.markerMeasures.size() == nrDetMarkers);
    }

    if (debug)
    {
        std::cout << "Detected Markers in Frame1.jpg: ";
        for (const auto& [id, marker] : out.detectorOut.markers)
        {
            std::cout << " " << id;
        }
        std::cout << std::endl;
    }


    frame2.copyTo(input.imgTs.img);
    input.imgTs.ts = 0.02;
    REQUIRE(wrapper.setInput(input));
    REQUIRE(wrapper.advance());
    out = wrapper.getOutput();
    nrDetMarkers = out.detectorOut.markers.size();
    if (debug)
    {
        std::cout << "Detected Markers in Frame2.jpg: ";
        for (const auto& [id, marker] : out.detectorOut.markers)
        {
            std::cout << " " << id;
        }
        std::cout << std::endl;
    }

    // There are no newly detected markers in frame 2, sonr priors must be zero
    if (nrDetMarkers > 0)
    {
        REQUIRE(out.isKeyFrame);
        REQUIRE(out.markerPriors.size() == 0);
        REQUIRE(out.markerMeasures.size() == nrDetMarkers);
    }

}
