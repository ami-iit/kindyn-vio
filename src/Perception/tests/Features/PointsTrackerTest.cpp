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
#include <iDynTree/Core/EigenHelpers.h>
#include <Eigen/Dense>

#include <iostream>
#include <cmath>

#include <PerceptionFeaturesResourceFolderPath.h>

#include <KinDynVIO/Perception/Features/ImageProcessor.h>
#include <KinDynVIO/Perception/CameraModels/PinHoleCamera.h>

using namespace BipedalLocomotion::Perception;
using namespace KinDynVIO::Perception;
using namespace BipedalLocomotion::ParametersHandler;

constexpr double dt{0.066};

// bool getCameraMotionFromArucoMarkers(const cv::Mat& frame1, const cv::Mat& frame2, iDynTree::Transform& c1_H_c2)
// {
//     // Initialize the detector
//     std::shared_ptr<IParametersHandler> parameterHandler = std::make_shared<StdImplementation>();
//     // Obtained from the realsense camera used for capturing the images
//     std::vector<double> K{616.873107910156,             0., 314.136962890625,
//                                         0., 617.2548828125, 244.331726074219,
//                           0.,             0.,               1.};
//     std::vector<double> gamma{0.0, 0.0, 0.0, 0.0, 0.0};
//
//     parameterHandler->setParameter("marker_dictionary", "4X4_50");
//     parameterHandler->setParameter("marker_length", 0.075);
//     parameterHandler->setParameter("camera_matrix",   K);
//     parameterHandler->setParameter("distortion_coefficients", gamma);
//
//     ArucoDetector detector;
//     if (!detector.initialize(parameterHandler))
//     {
//         return false;
//     }
//
//     // define placeholders
//     cv::Mat outputImage;
//     ArucoMarkerData marker9;
//     Eigen::Matrix4Xd m9Pose;
//
//     // get Marker 9 from image 1
//     bool ok{true};
//     ok = ok && detector.setImage(frame1, dt);
//     ok = ok && detector.advance();
//     ok = ok && detector.getDetectedMarkerData(/*id=*/ 9, marker9);
//     if (!ok)
//     {
//         return false;
//     }
//     m9Pose = marker9.pose;
//     iDynTree::Transform c1_H_m9;
//     iDynTree::fromEigen(c1_H_m9, m9Pose);
//     std::cout << "Marker 9 pose in camera frame 1: \n" << m9Pose << std::endl;
//
// //     // uncomment this block to view the output image
// //     detector.getImageWithDetectedMarkers(outputImage,
// //                                          /*drawFrames=*/ true,
// //                                          /*axisLengthForDrawing=*/ 0.1);
// //     cv::imshow("outputImage", outputImage);
// //     cv::waitKey();
//
//     // get Marker 9 from image 2
//     ok = ok && detector.setImage(frame2, 2*dt);
//     ok = ok && detector.advance();
//     ok = ok && detector.getDetectedMarkerData(/*id=*/ 9, marker9);
//     if (!ok)
//     {
//         return false;
//     }
//     m9Pose = marker9.pose;
//     iDynTree::Transform c2_H_m9;
//     iDynTree::fromEigen(c2_H_m9, m9Pose);
//     std::cout << "Marker 9 pose in camera frame 2: \n" << m9Pose << std::endl;
//
// //     // uncomment this block to view the output image
// //     detector.getImageWithDetectedMarkers(outputImage,
// //                                          /*drawFrames=*/ true,
// //                                          /*axisLengthForDrawing=*/ 0.1);
// //     cv::imshow("outputImage", outputImage);
// //     cv::waitKey();
//
//     c1_H_c2 = c1_H_m9*(c2_H_m9.inverse());
//
//     return true;
// }

std::shared_ptr<PinHoleCamera> getCamera(int row, int col)
{
    std::shared_ptr<IParametersHandler> parameterHandler = std::make_shared<StdImplementation>();
    // Obtained from the realsense camera used for capturing the images
    std::vector<double> K{616.873107910156,             0., 314.136962890625,
                                        0., 617.2548828125, 244.331726074219,
                          0.,             0.,               1.};
    std::vector<double> gamma{0.0, 0.0, 0.0, 0.0, 0.0};

    parameterHandler->setParameter("width", col);
    parameterHandler->setParameter("height", row);
    parameterHandler->setParameter("camera_matrix",   K);
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

    auto camera = getCamera(frame1.rows, frame1.cols);

//     iDynTree::Transform c1_H_c2;
//     REQUIRE(getCameraMotionFromArucoMarkers(frame1, frame2, c1_H_c2));
//     std::cout << "Camera pose from frame 1 to frame 2: \n" << iDynTree::toEigen(c1_H_c2.asHomogeneousTransform()) << std::endl;

    std::shared_ptr<IParametersHandler> parameterHandler = std::make_shared<StdImplementation>();
    parameterHandler->setParameter("tracker_type", "points");
    parameterHandler->setParameter("equalize_image", false);
    parameterHandler->setParameter("debug", true);

    ImageProcessor imgProc;
    REQUIRE(imgProc.initialize(parameterHandler));
    REQUIRE(imgProc.setCameraModel(camera));
    imgProc.setImage(frame1, dt);
    imgProc.advance();

    cv::Mat outImg1;
    imgProc.getImageWithDetectedFeatures(outImg1);

    imgProc.setImage(frame2, 2*dt);
    imgProc.advance();

    cv::Mat outImg2;
    imgProc.getImageWithDetectedFeatures(outImg2);

//     cv::imshow("processed Frame 1", outImg1);
//     cv::imshow("processed Frame 2", outImg2);
}
