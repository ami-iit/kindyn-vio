# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_kindyn_vio_test(
 NAME PointsTracker
 SOURCES PointsTrackerTest.cpp
 LINKS KinDynVIO::TestData KinDynVIO::PerceptionFeatures ${OpenCV_LIBS} BipedalLocomotion::ParametersHandler)

add_kindyn_vio_test(
NAME LinesTracker
SOURCES LinesTrackerTest.cpp
LINKS KinDynVIO::TestData KinDynVIO::PerceptionFeatures ${OpenCV_LIBS} BipedalLocomotion::ParametersHandler)

add_kindyn_vio_test(
 NAME ArucoDetectorExample
 SOURCES ArucoDetectorExample.cpp
 LINKS KinDynVIO::TestData KinDynVIO::PerceptionFeatures BipedalLocomotion::PerceptionFeatures ${OpenCV_LIBS} BipedalLocomotion::ParametersHandler Eigen3::Eigen iDynTree::idyntree-core)
