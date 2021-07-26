# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

include_directories(${CMAKE_CURRENT_BINARY_DIR})
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/PerceptionFeaturesResourceFolderPath.h.in" "${CMAKE_CURRENT_BINARY_DIR}/PerceptionFeaturesResourceFolderPath.h" @ONLY)

add_kindyn_vio_test(
 NAME PointsTracker
 SOURCES PointsTrackerTest.cpp
 LINKS KinDynVIO::PerceptionFeatures BipedalLocomotion::PerceptionFeatures ${OpenCV_LIBS} BipedalLocomotion::ParametersHandler Eigen3::Eigen iDynTree::idyntree-core)
 
#add_kindyn_vio_test(
# NAME LinesTracker
# SOURCES LinesTrackerTest.cpp
# LINKS KinDynVIO::PerceptionFeatures BipedalLocomotion::PerceptionFeatures ${OpenCV_LIBS} #BipedalLocomotion::ParametersHandler Eigen3::Eigen iDynTree::idyntree-core)

add_kindyn_vio_test(
 NAME ArucoDetectorExample
 SOURCES ArucoDetectorExample.cpp
 LINKS KinDynVIO::PerceptionFeatures BipedalLocomotion::PerceptionFeatures ${OpenCV_LIBS} BipedalLocomotion::ParametersHandler Eigen3::Eigen iDynTree::idyntree-core)
