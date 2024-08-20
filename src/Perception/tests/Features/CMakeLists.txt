# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

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
