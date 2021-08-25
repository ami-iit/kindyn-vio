# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(H_PREFIX include/KinDynVIO/Perception/CameraModels)
add_kindyn_vio_library(
NAME                   PerceptionCameraModels
SOURCES                src/PinHoleCamera.cpp
PUBLIC_HEADERS         ${H_PREFIX}/PinHoleCamera.h
SUBDIRECTORIES         tests/CameraModels
PUBLIC_LINK_LIBRARIES  ${OpenCV_LIBS} BipedalLocomotion::ParametersHandler BipedalLocomotion::TextLogging
INSTALLATION_FOLDER    Perception/CameraModels)

set(H_PREFIX include/KinDynVIO/Perception/Features)
add_kindyn_vio_library(
NAME                   PerceptionFeatures
SOURCES                src/ImageProcessor.cpp src/PointsTracker.cpp src/LinesTracker.cpp src/FeaturesManager.cpp
PUBLIC_HEADERS         ${H_PREFIX}/DataTypes.h ${H_PREFIX}/ImageProcessor.h ${H_PREFIX}/PointsTracker.h ${H_PREFIX}/LinesTracker.h ${H_PREFIX}/FeaturesManager.h
SUBDIRECTORIES         tests/Features
PUBLIC_LINK_LIBRARIES  ${OpenCV_LIBS} KinDynVIO::PerceptionCameraModels BipedalLocomotion::ParametersHandler BipedalLocomotion::TextLogging BipedalLocomotion::System
INSTALLATION_FOLDER    Perception/Features)


