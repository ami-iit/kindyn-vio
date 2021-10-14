# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ResourceFolderPath.h.in" "${CMAKE_CURRENT_BINARY_DIR}/ResourceFolderPath.h" @ONLY)
add_kindyn_vio_library(
NAME                   TestData
SOURCES                src/EstimatorTestUtils.cpp
PUBLIC_HEADERS         include/KinDynVIO/TestData/EstimatorTestUtils.h
PUBLIC_LINK_LIBRARIES  KinDynVIO::Estimators BipedalLocomotion::ParametersHandler BipedalLocomotion::TextLogging BipedalLocomotion::matioCppConversions BipedalLocomotion::Math matioCpp::matioCpp BipedalLocomotion::TextLogging iDynTree::idyntree-high-level iDynTree::idyntree-core iDynTree::idyntree-modelio-urdf
INSTALLATION_FOLDER    TestData)

## add the binary tree to the search path for include files
## so that we will find ResourceFolderPath.h
target_include_directories(TestData INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>")

