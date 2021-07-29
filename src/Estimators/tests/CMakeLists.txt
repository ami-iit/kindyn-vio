# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

include_directories(${CMAKE_CURRENT_BINARY_DIR})
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ResourceFolderPath.h.in" "${CMAKE_CURRENT_BINARY_DIR}/ResourceFolderPath.h" @ONLY)

add_kindyn_vio_test(
 NAME LeggedOdometryiSAM2
 SOURCES LeggedOdometryiSAM2.cpp
 LINKS BipedalLocomotion::FloatingBaseEstimators BipedalLocomotion::ContactDetectors BipedalLocomotion::ParametersHandler BipedalLocomotion::ManifConversions Eigen3::Eigen MANIF::manif gtsam matioCpp::matioCpp iDynTree::idyntree-core iDynTree::idyntree-modelio-urdf BipedalLocomotion::System)

add_kindyn_vio_test(
 NAME EKFOdometryiSAM2
 SOURCES EKFOdometryiSAM2.cpp
 LINKS BipedalLocomotion::FloatingBaseEstimators BipedalLocomotion::ContactDetectors BipedalLocomotion::ParametersHandler BipedalLocomotion::ManifConversions Eigen3::Eigen MANIF::manif gtsam matioCpp::matioCpp iDynTree::idyntree-core iDynTree::idyntree-modelio-urdf BipedalLocomotion::System)
