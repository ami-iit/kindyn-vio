# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(H_PREFIX include/KinDynVIO/Estimators)
add_kindyn_vio_library(
NAME                   Estimators
SOURCES                src/States.cpp src/IMUPreintegrator.cpp src/ArucoWrapper.cpp src/GraphManager.cpp
PUBLIC_HEADERS         ${H_PREFIX}/States.h ${H_PREFIX}/IMUPreintegrator.h ${H_PREFIX}/ArucoWrapper.h ${H_PREFIX}/IO.h ${H_PREFIX}/KinematicInertialFilterWrapper.h ${H_PREFIX}/GraphManager.h
SUBDIRECTORIES         tests
PUBLIC_LINK_LIBRARIES  KinDynVIO::PerceptionFeatures BipedalLocomotion::ParametersHandler BipedalLocomotion::TextLogging BipedalLocomotion::Math BipedalLocomotion::System BipedalLocomotion::PerceptionFeatures BipedalLocomotion::FloatingBaseEstimators BipedalLocomotion::ContactDetectors gtsam gtsam_unstable metis-gtsam iDynTree::idyntree-core Eigen3::Eigen
INSTALLATION_FOLDER    Estimators)

