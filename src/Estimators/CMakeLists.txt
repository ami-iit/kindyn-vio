# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

set(H_PREFIX include/KinDynVIO/Estimators)
add_kindyn_vio_library(
NAME                   Estimators
SOURCES                src/States.cpp src/IMUPreintegrator.cpp src/ArucoWrapper.cpp
PUBLIC_HEADERS         ${H_PREFIX}/States.h ${H_PREFIX}/IMUPreintegrator.h ${H_PREFIX}/ArucoWrapper.h
SUBDIRECTORIES         tests
PUBLIC_LINK_LIBRARIES  KinDynVIO::PerceptionFeatures BipedalLocomotion::ParametersHandler BipedalLocomotion::TextLogging BipedalLocomotion::Math BipedalLocomotion::System BipedalLocomotion::PerceptionFeatures gtsam gtsam_unstable metis-gtsam iDynTree::idyntree-core
INSTALLATION_FOLDER    Estimators)

