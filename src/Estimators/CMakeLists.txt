# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(H_PREFIX include/KinDynVIO/Estimators)
add_kindyn_vio_library(
NAME                   Estimators
SOURCES                src/KinDynVIO.cpp
PUBLIC_HEADERS         ${H_PREFIX}/KinDynVIO.h
SUBDIRECTORIES         tests
PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler BipedalLocomotion::TextLogging BipedalLocomotion::FloatingBaseEstimators gtsam
INSTALLATION_FOLDER    Estimators)

