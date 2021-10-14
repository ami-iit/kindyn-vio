# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(H_PREFIX include/KinDynVIO/Factors)
add_kindyn_vio_library(
NAME                   Factors
SOURCES                src/CentroidalDynamicsMeasurementBias.cpp src/PreintegratedCentroidalDynamicsMeasurements.cpp src/CentroidalDynamicsFactor.cpp
PUBLIC_HEADERS         ${H_PREFIX}/CentroidalDynamicsMeasurementBias.h ${H_PREFIX}/PreintegratedCentroidalDynamicsMeasurements.h ${H_PREFIX}/NoiseModelFactor10.h ${H_PREFIX}/CentroidalDynamicsFactor.h
SUBDIRECTORIES         tests
PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler BipedalLocomotion::TextLogging BipedalLocomotion::Math gtsam gtsam_unstable metis-gtsam iDynTree::idyntree-core iDynTree::idyntree-model iDynTree::idyntree-high-level
INSTALLATION_FOLDER    Factors)

