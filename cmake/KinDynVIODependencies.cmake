# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

################################################################################

include(KinDynVIOFindDependencies)


################################################################################
########################## Mandatory dependencies ##############################

find_package(iDynTree 3.0.0 REQUIRED) 

find_package(Eigen3 3.2.92 REQUIRED)

find_package(manif REQUIRED)

find_package(BipedalLocomotionFramework REQUIRED)

find_package(spdlog REQUIRED)

find_package(OpenCV 4.0.0 REQUIRED)

find_package(GTSAM REQUIRED)

########################## Optional dependencies ##############################

find_package(Catch2 QUIET)
checkandset_dependency(Catch2)

##########################      Components       ##############################
framework_dependent_option(FRAMEWORK_COMPILE_tests
  "Compile tests?" ON
  "FRAMEWORK_USE_Catch2;BUILD_TESTING" OFF)

