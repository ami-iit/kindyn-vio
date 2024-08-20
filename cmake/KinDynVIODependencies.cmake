# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

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

find_package(matioCpp REQUIRED)

########################## Optional dependencies ##############################

find_package(Catch2 QUIET)
checkandset_dependency(Catch2)

##########################      Components       ##############################
framework_dependent_option(FRAMEWORK_COMPILE_tests
  "Compile tests?" ON
  "FRAMEWORK_USE_Catch2;BUILD_TESTING" OFF)

