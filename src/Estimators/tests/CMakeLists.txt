# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

add_kindyn_vio_test(
 NAME IMUPreintegration
 SOURCES IMUPreintegrationTest.cpp
 LINKS  KinDynVIO::Estimators BipedalLocomotion::Math)

add_kindyn_vio_test(
 NAME ArucoWrapper
 SOURCES ArucoWrapperTest.cpp
 LINKS  KinDynVIO::TestData KinDynVIO::Estimators BipedalLocomotion::ParametersHandler ${OpenCV_LIBS})

