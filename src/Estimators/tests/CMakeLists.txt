# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_kindyn_vio_test(
 NAME IMUPreintegration
 SOURCES IMUPreintegrationTest.cpp
 LINKS  KinDynVIO::Estimators BipedalLocomotion::Math)

add_kindyn_vio_test(
 NAME ArucoWrapper
 SOURCES ArucoWrapperTest.cpp
 LINKS  KinDynVIO::TestData KinDynVIO::Estimators BipedalLocomotion::ParametersHandler ${OpenCV_LIBS})

