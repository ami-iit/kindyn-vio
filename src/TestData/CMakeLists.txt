# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ResourceFolderPath.h.in" "${CMAKE_CURRENT_BINARY_DIR}/ResourceFolderPath.h" @ONLY)
add_library(TestData INTERFACE)
# add the binary tree to the search path for include files
# so that we will find ResourceFolderPath.h
target_include_directories(TestData INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>")

add_library(KinDynVIO::TestData ALIAS TestData)



