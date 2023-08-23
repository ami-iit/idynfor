# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

include(iDynForFindDependencies)

#---------------------------------------------
## Required Dependencies
find_package(pinocchio REQUIRED)
find_package(iDynTree REQUIRED)

#---------------------------------------------
## Optional Dependencies
find_package(Catch2 3 QUIET)
# We do not use checkandset_dependency as the logic is different, as we always
# want for tests to run if the user enables ENABLE_TESTING
option(IDYNFOR_USE_SYSTEM_Catch2 "Use package Catch2 provided by the system" ${Catch2_FOUND})

find_package(VALGRIND QUIET)
checkandset_dependency(VALGRIND)

idynfor_dependent_option(IDYNFOR_RUN_Valgrind_tests
  "Run Valgrind tests?" OFF
  "BUILD_TESTING;VALGRIND_FOUND" OFF)

if(NOT IDYNFOR_USE_SYSTEM_Catch2 AND BUILD_TESTING AND NOT TARGET Catch2::Catch2WithMain)
  # If Catch2 >= 3 is not provided in the system but tests are compiled,
  # we get Catch2 via FetchContent
  Include(FetchContent)

  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.0.1 # or a later release
  )

  message(STATUS "Fetching Catch2.")
  FetchContent_MakeAvailable(Catch2)
endif()
