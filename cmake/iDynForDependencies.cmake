# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

include(iDynForFindDependencies)

#---------------------------------------------
## Required Dependencies
find_package(pinocchio REQUIRED)

#---------------------------------------------
## Optional Dependencies
find_package(Catch2 QUIET)
checkandset_dependency(Catch2)

find_package(VALGRIND QUIET)
checkandset_dependency(VALGRIND)

idynfor_dependent_option(IDYNFOR_COMPILE_tests
  "Compile tests?" ON
  "IDYNFOR_USE_Catch2;BUILD_TESTING" OFF)
