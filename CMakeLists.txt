# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.5)

# Load version from pixi.toml
file(READ pixi.toml PIXI_TOML)
# Format needs to be 1 to 4 numbers separated by dots
set(PIXI_VERSION_REGEX "version = \"(([0-9]+)(\\.[0-9]+)?(\\.[0-9]+)?(\\.[0-9]+)?)\"")
string(REGEX MATCH "${PIXI_VERSION_REGEX}" PIXI_REGEX_OUTPUT ${PIXI_TOML})
set(PIXI_VERSION_STRING ${CMAKE_MATCH_1})
if(PIXI_VERSION_STRING STREQUAL "")
  message(FATAL_ERROR "Impossible to read version from pixi.toml")
endif()

project(iDynFor
        LANGUAGES CXX C
        VERSION ${PIXI_VERSION_STRING})

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

### Options
option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)
option(BUILD_TESTING "Create tests using CMake" OFF)
if(BUILD_TESTING)
    enable_testing()
endif()

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, recommanded options are: Debug or Release" FORCE)
    endif()
    set(IDYNFOR_BUILD_TYPES "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${IDYNFOR_BUILD_TYPES})
endif()


include(iDynForDependencies)

add_subdirectory(src)

include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME}
                            VERSION ${${PROJECT_NAME}_VERSION}
                            COMPATIBILITY AnyNewerVersion
                            NO_EXPORT   # NO_EXPORT is required since we have only cmake scripts. To be changed in EXPORT ${${PROJECT_NAME}_TARGETS}
                            VARS_PREFIX ${PROJECT_NAME}
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO
                            DEPENDENCIES iDynTree pinocchio)
include(AddUninstallTarget)

# Add integration tests (unit tests for each library should be in each sublibrary directory).
if(BUILD_TESTING)
  include(AddiDynForUnitTest)
  add_subdirectory(test)
endif()