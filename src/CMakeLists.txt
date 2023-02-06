# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia
# SPDX-License-Identifier: BSD-3-Clause

set(iDynFor_SRC
        iDynFor/iDynTreePinocchioConversions.cpp
        iDynFor/KinDynComputations.cpp
)

set(iDynFor_HDR
        iDynFor/iDynTreePinocchioConversions.h
        iDynFor/KinDynComputations.h
)

add_library(iDynFor ${iDynFor_SRC} ${iDynFor_HDR})
add_library(iDynFor::iDynFor ALIAS iDynFor)

set_target_properties(iDynFor PROPERTIES VERSION ${${PROJECT_NAME}_VERSION}
                                         PUBLIC_HEADER "${iDynFor_HDR}")

target_include_directories(iDynFor PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                                          "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(iDynFor PUBLIC Eigen3::Eigen pinocchio::pinocchio iDynTree::idyntree-core  iDynTree::idyntree-model)

# Workaround for https://github.com/ami-iit/idynfor/issues/10
if(WIN32)
  target_compile_definitions(iDynFor PUBLIC NOMINMAX)
endif()

# Specify installation targets, typology and destination folders.
install(TARGETS iDynFor
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/iDynFor")
