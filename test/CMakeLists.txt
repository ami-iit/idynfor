# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia
# SPDX-License-Identifier: BSD-3-Clause

add_idynfor_test(NAME iDynTreePinocchioConversionsTest
                 SOURCES iDynTreePinocchioConversionsTest.cpp
                 LINKS iDynFor::iDynFor)

add_idynfor_test(NAME KinDynComputationsTest
                 SOURCES KinDynComputationsTest.cpp
                 LINKS iDynFor::iDynFor iDynTree::idyntree-high-level)
