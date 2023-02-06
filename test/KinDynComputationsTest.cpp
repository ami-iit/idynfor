/*
 * SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Catch2
#include <catch2/catch_test_macros.hpp>
#include <iDynFor/KinDynComputations.h>
#include <pinocchio/algorithm/model.hpp>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Model/ModelTestUtils.h>

TEST_CASE("KinDynComputations")
{
    // Seed the random generator used by iDynTree
    srand(0);

    for (size_t i = 0; i < 10; i++)
    {
        // For now just support 0-joint models
        iDynTree::Model idynmodel = iDynTree::getRandomModel(0);

        iDynFor::KinDynComputations kinDyn;

        // Before calling loadRobotModel, isValid should return false
        REQUIRE(!kinDyn.isValid());

        REQUIRE(kinDyn.loadRobotModel(idynmodel));

        // After calling loadRobotModel, isValid should return true
        REQUIRE(kinDyn.isValid());

        // Simple smoke test that model() is actually returning the same model
        REQUIRE(kinDyn.model().getNrOfLinks() == idynmodel.getNrOfLinks());
    }
}
