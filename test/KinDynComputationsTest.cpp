/*
 * SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Catch2
#include <catch2/catch_test_macros.hpp>
#include <iDynFor/iDynTreeFullyCompatibleKinDynComputations.h>
#include <iDynTree/KinDynComputations.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Model/ModelTestUtils.h>

TEST_CASE("KinDynComputations")
{
    // Seed the random generator used by iDynTree
    srand(0);

    for (size_t i = 0; i < 10; i++)
    {
        // For now just support 0-joint models with 0 additional frames
        iDynTree::Model idynmodel = iDynTree::getRandomModel(0, 0);

        // Create both a new and and old KinDynComputations to check consistency
        iDynFor::iDynTreeFullyCompatible::KinDynComputations kinDynFor;
        iDynTree::KinDynComputations kinDynTree;

        // Before calling loadRobotModel, isValid should return false
        REQUIRE(!kinDynTree.isValid());
        REQUIRE(!kinDynFor.isValid());

        REQUIRE(kinDynTree.loadRobotModel(idynmodel));
        REQUIRE(kinDynFor.loadRobotModel(idynmodel));

        // After calling loadRobotModel, isValid should return true
        REQUIRE(kinDynTree.isValid());
        REQUIRE(kinDynFor.isValid());

        // Simple smoke test that model() is actually returning the same model
        REQUIRE(kinDynFor.model().getNrOfLinks() == idynmodel.getNrOfLinks());

        int nrOfStatesToTest = 3;
        for (int st = 0; st < nrOfStatesToTest; st++)
        {
            // Set random state
            iDynTree::Transform world_H_base = iDynTree::getRandomTransform();
            iDynTree::Twist base_vel = iDynTree::getRandomTwist();
            iDynTree::Vector3 gravity;
            iDynTree::getRandomVector(gravity);
            iDynTree::VectorDynSize joint_pos, joint_vel;
            joint_pos.resize(idynmodel.getNrOfPosCoords());
            joint_vel.resize(idynmodel.getNrOfDOFs());
            iDynTree::getRandomVector(joint_pos);
            iDynTree::getRandomVector(joint_vel);

            REQUIRE(
                kinDynTree.setRobotState(world_H_base, joint_pos, base_vel, joint_vel, gravity));
            REQUIRE(kinDynFor.setRobotState(world_H_base, joint_pos, base_vel, joint_vel, gravity));

            // TODO : remove check when there are other frames
            REQUIRE(idynmodel.getNrOfFrames() == 1);

            // Test frame-related methods
            int nrOfFramesToTest = 3;
            for (int fr = 0; fr < nrOfFramesToTest; fr++)
            {
                iDynTree::FrameIndex randomFrameIndex = rand() % idynmodel.getNrOfFrames();
                REQUIRE(kinDynFor.getFrameName(randomFrameIndex)
                        == kinDynTree.getFrameName(randomFrameIndex));
                std::string frameName = kinDynFor.getFrameName(randomFrameIndex);
                REQUIRE(kinDynFor.getFrameIndex(frameName) == randomFrameIndex);
                REQUIRE(kinDynFor.getFrameIndex(frameName) == kinDynTree.getFrameIndex(frameName));

                Eigen::Matrix4d worldTransformFor = iDynTree::toEigen(
                    kinDynFor.getWorldTransform(randomFrameIndex).asHomogeneousTransform());
                Eigen::Matrix4d worldTransformTree = iDynTree::toEigen(
                    kinDynTree.getWorldTransform(randomFrameIndex).asHomogeneousTransform());
                std::cerr << "worldTransformFor: " << worldTransformFor << std::endl;
                std::cerr << "worldTransformTree: " << worldTransformTree << std::endl;

                REQUIRE(worldTransformFor.isApprox(worldTransformTree));
            }
        }
    }
}
