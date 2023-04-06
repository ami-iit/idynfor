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

#include "LocalModelTestUtils.h"

TEST_CASE("KinDynComputations")
{
    // Seed the random generator used by iDynTree
    srand(0);

    for (size_t i = 0; i < 100; i++)
    {
        size_t nrOfLinks = 10;
        size_t nrOfAdditionalFrames = 10;
        iDynTree::Model idynmodel = iDynFor_getRandomModel(nrOfLinks, nrOfAdditionalFrames);

        // Create both a new and and old KinDynComputations to check consistency
        iDynFor::iDynTreeFullyCompatible::KinDynComputations kinDynFor;
        iDynTree::KinDynComputations kinDynTree;

        // Before calling loadRobotModel, isValid should return false
        REQUIRE(!kinDynTree.isValid());
        REQUIRE(!kinDynFor.isValid());

        REQUIRE(kinDynTree.loadRobotModel(idynmodel));
        bool okLoad = kinDynFor.loadRobotModel(idynmodel);
        REQUIRE(okLoad);

        // Set random velocity representation
        iDynTree::FrameVelocityRepresentation velRepr = iDynFor_getRandomVelocityRepresentation();
        REQUIRE(kinDynTree.setFrameVelocityRepresentation(velRepr));
        REQUIRE(kinDynFor.setFrameVelocityRepresentation(velRepr));

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

            // Test frame-related methods
            int nrOfFramesToTest = 20;
            for (int fr = 0; fr < nrOfFramesToTest; fr++)
            {
                iDynTree::FrameIndex randomFrameIndex = rand() % idynmodel.getNrOfFrames();
                REQUIRE(kinDynFor.getFrameName(randomFrameIndex)
                        == kinDynTree.getFrameName(randomFrameIndex));
                std::string frameName = kinDynFor.getFrameName(randomFrameIndex);
                REQUIRE(kinDynFor.getFrameIndex(frameName) == randomFrameIndex);
                REQUIRE(kinDynFor.getFrameIndex(frameName) == kinDynTree.getFrameIndex(frameName));

                // Test getWorldTransform
                Eigen::Matrix4d worldTransformFor = iDynTree::toEigen(
                    kinDynFor.getWorldTransform(randomFrameIndex).asHomogeneousTransform());
                Eigen::Matrix4d worldTransformTree = iDynTree::toEigen(
                    kinDynTree.getWorldTransform(randomFrameIndex).asHomogeneousTransform());

                // Uncomment for debug
                // std::cerr << "WorldTransform of " <<
                // kinDynFor.model().getFrameName(randomFrameIndex) << std::endl; std::cerr <<
                // worldTransformFor << std::endl; std::cerr << worldTransformTree << std::endl;
                REQUIRE(worldTransformFor.isApprox(worldTransformTree));

                // Test getFrameVel
                Eigen::Matrix<double, 6, 1> frameVelFor
                    = iDynTree::toEigen(kinDynFor.getFrameVel(randomFrameIndex));
                Eigen::Matrix<double, 6, 1> frameVelTree
                    = iDynTree::toEigen(kinDynTree.getFrameVel(randomFrameIndex));
                REQUIRE(frameVelFor.isApprox(frameVelTree));


                // Test getFrameFreeFloatingJacobian
                Eigen::MatrixXd jacobianFor(6, 6+idynmodel.getNrOfDOFs());
                Eigen::MatrixXd jacobianTree(6, 6+idynmodel.getNrOfDOFs());
                REQUIRE(kinDynFor.getFrameFreeFloatingJacobian(
                    randomFrameIndex, iDynTree::make_matrix_view(jacobianFor)));
                REQUIRE(kinDynTree.getFrameFreeFloatingJacobian(
                    randomFrameIndex, iDynTree::make_matrix_view(jacobianTree)));
                REQUIRE(jacobianFor.isApprox(jacobianTree));


            }
        }
    }
}
