// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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

            iDynTree::Transform world_H_base_copy;
            iDynTree::Twist base_vel_copy;
            iDynTree::Vector3 gravity_copy;
            iDynTree::VectorDynSize joint_pos_copy, joint_vel_copy;
            kinDynFor.getRobotState(world_H_base_copy,
                                    joint_pos_copy,
                                    base_vel_copy,
                                    joint_vel_copy,
                                    gravity_copy);

            REQUIRE(iDynTree::toEigen(world_H_base_copy.asHomogeneousTransform())
                        .isApprox(iDynTree::toEigen(world_H_base.asHomogeneousTransform())));
            REQUIRE(iDynTree::toEigen(joint_pos_copy).isApprox(iDynTree::toEigen(joint_pos)));
            REQUIRE(iDynTree::toEigen(base_vel_copy).isApprox(iDynTree::toEigen(base_vel)));
            REQUIRE(iDynTree::toEigen(joint_vel_copy).isApprox(iDynTree::toEigen(joint_vel)));
            REQUIRE(iDynTree::toEigen(gravity_copy).isApprox(iDynTree::toEigen(gravity)));

            // Set random acceleration
            iDynTree::Vector6 base_acc;
            iDynTree::getRandomVector(base_acc);
            iDynTree::VectorDynSize joint_acc;
            joint_acc.resize(idynmodel.getNrOfDOFs());
            iDynTree::getRandomVector(joint_acc);

            // Uncomment once https://github.com/robotology/idyntree/issues/1092 is fixed
            //REQUIRE(
            //    kinDynTree.setRobotAcceleration(base_acc, joint_acc));
            REQUIRE(kinDynFor.setRobotAcceleration(base_acc, joint_acc));


            iDynTree::Vector6 base_acc_copy;
            iDynTree::VectorDynSize joint_acc_copy;
            kinDynFor.getRobotAcceleration(base_acc_copy,
                                           joint_acc_copy);
            REQUIRE(iDynTree::toEigen(base_acc_copy).isApprox(iDynTree::toEigen(base_acc)));
            REQUIRE(iDynTree::toEigen(joint_acc_copy).isApprox(iDynTree::toEigen(joint_acc)));

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


                // Test getFrameAcc
                Eigen::Matrix<double, 6, 1> frameAccFor
                    = iDynTree::toEigen(kinDynFor.getFrameAcc(randomFrameIndex, base_acc, joint_acc));
                Eigen::Matrix<double, 6, 1> frameAccTree
                    = iDynTree::toEigen(kinDynTree.getFrameAcc(randomFrameIndex, base_acc, joint_acc));
                // Uncomment for debug
                //std::cerr << "Frame acceleraton of " <<
                //kinDynFor.model().getFrameName(randomFrameIndex) << std::endl; std::cerr <<
                //frameAccFor << std::endl; std::cerr << frameAccTree << std::endl;
                REQUIRE(frameAccFor.isApprox(frameAccTree));

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
