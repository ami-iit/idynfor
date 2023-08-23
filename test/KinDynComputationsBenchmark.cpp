// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

// Catch2
#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>
#include <iDynFor/iDynTreeFullyCompatibleKinDynComputations.h>
#include <iDynFor/KinDynComputations.h>
#include <iDynFor/iDynTreePinocchioConversions.h>

#include <iDynTree/KinDynComputations.h>


#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Model/ModelTestUtils.h>

#include "LocalModelTestUtils.h"


bool setRandomState(iDynFor::iDynTreeFullyCompatible::KinDynComputations& kinDynFor)
{
    // Set random state
    iDynTree::Transform world_H_base = iDynTree::getRandomTransform();
    iDynTree::Twist base_vel = iDynTree::getRandomTwist();
    iDynTree::Vector3 gravity;
    iDynTree::getRandomVector(gravity);
    iDynTree::VectorDynSize joint_pos, joint_vel;
    joint_pos.resize(kinDynFor.model().getNrOfPosCoords());
    joint_vel.resize(kinDynFor.model().getNrOfDOFs());
    iDynTree::getRandomVector(joint_pos);
    iDynTree::getRandomVector(joint_vel);
    return kinDynFor.setRobotState(world_H_base, joint_pos, base_vel, joint_vel, gravity);
}

bool setRandomState(iDynTree::KinDynComputations& kinDynTree)
{
    // Set random state
    iDynTree::Transform world_H_base = iDynTree::getRandomTransform();
    iDynTree::Twist base_vel = iDynTree::getRandomTwist();
    iDynTree::Vector3 gravity;
    iDynTree::getRandomVector(gravity);
    iDynTree::VectorDynSize joint_pos, joint_vel;
    joint_pos.resize(kinDynTree.model().getNrOfPosCoords());
    joint_vel.resize(kinDynTree.model().getNrOfDOFs());
    iDynTree::getRandomVector(joint_pos);
    iDynTree::getRandomVector(joint_vel);
    return kinDynTree.setRobotState(world_H_base, joint_pos, base_vel, joint_vel, gravity);
}

bool setRandomState(iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl>& kinDynForEigen)
{
    size_t nrOfInternalDOFs = kinDynForEigen.model().getNrOfDOFs();
    iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl>::SE3s world_H_base
        = iDynFor::toPinocchio(iDynTree::getRandomTransform());
    iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl>::Vector6s base_vel =
        iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl>::Vector6s::Random();
    iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl>::Vector3s gravity =
            iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl>::Vector3s::Random();
    iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl>::VectorXs joint_pos
        = iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl>::VectorXs::Random(nrOfInternalDOFs, 1);
    iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl>::VectorXs joint_vel
        = iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl>::VectorXs::Random(nrOfInternalDOFs, 1);
    return kinDynForEigen.setRobotState(world_H_base, joint_pos, base_vel, joint_vel, gravity);
}


TEST_CASE("KinDynComputations Benchmarks")
{
    // Seed the random generator used by iDynTree
    srand(0);

    size_t nrOfLinks = 25;
    size_t nrOfAdditionalFrames = 10;
    iDynTree::Model idynmodel = iDynFor_getRandomModel(nrOfLinks, nrOfAdditionalFrames);

    // Create both a new and and old KinDynComputations to check consistency
    iDynFor::iDynTreeFullyCompatible::KinDynComputations kinDynFor;
    iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl> kinDynForEigen;
    iDynTree::KinDynComputations kinDynTree;


    // Before calling loadRobotModel, isValid should return false
    REQUIRE(!kinDynTree.isValid());
    REQUIRE(!kinDynFor.isValid());
    REQUIRE(!kinDynForEigen.isValid());

    REQUIRE(kinDynTree.loadRobotModel(idynmodel));
    bool okLoad = kinDynFor.loadRobotModel(idynmodel);
    REQUIRE(okLoad);
    okLoad = kinDynForEigen.loadRobotModel(idynmodel);
    REQUIRE(okLoad);


    // Note: this benchmarks always re-set the state and benchmark a given method, and are a bit
    // "artificial" in the sense that in typical workload, setRobotState is only called once,
    // and many methods are called after, tipically reusing cache
    // Anyhow, it is a good indicator for how fast computing the quantities related to each method is
    BENCHMARK_ADVANCED("getWorldTransform\niDynTree")(Catch::Benchmark::Chronometer meter) {
        setRandomState(kinDynTree);
        iDynTree::FrameIndex randomFrameIndex = rand() % kinDynTree.model().getNrOfFrames();

        meter.measure([&] {
            auto ret =  kinDynTree.getWorldTransform(randomFrameIndex);
        });
    };

    BENCHMARK_ADVANCED("getWorldTransform\niDynFor (iDynTree interface)")(Catch::Benchmark::Chronometer meter) {
        setRandomState(kinDynFor);
        iDynTree::FrameIndex randomFrameIndex = rand() % kinDynFor.model().getNrOfFrames();
        meter.measure([&] { return kinDynFor.getWorldTransform(randomFrameIndex); });
    };

    BENCHMARK_ADVANCED("getWorldTransform\niDynFor (Eigen interface)")(Catch::Benchmark::Chronometer meter) {
        setRandomState(kinDynForEigen);
        iDynTree::FrameIndex randomFrameIndex = rand() % kinDynForEigen.model().getNrOfFrames();
        iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl>::SE3s result;
        meter.measure([&] { return kinDynForEigen.getWorldTransform(randomFrameIndex, result); });
    };

    BENCHMARK_ADVANCED("getFrameVel\niDynTree")(Catch::Benchmark::Chronometer meter) {
        setRandomState(kinDynTree);
        iDynTree::FrameIndex randomFrameIndex = rand() % kinDynTree.model().getNrOfFrames();
        Eigen::Matrix<double, 6, 1> ret_twist;
        meter.measure([&] { return kinDynTree.getFrameVel(randomFrameIndex, ret_twist); });
    };

    BENCHMARK_ADVANCED("getFrameVel\niDynFor (iDynTree interface)")(Catch::Benchmark::Chronometer meter) {
        setRandomState(kinDynFor);
        iDynTree::FrameIndex randomFrameIndex = rand() % kinDynFor.model().getNrOfFrames();
        Eigen::Matrix<double, 6, 1> ret_twist;
        meter.measure([&] { return kinDynFor.getFrameVel(randomFrameIndex, ret_twist); });
    };

    BENCHMARK_ADVANCED("getFrameVel\niDynFor (Eigen interface)")(Catch::Benchmark::Chronometer meter) {
        setRandomState(kinDynForEigen);
        iDynTree::FrameIndex randomFrameIndex = rand() % kinDynForEigen.model().getNrOfFrames();
        iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl>::Vector6s res_vel;
        meter.measure([&] { return kinDynForEigen.getFrameVel(randomFrameIndex, res_vel); });
    };

    BENCHMARK_ADVANCED("getFrameFreeFloatingJacobian\niDynTree")(Catch::Benchmark::Chronometer meter) {
        setRandomState(kinDynTree);
        iDynTree::FrameIndex randomFrameIndex = rand() % kinDynTree.model().getNrOfFrames();
        Eigen::MatrixXd jacobian(6, 6+kinDynTree.model().getNrOfDOFs());
        meter.measure([&] { return kinDynTree.getFrameFreeFloatingJacobian(
            randomFrameIndex, iDynTree::make_matrix_view(jacobian)); });
    };

    BENCHMARK_ADVANCED("getFrameFreeFloatingJacobian\niDynFor (iDynTree interface)")(Catch::Benchmark::Chronometer meter) {
        setRandomState(kinDynFor);
        iDynTree::FrameIndex randomFrameIndex = rand() % kinDynFor.model().getNrOfFrames();
        Eigen::MatrixXd jacobian(6, 6+kinDynFor.model().getNrOfDOFs());
        meter.measure([&] { return kinDynFor.getFrameFreeFloatingJacobian(
            randomFrameIndex, iDynTree::make_matrix_view(jacobian)); });
    };

    BENCHMARK_ADVANCED("getFrameFreeFloatingJacobian\niDynFor (Eigen interface)")(Catch::Benchmark::Chronometer meter) {
        setRandomState(kinDynFor);
        iDynTree::FrameIndex randomFrameIndex = rand() % kinDynFor.model().getNrOfFrames();
        Eigen::MatrixXd jacobian(6, 6+kinDynFor.model().getNrOfDOFs());
        meter.measure([&] { return kinDynFor.getFrameFreeFloatingJacobian(
            randomFrameIndex, iDynTree::make_matrix_view(jacobian)); });
    };

}