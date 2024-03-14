// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

// Catch2
#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>
#include <iDynFor/iDynTreeFullyCompatibleKinDynComputations.h>
#include <iDynFor/KinDynComputations.h>
#include <iDynFor/iDynTreePinocchioConversions.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/KinDynComputations.h>

#include "pinocchio/algorithm/joint-configuration.hpp"

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Model/ModelTestUtils.h>

#include "LocalModelTestUtils.h"

#include <iCubModels/iCubModels.h>


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

bool setRandomState(pinocchio::ModelTpl<double, 0, pinocchio::JointCollectionDefaultTpl>& pinocchioModel, pinocchio::ModelTpl<double, 0, pinocchio::JointCollectionDefaultTpl>::VectorXs pinocchioModelPosition)
{
    pinocchioModelPosition = iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl>::VectorXs::Random(pinocchioModel.nq, 1);
    // This elements are a unit quaternion coefficient, so we can't just assign them randomly
    pinocchioModelPosition(3) = 0.0;
    pinocchioModelPosition(4) = 0.0;
    pinocchioModelPosition(5) = 0.0;
    pinocchioModelPosition(6) = 1.0;
    return true;
}


TEST_CASE("KinDynComputations Benchmarks")
{
    // Seed the random generator used by iDynTree
    srand(0);

    // Useful for debug
    //size_t nrOfLinks = 10;
    //size_t nrOfAdditionalFrames = 10;
    //iDynTree::Model idynmodel = iDynFor_getRandomModel(nrOfLinks, nrOfAdditionalFrames);

    // Load an iCub model to have a realistic humanoid example
    iDynTree::ModelLoader mdlLoader;
    bool okLoad = mdlLoader.loadModelFromFile(iCubModels::getModelFile("iCubGazeboV2_7"));
    REQUIRE(okLoad);
    std::cerr << "Running benchmark using model " << iCubModels::getModelFile("iCubGazeboV2_7") << std::endl;
    iDynTree::Model idynmodel = mdlLoader.model();

    // Create :
    // * iDynTree::KinDynComputations
    // * iDynFor::iDynTreeFullyCompatible::KinDynComputations
    // * iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl>
    // * Pure pinocchio model data structures
    // to compare performances consistency
    iDynFor::iDynTreeFullyCompatible::KinDynComputations kinDynFor;
    iDynTree::KinDynComputations kinDynTree;
    iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl> kinDynForEigen;
    pinocchio::ModelTpl<double, 0, pinocchio::JointCollectionDefaultTpl> pinocchioModel;

    // Before calling loadRobotModel, isValid should return false
    REQUIRE(!kinDynTree.isValid());
    REQUIRE(!kinDynFor.isValid());
    REQUIRE(!kinDynForEigen.isValid());

    REQUIRE(kinDynTree.loadRobotModel(idynmodel));
    okLoad = kinDynFor.loadRobotModel(idynmodel);
    REQUIRE(okLoad);
    okLoad = kinDynForEigen.loadRobotModel(idynmodel);
    REQUIRE(okLoad);

    bool verbose = true;
    okLoad = iDynFor::buildPinocchioModelfromiDynTree<double, 0, pinocchio::JointCollectionDefaultTpl>(kinDynFor.model(),
                                                                                  pinocchioModel,
                                                                                  verbose);
    REQUIRE(okLoad);
    pinocchio::DataTpl<double, 0, pinocchio::JointCollectionDefaultTpl> pinocchioData(pinocchioModel);
    pinocchio::ModelTpl<double, 0, pinocchio::JointCollectionDefaultTpl>::VectorXs pinocchioModelPosition;
    pinocchioModelPosition.resize(pinocchioModel.nq);
    setRandomState(pinocchioModel, pinocchioModelPosition);
    pinocchio::crba(pinocchioModel, pinocchioData, pinocchioModelPosition);


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
        setRandomState(kinDynForEigen);
        iDynTree::FrameIndex randomFrameIndex = rand() % kinDynForEigen.model().getNrOfFrames();
        iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl>::Matrix6Xs jacobian(6, 6+kinDynForEigen.model().getNrOfDOFs());
        meter.measure([&] { return kinDynForEigen.getFrameFreeFloatingJacobian(
            randomFrameIndex, jacobian); });
    };

    BENCHMARK_ADVANCED("getFreeFloatingMassMatrix\niDynTree")(Catch::Benchmark::Chronometer meter) {
        setRandomState(kinDynTree);
        Eigen::MatrixXd massMatrix(6+kinDynTree.model().getNrOfDOFs(), 6+kinDynTree.model().getNrOfDOFs());
        meter.measure([&] { return kinDynTree.getFreeFloatingMassMatrix(iDynTree::make_matrix_view(massMatrix)); });
    };

    BENCHMARK_ADVANCED("getFreeFloatingMassMatrix\niDynFor (iDynTree interface)")(Catch::Benchmark::Chronometer meter) {
        setRandomState(kinDynFor);
        Eigen::MatrixXd massMatrix(6+kinDynFor.model().getNrOfDOFs(), 6+kinDynFor.model().getNrOfDOFs());
        meter.measure([&] { return kinDynFor.getFreeFloatingMassMatrix(iDynTree::make_matrix_view(massMatrix)); });
    };

    BENCHMARK_ADVANCED("getFreeFloatingMassMatrix\niDynFor (Eigen interface)")(Catch::Benchmark::Chronometer meter) {
        setRandomState(kinDynForEigen);
        Eigen::MatrixXd massMatrix(6+kinDynForEigen.model().getNrOfDOFs(), 6+kinDynForEigen.model().getNrOfDOFs());
        meter.measure([&] { return kinDynForEigen.getFreeFloatingMassMatrix(massMatrix); });
    };

    BENCHMARK_ADVANCED("getFreeFloatingMassMatrix\npure pinocchio equivalent")(Catch::Benchmark::Chronometer meter) {
        setRandomState(pinocchioModel, pinocchioModelPosition);
        Eigen::MatrixXd massMatrix(pinocchioModel.nv, pinocchioModel.nv);
        meter.measure([&] { return pinocchio::crba(pinocchioModel, pinocchioData, pinocchioModelPosition); });
    };

}