// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

// Catch2
#include <catch2/catch_test_macros.hpp>
#include <iDynFor/iDynTreePinocchioConversions.h>
#include <pinocchio/algorithm/model.hpp>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Model/ModelTestUtils.h>

#include "LocalModelTestUtils.h"

// The serialization used by iDynTree and the one used by Pinocchio are different:
// iDynTree: m mc_x mc_y mc_z I_xx I_xy I_xz I_yy I_yz I_zz
// pinocchio: m mc_x mc_y mc_z I_xx I_xy I_yy I_xz I_yz I_zz
// i.e. I_xz and I_yy are exchanged
// See docs of pinocchio::InertiaTmpl::toDynamicParameters
// and iDynTree::SpatialInertia::asVector for more details
inline Eigen::Matrix<double, 10, 1>
linkDynamicParametersFromPinocchioToiDynTreeSerialization(const Eigen::Matrix<double, 10, 1>& in)
{
    Eigen::Matrix<double, 10, 1> out = in;
    out[6] = in[7];
    out[7] = in[6];
    return out;
}

TEST_CASE("toPinocchio::iDynTree::SpatialInertia")
{
    // Seed the random generator used by iDynTree
    srand(0);

    for (size_t i = 0; i < 100; i++)
    {
        iDynTree::SpatialInertia idyn_inertia = iDynTree::getRandomInertia();
        pinocchio::Inertia pin_inertia = iDynFor::toPinocchio(idyn_inertia);

        // Verify that converted to a vector of inertial parameters the result is the same
        Eigen::Matrix<double, 10, 1> idyn_inertia_param
            = iDynTree::toEigen(idyn_inertia.asVector());
        Eigen::Matrix<double, 10, 1> pin_inertia_param = pin_inertia.toDynamicParameters();
        Eigen::Matrix<double, 10, 1> pin_inertia_param_idyn_serialization
            = linkDynamicParametersFromPinocchioToiDynTreeSerialization(pin_inertia_param);
        REQUIRE(idyn_inertia_param.isApprox(pin_inertia_param_idyn_serialization));
    }
}

std::string fromPinocchioFrameTypeToString(const pinocchio::FrameType& in)
{
    switch (in)
    {
    case pinocchio::FrameType::OP_FRAME:
        return "OP_FRAME";
    case pinocchio::FrameType::JOINT:
        return "JOINT";
    case pinocchio::FrameType::FIXED_JOINT:
        return "FIXED_JOINT";
    case pinocchio::FrameType::BODY:
        return "BODY";
    case pinocchio::FrameType::SENSOR:
        return "SENSOR";
    default:
        return "UNKNOWN";
    }
}

void printPinocchioFrameInfo(const pinocchio::Frame& frame)
{
    std::cerr << "frame.name: " << frame.name << std::endl;
    std::cerr << "frame.parent(joint): " << frame.parent << std::endl;
    std::cerr << "frame.previousFrame: " << frame.previousFrame << std::endl;
    std::cerr << "frame.type: " << fromPinocchioFrameTypeToString(frame.type) << std::endl;
}

size_t countFramesOfType(const pinocchio::Model& model, const pinocchio::FrameType reqType)
{
    size_t ret = 0;
    for (auto& frame : model.frames)
    {
        if (frame.type == reqType)
        {
            ret++;
        }
    }
    return ret;
}

void printPinocchioModelInfo(const pinocchio::Model& model)
{
    std::cerr << "printPinocchioModelInfo" << std::endl;
    std::cerr << model << std::endl;
    std::cerr << "model.nbodies: " << model.nbodies << std::endl;
    std::cerr << "model.njoints: " << model.njoints << std::endl;
    std::cerr << "model.nframes: " << model.nframes << std::endl;
    for (auto& frame : model.frames)
    {
        printPinocchioFrameInfo(frame);
    }
    for (auto& inertia : model.inertias)
    {
        std::cerr << "inertia: " << inertia << std::endl;
    }

    std::cerr << "total frame of type BODY: "
              << countFramesOfType(model, pinocchio::FrameType::BODY) << std::endl;
    std::cerr << "total frame of type JOINT: "
              << countFramesOfType(model, pinocchio::FrameType::JOINT) << std::endl;
    std::cerr << "total frame of type FIXED_JOINT: "
              << countFramesOfType(model, pinocchio::FrameType::FIXED_JOINT) << std::endl;
}

TEST_CASE("buildPinocchioModelfromiDynTree")
{
    // Seed the random generator used by iDynTree
    srand(0);

    for (size_t i = 0; i < 10; i++)
    {
        iDynTree::Model idynmodel = iDynFor_getRandomModel(10, 10);
        pinocchio::Model pinmodel;
        bool verbose = true;
        bool ok = iDynFor::buildPinocchioModelfromiDynTree(idynmodel, pinmodel, verbose);
        REQUIRE(ok);

        // Uncomment for debug
        /*
        std::cerr << "iDynTree: " << std::endl;
        std::cerr << idynmodel.toString() << std::endl;
        std::cerr << "pinocchio: " << std::endl;
        printPinocchioModelInfo(pinmodel);
        REQUIRE(idynmodel.getNrOfLinks() == countFramesOfType(pinmodel,
        pinocchio::FrameType::BODY)); REQUIRE(idynmodel.getNrOfJoints() + 2 ==
                countFramesOfType(pinmodel, pinocchio::FrameType::JOINT) +
                countFramesOfType(pinmodel, pinocchio::FrameType::FIXED_JOINT));*/

        // See doc/theory_background.md for the motivation
        // In a nutshell, pinocchio consider also the "universe" as
        // a link and the joint connecting the "universe" and the base as a joint
        // For joints, two additional joints are considered in pinocchio

        // iDynTree's getNrOfPosCoords() consider only internal coordinates
        // so we need to add 7 (3 linear position + 4 quaternion) to match pinocchio
        REQUIRE(idynmodel.getNrOfPosCoords() + 7 == pinmodel.nq);
        // iDynTree's getNrOfDOFs() consider only internal coordinates
        // so we need to add 6 (3 linear velocity + 3 angular velocity) to match pinocchio
        REQUIRE(idynmodel.getNrOfDOFs() + 6 == pinmodel.nv);

        REQUIRE(pinmodel.existBodyName(idynmodel.getLinkName(idynmodel.getDefaultBaseLink())));

        // Verify that the inertia of the models match
        for (iDynTree::LinkIndex lnkIdxiDynTree = 0; lnkIdxiDynTree < idynmodel.getNrOfLinks();
             lnkIdxiDynTree++)
        {
            // In iDynTree, inertia is indexed w.r.t. to LinkIndex, in pinocchio with rispect to the
            // parent joint of the link with that inertia in the pinmodel.inertias attribute.
            //
            // However, the inertia stored by pinocchio in pinmodel.inertias are not the inertia of
            // the link, but rather the inertia combined considering all the links that are
            // connected to that joint via fixed joint. For this reason, no we only keep a TODO
            // here.
            // TODO check that inertia are consistent
        }
    }
}

TEST_CASE("toAndFromPinocchio::iDynTree::Transform")
{
    // Seed the random generator used by iDynTree
    srand(0);

    for (size_t i = 0; i < 100; i++)
    {
        iDynTree::Transform idyn_transform = iDynTree::getRandomTransform();
        pinocchio::SE3 pin_transform = iDynFor::toPinocchio(idyn_transform);
        iDynTree::Transform idyn_transform_via_pin = iDynFor::fromPinocchio(pin_transform);

        Eigen::Matrix4d eigen_transform
            = iDynTree::toEigen(idyn_transform.asHomogeneousTransform());
        Eigen::Matrix4d eigen_transform_check
            = iDynTree::toEigen(idyn_transform_via_pin.asHomogeneousTransform());
        // std::cerr << "eigen_transform: " << eigen_transform << std::endl;
        // std::cerr << "eigen_transform_check: " << eigen_transform_check << std::endl;

        REQUIRE(eigen_transform.isApprox(eigen_transform_check));
    }
}
