/*
 * SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Catch2
#include <catch2/catch_test_macros.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <iDynFor/iDynTreePinocchioConversions.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Model/ModelTestUtils.h>

// The serialization used by iDynTree and the one used by Pinocchio are different:
// iDynTree: m mc_x mc_y mc_z I_xx I_xy I_xz I_yy I_yz I_zz
// pinocchio: m mc_x mc_y mc_z I_xx I_xy I_yy I_xz I_yz I_zz
// i.e. I_xz and I_yy are exchanged
// See docs of pinocchio::InertiaTmpl::toDynamicParameters
// and iDynTree::SpatialInertia::asVector for more details
inline Eigen::Matrix<double,10,1> linkDynamicParametersFromPinocchioToiDynTreeSerialization(const Eigen::Matrix<double,10,1>& in)
{
    Eigen::Matrix<double,10,1> out = in;
    out[6] = in[7];
    out[7] = in[6];
    return out;
}

TEST_CASE("toPinocchio::iDynTree::SpatialInertia")
{
    // Seed the random generator used by iDynTree
    srand(0);

    for(size_t i=0; i < 100; i++) {
        iDynTree::SpatialInertia idyn_inertia = iDynTree::getRandomInertia();
        pinocchio::Inertia pin_inertia = iDynFor::toPinocchio(idyn_inertia);

        // Verify that converted to a vector of inertial parameters the result is the same
        Eigen::Matrix<double,10,1> idyn_inertia_param = iDynTree::toEigen(idyn_inertia.asVector());
        Eigen::Matrix<double,10,1> pin_inertia_param = pin_inertia.toDynamicParameters();
        Eigen::Matrix<double,10,1> pin_inertia_param_idyn_serialization =
            linkDynamicParametersFromPinocchioToiDynTreeSerialization(pin_inertia_param);
        REQUIRE(idyn_inertia_param.isApprox(pin_inertia_param_idyn_serialization));
    }
}


TEST_CASE("buildPinocchioModelfromiDynTree")
{
    // Seed the random generator used by iDynTree
    srand(0);

    for(size_t i=0; i < 10; i++) {
        // For now just support 0-joint models
        iDynTree::Model idynmodel = iDynTree::getRandomModel(0);
        pinocchio::Model pinmodel;
        bool verbose = true;
        iDynFor::buildPinocchioModelfromiDynTree(idynmodel, pinmodel, verbose);

        REQUIRE(pinmodel.nbodies == idynmodel.getNrOfLinks());
        REQUIRE(pinmodel.existBodyName(idynmodel.getLinkName(idynmodel.getDefaultBaseLink())));

        // Verify that the inertia of the models match
        iDynTree::SpatialInertia idyn_inertia = idynmodel.getLink(idynmodel.getDefaultBaseLink())->getInertia();
        pinocchio::Inertia pin_inertia = pinmodel.inertias[0];

        Eigen::Matrix<double,10,1> idyn_inertia_param = iDynTree::toEigen(idyn_inertia.asVector());
        Eigen::Matrix<double,10,1> pin_inertia_param = pin_inertia.toDynamicParameters();
        Eigen::Matrix<double,10,1> pin_inertia_param_idyn_serialization =
            linkDynamicParametersFromPinocchioToiDynTreeSerialization(pin_inertia_param);
        REQUIRE(idyn_inertia_param.isApprox(pin_inertia_param_idyn_serialization));
    }

}

