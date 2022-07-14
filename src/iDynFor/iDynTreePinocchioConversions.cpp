/*
 * SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <iDynFor/iDynTreePinocchioConversions.h>

#include <iDynTree/Core/EigenHelpers.h>

namespace iDynFor
{

pinocchio::Inertia toPinocchio(const iDynTree::SpatialInertia& inertiaIDynTree)
{
    const double mass = inertiaIDynTree.getMass();
    const pinocchio::Inertia::Vector3 com = 
        iDynTree::toEigen(inertiaIDynTree.getCenterOfMass());
    const pinocchio::Inertia::Matrix3 I = 
        iDynTree::toEigen(inertiaIDynTree.getRotationalInertiaWrtCenterOfMass());

    return pinocchio::Inertia(mass, com, I);
}

}
