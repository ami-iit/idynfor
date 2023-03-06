/*
 * SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <iDynFor/iDynTreePinocchioConversions.h>

#include <iDynTree/Core/EigenHelpers.h>

namespace iDynFor
{

pinocchio::SE3 toPinocchio(const iDynTree::Transform& transform_idyntree)
{
    pinocchio::SE3 SE3_pinocchio;
    SE3_pinocchio.rotation_impl(iDynTree::toEigen(transform_idyntree.getRotation()));
    SE3_pinocchio.translation_impl(iDynTree::toEigen(transform_idyntree.getPosition()));
    return SE3_pinocchio;
}

iDynTree::Transform fromPinocchio(const pinocchio::SE3& SE3_pinocchio)
{
    iDynTree::Transform transform_idyntree;
    iDynTree::Rotation rot_idyntree;
    iDynTree::Position pos_idyntree;

    iDynTree::toEigen(rot_idyntree) = SE3_pinocchio.rotation_impl();
    iDynTree::toEigen(pos_idyntree) = SE3_pinocchio.translation_impl();

    transform_idyntree.setRotation(rot_idyntree);
    transform_idyntree.setPosition(pos_idyntree);

    return transform_idyntree;
}

pinocchio::Inertia toPinocchio(const iDynTree::SpatialInertia& inertiaIDynTree)
{
    const double mass = inertiaIDynTree.getMass();
    const pinocchio::Inertia::Vector3 com = iDynTree::toEigen(inertiaIDynTree.getCenterOfMass());
    const pinocchio::Inertia::Matrix3 I
        = iDynTree::toEigen(inertiaIDynTree.getRotationalInertiaWrtCenterOfMass());

    return pinocchio::Inertia(mass, com, I);
}

} // namespace iDynFor
