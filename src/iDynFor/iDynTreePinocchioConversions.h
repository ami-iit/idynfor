/*
 * SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia
 * SPDX-License-Identifier: BSD-3-Clause
 */

// pinocchio
#include <pinocchio/spatial/inertia.hpp>

// iDynTree 
#include <iDynTree/Core/SpatialInertia.h>

namespace iDynFor
{

pinocchio::Inertia toPinocchio(const iDynTree::SpatialInertia& inertiaIDynTree);

}
