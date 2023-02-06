/*
 * SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <iDynFor/KinDynComputations.h>

#include <iDynFor/iDynTreePinocchioConversions.h>

namespace iDynFor
{

struct KinDynComputations::Impl
{
    bool modelLoaded = false;
    iDynTree::Model idyntreeModel;
    pinocchio::Model pinModel;
};

KinDynComputations::KinDynComputations()
{
    m_pimpl = std::make_unique<KinDynComputations::Impl>();
}

KinDynComputations::~KinDynComputations() = default;

bool KinDynComputations::loadRobotModel(const iDynTree::Model & model)
{
    m_pimpl->idyntreeModel = model;
    // TODO: handle verbosity
    bool verbose = true;

    // TODO: understand if we should catch exception and return bool?
    iDynFor::buildPinocchioModelfromiDynTree(m_pimpl->idyntreeModel,
                                                       m_pimpl->pinModel,
                                                       verbose);

    return (m_pimpl->modelLoaded = true);
}

bool KinDynComputations::isValid() const
{
    return m_pimpl->modelLoaded;
}

const iDynTree::Model& KinDynComputations::model() const
{
    return m_pimpl->idyntreeModel;
}

const iDynTree::Model& KinDynComputations::getRobotModel() const
{
    return m_pimpl->idyntreeModel;
}

}
