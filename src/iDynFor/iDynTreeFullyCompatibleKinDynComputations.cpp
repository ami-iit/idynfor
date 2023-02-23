/*
 * SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <iDynFor/iDynTreeFullyCompatibleKinDynComputations.h>

#include <iDynFor/KinDynComputations.h>

namespace iDynFor
{

namespace iDynTreeFullyCompatible
{

struct KinDynComputations::Impl
{
    iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl> kindyn;
};

KinDynComputations::KinDynComputations()
{
    m_pimpl = std::make_unique<KinDynComputations::Impl>();
}

KinDynComputations::~KinDynComputations() = default;

bool KinDynComputations::loadRobotModel(const iDynTree::Model& model)
{
    return m_pimpl->kindyn.loadRobotModel(model);
}

bool KinDynComputations::isValid() const
{
    return m_pimpl->kindyn.isValid();
}

const iDynTree::Model& KinDynComputations::model() const
{
    return m_pimpl->kindyn.model();
}

const iDynTree::Model& KinDynComputations::getRobotModel() const
{
    return m_pimpl->kindyn.getRobotModel();
}

bool KinDynComputations::setRobotState(const iDynTree::Transform& world_H_base,
                                       const iDynTree::VectorDynSize& s,
                                       const iDynTree::Twist& base_velocity,
                                       const iDynTree::VectorDynSize& s_dot,
                                       const iDynTree::Vector3& world_gravity)
{
    // TODO add handling of other arguments
    Eigen::VectorXd dummy_vec;
    Eigen::Matrix<double, 6, 1> dummy_twist;
    Eigen::Matrix<double, 3, 1> dummy_vec3;

    return m_pimpl->kindyn.setRobotState(toPinocchio(world_H_base),
                                         dummy_vec,
                                         dummy_twist,
                                         dummy_vec,
                                         dummy_vec3);

    return false;
}

int KinDynComputations::getFrameIndex(const std::string& frameName) const
{
    return m_pimpl->kindyn.getFrameIndex(frameName);
}

std::string KinDynComputations::getFrameName(const iDynTree::FrameIndex frameIndex) const
{
    return m_pimpl->kindyn.getFrameName(frameIndex);
}

iDynTree::Transform KinDynComputations::getWorldTransform(const iDynTree::FrameIndex frameIndex)
{
    pinocchio::SE3 pinocchio_SE3;
    m_pimpl->kindyn.getWorldTransform(frameIndex, pinocchio_SE3);
    return fromPinocchio(pinocchio_SE3);
}

} // namespace iDynTreeFullyCompatible
} // namespace iDynFor
