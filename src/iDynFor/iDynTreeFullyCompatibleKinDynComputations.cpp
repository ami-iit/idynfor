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
    // Dummy
    Eigen::VectorXd bufferJointPos;
    Eigen::VectorXd bufferJointVel;
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

bool KinDynComputations::setFrameVelocityRepresentation(
    const iDynTree::FrameVelocityRepresentation frameVelRepr)
{
    return m_pimpl->kindyn.setFrameVelocityRepresentation(frameVelRepr);
}

iDynTree::FrameVelocityRepresentation KinDynComputations::getFrameVelocityRepresentation() const
{
    return m_pimpl->kindyn.getFrameVelocityRepresentation();
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
    Eigen::Matrix<double, 6, 1> base_velocity_eig = iDynTree::toEigen(base_velocity);
    Eigen::Matrix<double, 3, 1> world_gravity_eig = iDynTree::toEigen(world_gravity);
    m_pimpl->bufferJointPos = iDynTree::toEigen(s);
    m_pimpl->bufferJointVel = iDynTree::toEigen(s_dot);

    return m_pimpl->kindyn.setRobotState(toPinocchio(world_H_base),
                                         m_pimpl->bufferJointPos,
                                         base_velocity_eig,
                                         m_pimpl->bufferJointVel,
                                         world_gravity_eig);
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

iDynTree::Twist KinDynComputations::getFrameVel(const std::string& frameName)
{
    return getFrameVel(getFrameIndex(frameName));
}

bool KinDynComputations::getFrameVel(const std::string& frameName, iDynTree::Span<double> twist)
{
    return getFrameVel(getFrameIndex(frameName), twist);
}

bool KinDynComputations::getFrameVel(const iDynTree::FrameIndex frameIdx,
                                     iDynTree::Span<double> twist)
{
    constexpr int expected_twist_size = 6;
    bool ok = twist.size() == expected_twist_size;
    if (!ok)
    {
        iDynTree::reportError("KinDynComputations", "getFrameVel", "Wrong size in input twist");
        return false;
    }

    iDynTree::toEigen(twist) = iDynTree::toEigen(getFrameVel(frameIdx));
    return true;
}

iDynTree::Twist KinDynComputations::getFrameVel(const iDynTree::FrameIndex frameIndex)
{
    Eigen::Matrix<double, 6, 1> ret_twist;
    iDynTree::Twist ret_twist_idyntree;
    m_pimpl->kindyn.getFrameVel(frameIndex, ret_twist);
    iDynTree::toEigen(ret_twist_idyntree.getLinearVec3()) = ret_twist.segment<3>(0);
    iDynTree::toEigen(ret_twist_idyntree.getAngularVec3()) = ret_twist.segment<3>(3);
    return ret_twist_idyntree;
}

} // namespace iDynTreeFullyCompatible
} // namespace iDynFor
