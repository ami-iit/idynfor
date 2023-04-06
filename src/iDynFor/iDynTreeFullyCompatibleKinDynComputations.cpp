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
    iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl>::Matrix6Xs bufferJacobian;
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
    // we cannot avoid to allocate memory since the linear and the angular components of a twist
    // in iDynTree are not contiguous in space
    Eigen::Matrix<double, 6, 1> base_velocity_eig = iDynTree::toEigen(base_velocity);
    m_pimpl->bufferJacobian.resize(6, 6 + s.size());

    return m_pimpl->kindyn.setRobotState(toPinocchio(world_H_base),
                                         iDynTree::toEigen(s),
                                         base_velocity_eig,
                                         iDynTree::toEigen(s_dot),
                                         iDynTree::toEigen(world_gravity));
}

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

bool KinDynComputations::getFrameFreeFloatingJacobian(const std::string & frameName,
                                    iDynTree::MatrixDynSize & outJacobian)
{
    return getFrameFreeFloatingJacobian(getFrameIndex(frameName),outJacobian);
}

bool KinDynComputations::getFrameFreeFloatingJacobian(const iDynTree::FrameIndex frameIndex,
                                  iDynTree::MatrixDynSize & outJacobian)
{
    return getFrameFreeFloatingJacobian(frameIndex, iDynTree::MatrixView<double>(outJacobian));
}

bool KinDynComputations::getFrameFreeFloatingJacobian(const std::string & frameName,
                                      iDynTree::MatrixView<double> outJacobian)
{
    return getFrameFreeFloatingJacobian(getFrameIndex(frameName),outJacobian);
}

bool KinDynComputations::getFrameFreeFloatingJacobian(const iDynTree::FrameIndex frameIndex,
                                      iDynTree::MatrixView<double> outJacobian)
{
    // We need to do a copy here to go from ColumnMajor to RowMajor
    // TODO understand if we can avoid this, if I just passed iDynTree::toEigen(outJacobian) and change the
    // bool KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::getFrameFreeFloatingJacobian signature to
    // accept a Eigen::Ref or Eigen::Map, I get as errors
    // /home/traversaro/idynfor/src/iDynFor/iDynTreeFullyCompatibleKinDynComputations.cpp:153:86: error: cannot convert 'Eigen::Map<Eigen::Matrix<double, -1, -1, 1>, 0, Eigen::Stride<-1, -1> >' to 'Eigen::Ref<Eigen::Matrix<double, -1, -1>, 0, Eigen::OuterStride<> >&'
    bool ok = m_pimpl->kindyn.getFrameFreeFloatingJacobian(frameIndex, m_pimpl->bufferJacobian);
    iDynTree::toEigen(outJacobian) = m_pimpl->bufferJacobian;
    return ok;
}

} // namespace iDynTreeFullyCompatible
} // namespace iDynFor
