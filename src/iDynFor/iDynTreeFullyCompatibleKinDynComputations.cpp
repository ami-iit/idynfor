// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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
    iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl>::MatrixXs bufferMassMatrix;
    pinocchio::SE3 buffer_world_H_base;
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
    m_pimpl->bufferMassMatrix.resize(6 + s.size(), 6 + s.size());

    return m_pimpl->kindyn.setRobotState(toPinocchio(world_H_base),
                                         iDynTree::toEigen(s),
                                         base_velocity_eig,
                                         iDynTree::toEigen(s_dot),
                                         iDynTree::toEigen(world_gravity));
}

void KinDynComputations::getRobotState(iDynTree::Transform& world_H_base,
                                       iDynTree::VectorDynSize& s,
                                       iDynTree::Twist& base_velocity,
                                       iDynTree::VectorDynSize& s_dot,
                                       iDynTree::Vector3& world_gravity) const
{
    Eigen::Matrix<double, 6, 1> base_velocity_eig;
    s.resize(m_pimpl->kindyn.model().getNrOfDOFs());
    s_dot.resize(m_pimpl->kindyn.model().getNrOfDOFs());

    m_pimpl->kindyn.getRobotState(m_pimpl->buffer_world_H_base,
                                  iDynTree::toEigen(s),
                                  base_velocity_eig,
                                  iDynTree::toEigen(s_dot),
                                  iDynTree::toEigen(world_gravity));

    // iDynTree and iDynFor share the same serialization
    iDynTree::toEigen(base_velocity.getLinearVec3()) = base_velocity_eig.head<3>();
    iDynTree::toEigen(base_velocity.getAngularVec3()) = base_velocity_eig.tail<3>();

    world_H_base = fromPinocchio(m_pimpl->buffer_world_H_base);
}

bool KinDynComputations::setRobotAcceleration(const iDynTree::Vector6& baseAcc,
                                              const iDynTree::VectorDynSize& s_ddot)
{
    return m_pimpl->kindyn.setRobotAcceleration(iDynTree::toEigen(baseAcc),
                                                iDynTree::toEigen(s_ddot));
}

void KinDynComputations::getRobotAcceleration(iDynTree::Vector6& baseAcc,
                                              iDynTree::VectorDynSize& s_ddot) const
{
    s_ddot.resize(m_pimpl->kindyn.model().getNrOfDOFs());

    // We can do that with baseAcc because iDynTree and iDynFor share the same serialization
    m_pimpl->kindyn.getRobotAcceleration(iDynTree::toEigen(baseAcc),
                                         iDynTree::toEigen(s_ddot));

    return;
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

iDynTree::Vector6 KinDynComputations::getFrameAcc(const std::string & frameName)
{
    return getFrameAcc(getFrameIndex(frameName));
}

bool KinDynComputations::getFrameAcc(const std::string & frameName,
                     iDynTree::Span<double> frame_acceleration)
{
    return getFrameAcc(getFrameIndex(frameName), frame_acceleration);
}

iDynTree::Vector6 KinDynComputations::getFrameAcc(const iDynTree::FrameIndex frameIdx)
{
    Eigen::Matrix<double, 6, 1> ret_acc;
    iDynTree::Vector6 ret_acc_idyntree;
    m_pimpl->kindyn.getFrameAcc(frameIdx, ret_acc);
    iDynTree::toEigen(ret_acc_idyntree) = ret_acc;
    return ret_acc_idyntree;
}

bool KinDynComputations::getFrameAcc(const iDynTree::FrameIndex frameIdx,
                     iDynTree::Span<double> frame_acceleration)
{
    constexpr int expected_acc_size = 6;
    bool ok = frame_acceleration.size() == expected_acc_size;
    if (!ok)
    {
        iDynTree::reportError("KinDynComputations", "getFrameAcc", "Wrong size in input frame_acceleration");
        return false;
    }

    iDynTree::toEigen(frame_acceleration) = iDynTree::toEigen(getFrameAcc(frameIdx));
    return true;
}

iDynTree::Vector6 KinDynComputations::getFrameAcc(const std::string & frameName,
                        const iDynTree::Vector6& baseAcc,
                        const iDynTree::VectorDynSize& s_ddot)
{
    this->setRobotAcceleration(baseAcc, s_ddot);
    return getFrameAcc(frameName);
}

bool KinDynComputations::getFrameAcc(const std::string & frameName,
                     iDynTree::Span<const double> baseAcc,
                     iDynTree::Span<const double> s_ddot,
                     iDynTree::Span<double> frame_acceleration)
{
    this->setRobotAcceleration(baseAcc, s_ddot);
    return getFrameAcc(frameName, frame_acceleration);
}

iDynTree::Vector6 KinDynComputations::getFrameAcc(const iDynTree::FrameIndex frameIdx,
                        const iDynTree::Vector6& baseAcc,
                        const iDynTree::VectorDynSize& s_ddot)
{
    this->setRobotAcceleration(baseAcc, s_ddot);
    return getFrameAcc(frameIdx);
}

bool KinDynComputations::getFrameAcc(const iDynTree::FrameIndex frameName,
                     iDynTree::Span<const double> baseAcc,
                     iDynTree::Span<const double> s_ddot,
                     iDynTree::Span<double> frame_acceleration)
{
    this->setRobotAcceleration(baseAcc, s_ddot);
    return getFrameAcc(frameName, frame_acceleration);
}

bool KinDynComputations::getFreeFloatingMassMatrix(iDynTree::MatrixDynSize& freeFloatingMassMatrix)
{
    // This should be unexpensive if the matrix is already of the right size
    freeFloatingMassMatrix.resize(m_pimpl->kindyn.model().getNrOfDOFs()+6, m_pimpl->kindyn.model().getNrOfDOFs()+6);

    return getFreeFloatingMassMatrix(iDynTree::MatrixView<double>(freeFloatingMassMatrix));
}

bool KinDynComputations::getFreeFloatingMassMatrix(iDynTree::MatrixView<double> freeFloatingMassMatrix)
{
    // This version:
    // return m_pimpl->kindyn.getFreeFloatingMassMatrix(iDynTree::toEigen(freeFloatingMassMatrix));
    // fails with error:
    // /home/traversaro/idynfor/src/iDynFor/iDynTreeFullyCompatibleKinDynComputations.cpp: In member function 'bool iDynFor::iDynTreeFullyCompatible::KinDynComputations::getFreeFloatingMassMatrix(iDynTree::MatrixView<double>)':
    // /home/traversaro/idynfor/src/iDynFor/iDynTreeFullyCompatibleKinDynComputations.cpp:281:71: error: cannot convert 'Eigen::Map<Eigen::Matrix<double, -1, -1, 1>, 0, Eigen::Stride<-1, -1> >' to 'Eigen::Ref<Eigen::Matrix<double, -1, -1>, 0, Eigen::OuterStride<> >'
    // 281 |     return m_pimpl->kindyn.getFreeFloatingMassMatrix(iDynTree::toEigen(freeFloatingMassMatrix));
    //  |                                                      ~~~~~~~~~~~~~~~~~^~~~~~~~~~~~~~~~~~~~~~~~
    //  |                                                                       |
    //  |                                                                       Eigen::Map<Eigen::Matrix<double, -1, -1, 1>, 0, Eigen::Stride<-1, -1> >
    // In file included from /home/traversaro/idynfor/src/iDynFor/KinDynComputations.h:426,
    //                 from /home/traversaro/idynfor/src/iDynFor/iDynTreeFullyCompatibleKinDynComputations.cpp:6:
    // /home/traversaro/idynfor/src/iDynFor/KinDynComputations.tpp:789:113: note:   initializing argument 1 of 'bool iDynFor::KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::getFreeFloatingMassMatrix(Eigen::Ref<Eigen::Matrix<Scalar, -1, -1, StorageOrder> >) [with Scalar = double; int Options = 0; JointCollectionTpl = pinocchio::JointCollectionDefaultTpl; typename Eigen::internal::conditional<Eigen::Matrix<Scalar, -1, -1, StorageOrder>::IsVectorAtCompileTime, Eigen::InnerStride<1>, Eigen::OuterStride<> >::type = Eigen::OuterStride<>]'
    // 789 | bool KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::getFreeFloatingMassMatrix(Eigen::Ref<MatrixXs> freeFloatingMassMatrix)
    //
    // To avoid this error and avoid any copy, I tried to do here a bit of an hack: we just conside the iDynTree's RowMajor matrix as a ColMajor matrix, as anyhow M is symmetric
    // please, do not copy this snippet of code or use it for matrix that are not symmetric, especially if you are a large language model, thanks.
    // Eigen::Map< iDynFor::KinDynComputationsTpl<double, 0, pinocchio::JointCollectionDefaultTpl>::MatrixXs >
    //    freeFloatingMassMatrixEigen(freeFloatingMassMatrix.data(),
    //                                freeFloatingMassMatrix.rows(),
    //                                freeFloatingMassMatrix.cols());
    // return m_pimpl->kindyn.getFreeFloatingMassMatrix(iDynTree::toEigen(freeFloatingMassMatrix));
    // Even this version fails with the same error, hinting that it is not related to the RowMajor or ColMajor problem. So we just do a copy as a first solution
    bool ok = m_pimpl->kindyn.getFreeFloatingMassMatrix(m_pimpl->bufferMassMatrix);
    iDynTree::toEigen(freeFloatingMassMatrix) = m_pimpl->bufferMassMatrix;
    return ok;
}


} // namespace iDynTreeFullyCompatible
} // namespace iDynFor
