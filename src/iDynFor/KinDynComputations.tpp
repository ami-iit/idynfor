/*
 * SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <iDynFor/iDynTreePinocchioConversions.h>

#include <iDynTree/Core/Utils.h>

#include <pinocchio/algorithm/joint-configuration.hpp>

namespace iDynFor
{

// Internal functions methods

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
void KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::invalidateCache()
{
    m_isFwdKinematicsUpdated = false;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
void KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::computeFwdKinematics()
{
    if( this->m_isFwdKinematicsUpdated )
    {
        return;
    }

    this->convertModelStateFromiDynTreeToPinocchio();

    // Compute position and velocity kinematics
    // TODO(traversaro): Swich to the four-parameters variant when we add support for velocities
    pinocchio::forwardKinematics(m_pinModel, m_pinData, m_pin_model_position);

    this->m_isFwdKinematicsUpdated = true;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
void KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::convertModelStateFromiDynTreeToPinocchio()
{
    typedef Eigen::Matrix<Scalar, 4, 1, Options> Vector4s;
    typedef Eigen::Quaternion<Scalar, Options> Quaternions;

    // Position
    // The initial three elements of the pinocchio q vector are the origin of the base frame
    // w.r.t. to the world frame, i.e. {}^A o_B \in \mathbb{R}^3
    m_pin_model_position.block(0,0,3,1) = m_world_H_base.translation();

    // The next four elements are the quaternion corresponding to the {}^A R_B \in SO(3) orientation
    // As the quaternion's coeffs method is used by pinocchio, pay attention that the order is (imaginary, real)
    Quaternions quaternion(m_world_H_base.rotation());
    m_pin_model_position.block(3,0,4,1) = Eigen::Map<Vector4s>(quaternion.coeffs().data());

    // The rest of the elements are the position of the internal joints, accounting for the
    // difference in position coordinate serialization between iDynTree and Pinocchio
    // TODO(traversaro) : handle this

    return;
}


template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
inline bool KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::loadRobotModel(
    const iDynTree::Model& model)
{
    m_idyntreeModel = model;
    // TODO: handle verbosity
    bool verbose = true;

    // TODO: understand if we should catch exception and return bool?
    iDynFor::buildPinocchioModelfromiDynTree<Scalar, Options, JointCollectionTpl>(m_idyntreeModel,
                                                                                  m_pinModel,
                                                                                  verbose);

    m_pinData = pinocchio::DataTpl<Scalar, Options, JointCollectionTpl>(m_pinModel);

    // Resize m_pin_model_position to right size
    m_pin_model_position.resize(m_pinModel.nq);
    m_pin_model_position.setZero();

    return (m_modelLoaded = true);
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
bool KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::isValid() const
{
    return m_modelLoaded;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
const iDynTree::Model& KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::model() const
{
    return m_idyntreeModel;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
const iDynTree::Model&
KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::getRobotModel() const
{
    return m_idyntreeModel;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
bool KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::setRobotState(
    const SE3s& world_H_base,
    const VectorXs& joint_pos,
    const Vector6s& base_velocity,
    const VectorXs& joint_vel,
    const Vector3s& world_gravity)
{
    this->invalidateCache();

    // Save pos
    m_world_H_base = world_H_base;
    m_joint_pos = joint_pos;

    // Save gravity
    // TODO: implement

    // Save vel
    // TODO

    return true;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
int KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::getFrameIndex(
    const std::string& frameName) const
{
    int index = m_idyntreeModel.getFrameIndex(frameName);
    iDynTree::reportErrorIf(index < 0,
                            "iDynFor::KinDynComputationsTpl::getFrameIndex",
                            "requested frameName not found in model");
    return index;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
std::string KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::getFrameName(
    iDynTree::FrameIndex frameIndex) const
{
    return m_idyntreeModel.getFrameName(frameIndex);
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
bool KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::getWorldTransform(
    const iDynTree::FrameIndex frameIndex, SE3s& world_H_frame)
{
    // TODO: implement frame different from the base one
    iDynTree::reportErrorIf(frameIndex != 0,
                            "iDynFor::KinDynComputationsTpl::getWorldTransform",
                            "requested frame not supported");

    this->computeFwdKinematics();

    // Convert iDynTree::FrameIndex to pinocchio::FrameIndex
    // TODO(traversaro): also support additional frames, not only frames associated to a link
    // TODO(traversaro): cache this information, there is no need to do a string search every time
    pinocchio::FrameIndex pinFrameIndex = m_pinModel.getFrameId(m_idyntreeModel.getFrameName(frameIndex));

    // After computeFwdKinematics computed the forwardKinematics, in m_pinData it is
    // store the universe_H_<..> transform for each joint frame
    // From the iDynTree-point of view, we are interested only in the universe_H_<..>
    // transforms of link frames and additional frames, so we compute the one request
    // See https://github.com/stack-of-tasks/pinocchio/issues/802#issuecomment-496210616
    // TODO(traversaro): understand if it is worth to cache also this
    pinocchio::updateFramePlacement(m_pinModel, m_pinData, pinFrameIndex);
    world_H_frame = m_pinData.oMf[pinFrameIndex];

    return true;
}

} // namespace iDynFor
