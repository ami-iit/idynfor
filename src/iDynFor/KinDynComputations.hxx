/*
 * SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <iDynFor/iDynTreePinocchioConversions.h>

#include <iDynTree/Core/Utils.h>

namespace iDynFor
{

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
    const pinocchio::SE3Tpl<Scalar, Options>& world_H_base,
    const VectorXs& joint_pos,
    const Vector6s& base_velocity,
    const VectorXs& joint_vel,
    const Eigen::Matrix<Scalar, 3, 1, Options>& world_gravity)
{
    // TODO(traversaro): call as soon as it is implemented
    // this->invalidateCache();

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
    const iDynTree::FrameIndex frameIndex, pinocchio::SE3& world_H_frame)
{
    // TODO: implement frame different from the base one
    iDynTree::reportErrorIf(frameIndex != 0,
                            "iDynFor::KinDynComputationsTpl::getWorldTransform",
                            "requested frame not supported");
    world_H_frame = m_world_H_base;
    return true;
}

} // namespace iDynFor
