/*
 * SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef IDYNFOR_IDYNTREE_FULLY_COMPATIBLE_KINDYNCOMPUTATIONS_H
#define IDYNfOR_IDYNTREE_FULLY_COMPATIBLE_KINDYNCOMPUTATIONS_H

#include <memory>

#include <iDynTree/Model/Model.h>

namespace iDynFor
{

namespace iDynTreeFullyCompatible
{

/*
 * \brief Variant of KinDynComputations with interface fully compatible with
 * iDynTree::KinDynComputations types.
 *
 * This class provides a fully drop-in compatible variable of iDynTree::KinDynComputations but based
 * on iDynFor/pinocchio, to provide a critical speed improvement, without the compilation time
 * overhead caused by including the pinocchio headers. No documentation is provided, for each
 * method, for the documentation check the documentation of the corresponding method in
 * iDynTre::KinDynComputations .
 */
class KinDynComputations
{
private:
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;

public:
    KinDynComputations();
    virtual ~KinDynComputations();

    bool loadRobotModel(const iDynTree::Model& model);
    bool isValid() const;

    const iDynTree::Model& model() const;
    const iDynTree::Model& getRobotModel() const;

    bool setRobotState(const iDynTree::Transform& world_T_base,
                       const iDynTree::VectorDynSize& s,
                       const iDynTree::Twist& base_velocity,
                       const iDynTree::VectorDynSize& s_dot,
                       const iDynTree::Vector3& world_gravity);

    int getFrameIndex(const std::string& frameName) const;
    std::string getFrameName(const iDynTree::FrameIndex frameIndex) const;
    iDynTree::Transform getWorldTransform(const iDynTree::FrameIndex frameIndex);
};

} // namespace iDynTreeFullyCompatible

} // namespace iDynFor

#endif
