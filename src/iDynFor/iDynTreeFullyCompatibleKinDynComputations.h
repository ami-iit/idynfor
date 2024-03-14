// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNFOR_IDYNTREE_FULLY_COMPATIBLE_KINDYNCOMPUTATIONS_H
#define IDYNfOR_IDYNTREE_FULLY_COMPATIBLE_KINDYNCOMPUTATIONS_H

#include <memory>

#include <iDynTree/Model/Model.h>
// This header provides the iDynTree::FrameVelocityRepresentation enum
#include <iDynTree/Model/FreeFloatingMatrices.h>

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

    bool setFrameVelocityRepresentation(const iDynTree::FrameVelocityRepresentation frameVelRepr);
    iDynTree::FrameVelocityRepresentation getFrameVelocityRepresentation() const;

    const iDynTree::Model& model() const;
    const iDynTree::Model& getRobotModel() const;

    bool setRobotState(const iDynTree::Transform& world_T_base,
                       const iDynTree::VectorDynSize& s,
                       const iDynTree::Twist& base_velocity,
                       const iDynTree::VectorDynSize& s_dot,
                       const iDynTree::Vector3& world_gravity);

    void getRobotState(iDynTree::Transform& world_T_base,
                       iDynTree::VectorDynSize& s,
                       iDynTree::Twist& base_velocity,
                       iDynTree::VectorDynSize& s_dot,
                       iDynTree::Vector3& world_gravity) const;

    bool setRobotAcceleration(const iDynTree::Vector6& baseAcc,
                          const iDynTree::VectorDynSize& s_ddot);

    void getRobotAcceleration(iDynTree::Vector6& baseAcc,
                              iDynTree::VectorDynSize& s_ddot) const;

    int getFrameIndex(const std::string& frameName) const;
    std::string getFrameName(const iDynTree::FrameIndex frameIndex) const;
    iDynTree::Transform getWorldTransform(const iDynTree::FrameIndex frameIndex);

    iDynTree::Twist getFrameVel(const std::string& frameName);
    bool getFrameVel(const std::string& frameName, iDynTree::Span<double> twist);
    iDynTree::Twist getFrameVel(const iDynTree::FrameIndex frameIdx);
    bool getFrameVel(const iDynTree::FrameIndex frameIdx, iDynTree::Span<double> twist);

    bool getFrameFreeFloatingJacobian(const std::string & frameName,
                                      iDynTree::MatrixDynSize & outJacobian);
    bool getFrameFreeFloatingJacobian(const iDynTree::FrameIndex frameIndex,
                                      iDynTree::MatrixDynSize & outJacobian);
    bool getFrameFreeFloatingJacobian(const std::string & frameName,
                                      iDynTree::MatrixView<double> outJacobian);
    bool getFrameFreeFloatingJacobian(const iDynTree::FrameIndex frameIndex,
                                      iDynTree::MatrixView<double> outJacobian);

    iDynTree::Vector6 getFrameAcc(const std::string & frameName);
    bool getFrameAcc(const std::string & frameName,
                     iDynTree::Span<double> frame_acceleration);
    iDynTree::Vector6 getFrameAcc(const iDynTree::FrameIndex frameIdx);
    bool getFrameAcc(const iDynTree::FrameIndex frameName,
                     iDynTree::Span<double> frame_acceleration);

    iDynTree::Vector6 getFrameAcc(const std::string & frameName,
                        const iDynTree::Vector6& baseAcc,
                        const iDynTree::VectorDynSize& s_ddot);
    bool getFrameAcc(const std::string & frameName,
                     iDynTree::Span<const double> baseAcc,
                     iDynTree::Span<const double> s_ddot,
                     iDynTree::Span<double> frame_acceleration);
    iDynTree::Vector6 getFrameAcc(const iDynTree::FrameIndex frameIdx,
                        const iDynTree::Vector6& baseAcc,
                        const iDynTree::VectorDynSize& s_ddot);
    bool getFrameAcc(const iDynTree::FrameIndex frameName,
                     iDynTree::Span<const double> baseAcc,
                     iDynTree::Span<const double> s_ddot,
                     iDynTree::Span<double> frame_acceleration);

    bool getFreeFloatingMassMatrix(iDynTree::MatrixDynSize& freeFloatingMassMatrix);
    bool getFreeFloatingMassMatrix(iDynTree::MatrixView<double> freeFloatingMassMatrix);
};

} // namespace iDynTreeFullyCompatible

} // namespace iDynFor

#endif
