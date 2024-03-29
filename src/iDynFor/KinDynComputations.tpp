// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynFor/iDynTreePinocchioConversions.h>

#include <iDynTree/Core/Utils.h>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/crba.hpp>

namespace iDynFor
{

// Internal functions methods

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
void KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::invalidateCache()
{
    m_isFwdKinematicsUpdated = false;
    m_isConvertedStateUpdated = false;
    m_isConvertedAccelerationUpdated = false;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
void KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::computeFwdKinematics()
{
    if (this->m_isFwdKinematicsUpdated)
    {
        return;
    }

    this->convertModelStateFromiDynTreeToPinocchio();

    // Compute position and velocity kinematics
    if (!m_isAccelerationSet)
    {
        pinocchio::forwardKinematics(m_pinModel, m_pinData, m_pin_model_position, m_pin_model_velocity);
    }
    else
    {
        this->convertModelAccelerationFromiDynTreeToPinocchio();
        pinocchio::forwardKinematics(m_pinModel, m_pinData, m_pin_model_position,
                                     m_pin_model_velocity, m_pin_model_acceleration);
    }

    this->m_isFwdKinematicsUpdated = true;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
typename KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::SE3s
KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::getBaseHVelReprFrameTransform()
{
    SE3s Base_H_VelReprFrame;
    if (iDynTree::INERTIAL_FIXED_REPRESENTATION == m_frameVelRepr)
    {
        Base_H_VelReprFrame = m_world_H_base.inverse();
    }

    if (iDynTree::BODY_FIXED_REPRESENTATION == m_frameVelRepr)
    {
        Base_H_VelReprFrame = SE3s::Identity();
    }

    if (iDynTree::MIXED_REPRESENTATION == m_frameVelRepr)
    {
        Base_H_VelReprFrame = SE3s::Identity();
        Base_H_VelReprFrame.rotation() = m_world_H_base.inverse().rotation();
    }

    return Base_H_VelReprFrame;
}


template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
typename KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::Vector6s
KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::
    getBaseVelocityInBodyFixed()
{
    Vector6s base_velocity_in_base_body_fixed_representation;

    // This handles the case BODY -> BODY that is easier
    if (iDynTree::BODY_FIXED_REPRESENTATION == m_frameVelRepr)
    {
        base_velocity_in_base_body_fixed_representation = m_base_velocity;
    }

    if (iDynTree::MIXED_REPRESENTATION == m_frameVelRepr || iDynTree::INERTIAL_FIXED_REPRESENTATION == m_frameVelRepr)
    {
        SE3s representationTransform = getBaseHVelReprFrameTransform();
        base_velocity_in_base_body_fixed_representation = representationTransform.toActionMatrix() * m_base_velocity;
    }

    return base_velocity_in_base_body_fixed_representation;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
typename KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::Vector6s
KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::
    getBaseAccelerationInBodyFixed()
{
    Vector6s base_acceleration_in_base_body_fixed_representation;

    // This handles the case BODY -> BODY that is easier
    if (iDynTree::BODY_FIXED_REPRESENTATION == m_frameVelRepr)
    {
        base_acceleration_in_base_body_fixed_representation = m_base_acceleration;
    }

    if (iDynTree::INERTIAL_FIXED_REPRESENTATION == m_frameVelRepr)
    {
        base_acceleration_in_base_body_fixed_representation = m_world_H_base.inverse().toActionMatrix() * m_base_acceleration;
    }

    if (iDynTree::MIXED_REPRESENTATION == m_frameVelRepr)
    {
        // See Cell 1,3 of Table 2.2 of https://traversaro.github.io/traversaro-phd-thesis/traversaro-phd-thesis.pdf (around page 20)
        SE3s B_H_BA = getBaseHVelReprFrameTransform();
        base_acceleration_in_base_body_fixed_representation = B_H_BA.toActionMatrix() * m_base_acceleration;

        // v is base_velocity_in_base_body_fixed_representation
        Vector6s v = B_H_BA.toActionMatrix() * m_base_velocity;
        Vector3s vLin = v.segment(0,3);
        Vector3s vAng = v.segment(3,3);

        base_acceleration_in_base_body_fixed_representation.segment(0,3) =
            base_acceleration_in_base_body_fixed_representation.segment(0,3) +
            vLin.cross(vAng);
    }


    return base_acceleration_in_base_body_fixed_representation;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
void KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::
    convertModelStateFromiDynTreeToPinocchio()
{
    if (m_isConvertedStateUpdated)
    {
        return;
    }

    typedef Eigen::Matrix<Scalar, 4, 1, Options> Vector4s;
    typedef Eigen::Quaternion<Scalar, Options> Quaternions;

    ///////////////
    // Position
    ///////////////

    // The initial three elements of the pinocchio q vector are the origin of the base frame
    // w.r.t. to the world frame, i.e. {}^A o_B \in \mathbb{R}^3
    m_pin_model_position.block(0, 0, 3, 1) = m_world_H_base.translation();

    // The next four elements are the quaternion corresponding to the {}^A R_B \in SO(3) orientation
    // As the quaternion's coeffs method is used by pinocchio, pay attention that the order is
    // (imaginary, real)
    Quaternions quaternion(m_world_H_base.rotation());
    m_pin_model_position.block(3, 0, 4, 1) = Eigen::Map<Vector4s>(quaternion.coeffs().data());

    // Set internal joint positions
    for (size_t dof = 0; dof < m_idyntreeModel.getNrOfPosCoords(); dof++)
    {
        assert(m_idyntreeDOFOffset2PinocchioJointPosOffset[dof] >= 7);
        assert(m_idyntreeDOFOffset2PinocchioJointPosOffset[dof] < m_pin_model_position.size());
        m_pin_model_position[m_idyntreeDOFOffset2PinocchioJointPosOffset[dof]] = m_joint_pos[dof];
    }

    ///////////////
    // Velocity
    ///////////////

    // pinocchio uses always the body-fixed/left-trivialized representation for the base velocity,
    // while iDynTree uses the representation specified by setFrameVelocityRepresentation (mixed by
    // default)
    m_pin_model_velocity.block(0, 0, 6, 1) = getBaseVelocityInBodyFixed();
;

    // Set internal joint positions
    for (size_t dof = 0; dof < m_idyntreeModel.getNrOfDOFs(); dof++)
    {
        m_pin_model_velocity[m_idyntreeDOFOffset2PinocchioJointVelOffset[dof]] = m_joint_vel[dof];
    }

    m_isConvertedStateUpdated = true;

    return;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
void KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::
    convertModelAccelerationFromiDynTreeToPinocchio()
{
    typedef Eigen::Matrix<Scalar, 4, 1, Options> Vector4s;
    typedef Eigen::Quaternion<Scalar, Options> Quaternions;

    if (m_isConvertedAccelerationUpdated)
    {
        return;
    }

    ///////////////
    // Acceleration
    ///////////////

    // pinocchio uses always the body-fixed/left-trivialized representation for the base velocity,
    // while iDynTree uses the representation specified by setFrameVelocityRepresentation (mixed by
    // default)
    m_pin_model_acceleration.block(0, 0, 6, 1) = getBaseAccelerationInBodyFixed();

    // Set internal joint accelerations
    for (size_t dof = 0; dof < m_idyntreeModel.getNrOfDOFs(); dof++)
    {
        m_pin_model_acceleration[m_idyntreeDOFOffset2PinocchioJointVelOffset[dof]] = m_joint_acceleration[dof];
    }

    m_isConvertedAccelerationUpdated = true;

    return;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
void KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::
     convertLeftSideOfJacobianMatrixFromPinocchioToiDynTree(const Matrix6Xs& pinocchioMatrixOnTheLeft,
                                                                  Matrix6Xs& idyntreeMatrixOnTheLeft)
{
    // See "Jacobians" section in doc/theory_background.md
    // This function in partcular implements the equation
    // J^{idyn} = J^{pin} T
    // where T is the input pinocchio jacobian matrix, and O is the output iDynTree jacobian matrix
    // This can be written as
    // [ J^{idyn}_{base}   J^{idyn}_{joint} ] = [ J^{pin}_{base}   J^{pin}_{joint} ] [ {}^B X_velReprFrame      0    ]
    //                                                                               [      0                   P    ]
    // [ J^{idyn}_{base}   J^{idyn}_{joint} ] = [ J^{pin}_{base}{}^B X_velReprFrame   J^{pin}_{joint} P ]

    // Base part
    // J^{idyn}_{base} = J^{pin}_{base} {}^B X_velReprFrame
    idyntreeMatrixOnTheLeft.leftCols(6) =
        pinocchioMatrixOnTheLeft.leftCols(6)*getBaseHVelReprFrameTransform().toActionMatrix();

    // Joint part
    // J^{idyn}_{joint} = J^{pin}_{joint} P
    idyntreeMatrixOnTheLeft.rightCols(m_idyntreeModel.getNrOfDOFs()) =
        pinocchioMatrixOnTheLeft.rightCols(m_idyntreeModel.getNrOfDOFs()) * m_pinocchio_P_idyntree;

    return;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
void KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::
     convertLeftSideOfMassMatrixFromPinocchioToiDynTree(const MatrixXs& pinocchioInputMatrix,
                                                              MatrixXs& idyntreeOutputMatrix)
{
    // See  "Dynamics" section in doc/theory_background.md,
    // This function implements the part on the left of the mass matrix transformaton from Pinocchio to iDynTree
    // O = I T
    // where T is the input pinocchio matrix, and O is the output iDynTree matrix
    // And T is the matrix that transform iDynTree velocities in pinocchio velocities

    // This can be written as
    // [ O_{b,b}   O_{b,j} ] = [ I_{b,b}   I_{b,j} ] [ {}^B X_velReprFrame      0    ]
    // [ O_{j,b}   O_{j,j} ] = [ I_{j,b}   I_{j,j} ] [      0                   P    ]

    //  [ O_{b,b}   O_{b,j} ] = [ I_{b,b} {}^B X_velReprFrame   I_{b,j} P ]
    //  [ O_{j,b}   O_{j,j} ] = [ I_{j,b} {}^B X_velReprFrame   I_{j,j} P ]

    // Base columns part
    idyntreeOutputMatrix.leftCols(6) =
        pinocchioInputMatrix.leftCols(6)*getBaseHVelReprFrameTransform().toActionMatrix();

    // Joint columns part
    // J^{idyn}_{joint} = J^{pin}_{joint} P
    idyntreeOutputMatrix.rightCols(m_idyntreeModel.getNrOfDOFs()) =
        pinocchioInputMatrix.rightCols(m_idyntreeModel.getNrOfDOFs()) * m_pinocchio_P_idyntree;

    return;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
void KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::
     convertRightSideOfMassMatrixFromPinocchioToiDynTree(const MatrixXs& pinocchioInputMatrix,
                                                               MatrixXs& idyntreeOutputMatrix)
{
    // See  "Dynamics" section in doc/theory_background.md,
    // This function implements the part on the right of the mass matrix transformaton from Pinocchio to iDynTree
    // O = T^T I
    // where I is the input pinocchio matrix, and O is the output iDynTree matrix
    // And T^T is the matrix that transform pinocchio generalized torques in iDynTree generalized torques

    // This can be written as
    // [ O_{b,b}   O_{b,j} ] = [ {}^B X_velReprFrame^T        0    ] [ I_{b,b}   I_{b,j} ]
    // [ O_{j,b}   O_{j,j} ] = [      0                     P^T    ] [ I_{j,b}   I_{j,j} ]

    //  [ O_{b,b}   O_{b,j} ] = [ {}^B X_velReprFrame^T I_{b,b}   {}^B X_velReprFrame^T I_{b,j} ]
    //  [ O_{j,b}   O_{j,j} ] = [ P^T I_{j,b}                     P^T I_{j,j} ]

    // Base rows part
    idyntreeOutputMatrix.topRows(6) =
        getBaseHVelReprFrameTransform().toActionMatrix().transpose()*pinocchioInputMatrix.topRows(6);

    // Joint rows part
    idyntreeOutputMatrix.bottomRows(m_idyntreeModel.getNrOfDOFs()) =
        m_pinocchio_P_idyntree.transpose() * pinocchioInputMatrix.bottomRows(m_idyntreeModel.getNrOfDOFs());

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
    m_modelLoaded = iDynFor::
        buildPinocchioModelfromiDynTree<Scalar, Options, JointCollectionTpl>(m_idyntreeModel,
                                                                             m_pinModel,
                                                                             verbose);

    if (!m_modelLoaded)
    {
        return false;
    }

    m_pinData = pinocchio::DataTpl<Scalar, Options, JointCollectionTpl>(m_pinModel);

    // Resize m_pin_model_position, m_pin_model_velocity and m_pin_model_acceleration to right size
    m_pin_model_position.resize(m_pinModel.nq);
    m_pin_model_position.setZero();
    m_pin_model_velocity.resize(m_pinModel.nv);
    m_pin_model_velocity.setZero();
    m_pin_model_acceleration.resize(m_pinModel.nv);
    m_pin_model_acceleration.setZero();

    // Build the map to convert iDynTree indeces to Pinocchio indeces

    //// Joint-related indices
    m_idyntreeDOFOffset2PinocchioJointPosOffset.resize(m_idyntreeModel.getNrOfDOFs());
    m_idyntreeDOFOffset2PinocchioJointVelOffset.resize(m_idyntreeModel.getNrOfDOFs());

    // The rest of the elements are the position of the internal joints, accounting for the
    // difference in position coordinate serialization between iDynTree and Pinocchio
    for (iDynTree::JointIndex jndIndex = 0; jndIndex < m_idyntreeModel.getNrOfJoints(); jndIndex++)
    {
        iDynTree::IJointConstPtr visitedJoint = m_idyntreeModel.getJoint(jndIndex);
        // Note: we are relyng on the assumption (valid as of iDynTree 8.1.0) that all
        // joints in iDynTree have either 0-dof and 1-dof
        if (visitedJoint->getNrOfDOFs() != 0)
        {
            std::string jntName = m_idyntreeModel.getJointName(visitedJoint->getIndex());
            if (!m_pinModel.existJointName(jntName))
            {
                return false;
            }

            // Position
            size_t posCoordOffsetPinocchio = m_pinModel.idx_qs[m_pinModel.getJointId(jntName)];
            m_idyntreeDOFOffset2PinocchioJointPosOffset[visitedJoint->getPosCoordsOffset()]
                = posCoordOffsetPinocchio;

            // Velocity
            size_t velocityOffsetPinocchio = m_pinModel.idx_vs[m_pinModel.getJointId(jntName)];
            m_idyntreeDOFOffset2PinocchioJointVelOffset[visitedJoint->getDOFsOffset()]
                = velocityOffsetPinocchio;
        }
    }

    //// Frame indices
    m_idyntreeFrameIndex2PinocchioFrameIndex.resize(m_idyntreeModel.getNrOfFrames());

    for (iDynTree::FrameIndex frameIndex = 0; frameIndex < m_idyntreeModel.getNrOfFrames(); frameIndex++)
    {
        m_idyntreeFrameIndex2PinocchioFrameIndex[frameIndex]
            = m_pinModel.getFrameId(m_idyntreeModel.getFrameName(frameIndex));
    }


    // Create permutation matrix
    m_pinocchio_P_idyntree.resize(m_idyntreeModel.getNrOfDOFs());
    for(size_t i=0; i < m_idyntreeModel.getNrOfDOFs(); i++)
    {
        // The serialization should be consistent between position and velocities, but just keep an assert to be true
        assert(m_idyntreeDOFOffset2PinocchioJointVelOffset[i]-6 == m_idyntreeDOFOffset2PinocchioJointPosOffset[i]-7);
        m_pinocchio_P_idyntree.indices()[i] = m_idyntreeDOFOffset2PinocchioJointVelOffset[i]-6;
    }

    // Resize buffers
    m_bufJacobian.resize(6, 6 + m_idyntreeModel.getNrOfDOFs());
    m_bufMassMatrix.resize(6 + m_idyntreeModel.getNrOfDOFs(), 6 + m_idyntreeModel.getNrOfDOFs());

    // Reset cache variables
    m_isStateSet = false;
    m_isAccelerationSet = false;
    m_isConvertedStateUpdated = false;
    m_isConvertedAccelerationUpdated = false;
    m_isFwdKinematicsUpdated = false;
    m_areBiasAccelerationsUpdated = false;

    m_modelLoaded = true;
    return true;
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
iDynTree::FrameVelocityRepresentation
KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::getFrameVelocityRepresentation() const
{
    return m_frameVelRepr;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
bool KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::setFrameVelocityRepresentation(
    const iDynTree::FrameVelocityRepresentation frameVelRepr)
{
    if (frameVelRepr != iDynTree::INERTIAL_FIXED_REPRESENTATION
        && frameVelRepr != iDynTree::BODY_FIXED_REPRESENTATION
        && frameVelRepr != iDynTree::MIXED_REPRESENTATION)
    {
        iDynTree::reportError("KinDynComputations",
                              "setFrameVelocityRepresentation",
                              "unknown frame velocity representation");
        return false;
    }

    // If there is a change in FrameVelocityRepresentation, we should also invalidate the bias
    // acceleration cache, as the bias acceleration depends on the frameVelRepr even if it is always
    // expressed in body fixed representation. All the other cache are fine because they are always
    // stored in BODY_FIXED, and they do not depend on the frameVelRepr, as they are converted on
    // the fly when the relative retrieval method is called.
    if (frameVelRepr != m_frameVelRepr)
    {
        m_areBiasAccelerationsUpdated = false;
    }

    m_frameVelRepr = frameVelRepr;
    return true;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
bool KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::setRobotState(
    const SE3s& world_H_base,
    const Eigen::Ref<const VectorXs>& joint_pos,
    const Eigen::Ref<const Vector6s>& base_velocity,
    const Eigen::Ref<const VectorXs>& joint_vel,
    const Eigen::Ref<const Vector3s>& world_gravity)
{
    m_isStateSet = true;

    this->invalidateCache();

    if (joint_pos.size() != m_idyntreeModel.getNrOfPosCoords())
    {
        if (m_verbose)
        {
            std::cerr << "iDynFor::KinDynComputationsTpl wrong size of joint_pos argument "
                         "(required: "
                      << m_idyntreeModel.getNrOfPosCoords() << ", got: " << joint_pos.size() << ")"
                      << std::endl;
        }
        return false;
    }

    if (joint_vel.size() != m_idyntreeModel.getNrOfDOFs())
    {
        if (m_verbose)
        {
            std::cerr << "iDynFor::KinDynComputationsTpl wrong size of joint_vel argument "
                         "(required: "
                      << m_idyntreeModel.getNrOfDOFs() << ", got: " << joint_vel.size() << ")"
                      << std::endl;
        }
        return false;
    }

    // Save pos
    m_world_H_base = world_H_base;
    m_joint_pos = joint_pos;

    // Save vel
    m_base_velocity = base_velocity;
    m_joint_vel = joint_vel;

    // Save gravity
    m_world_gravity = world_gravity;

    return true;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
bool KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::getRobotState(
    SE3s& world_H_base,
    Eigen::Ref<VectorXs> joint_pos,
    Eigen::Ref<Vector6s> base_velocity,
    Eigen::Ref<VectorXs> joint_vel,
    Eigen::Ref<Vector3s> world_gravity) const
{
    world_H_base = m_world_H_base;
    base_velocity = m_base_velocity;
    return this->getRobotState(joint_pos, joint_vel, world_gravity);
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
bool KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::getRobotState(
    Eigen::Ref<VectorXs> joint_pos,
    Eigen::Ref<VectorXs> joint_vel,
    Eigen::Ref<Vector3s> world_gravity) const
{
    if (joint_pos.size() != m_idyntreeModel.getNrOfDOFs())
    {
        std::cerr << "iDynFor::KinDynComputationsTpl wrong size of joint_pos argument "
                     "(required: "
                  << m_idyntreeModel.getNrOfDOFs() << ", got: " << joint_pos.size() << ")"
                  << std::endl;
        return false;
    }

    if (joint_vel.size() != m_idyntreeModel.getNrOfDOFs())
    {
        std::cerr << "iDynFor::KinDynComputationsTpl wrong size of joint_vel argument "
                     "(required: "
                  << m_idyntreeModel.getNrOfDOFs() << ", got: " << joint_vel.size() << ")"
                  << std::endl;
        return false;
    }

    joint_pos = m_joint_pos;
    joint_vel = m_joint_vel;
    world_gravity = m_world_gravity;

    return true;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
bool KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::setRobotAcceleration(
    const Eigen::Ref<const Vector6s>& base_acceleration,
    const Eigen::Ref<const VectorXs>& joint_acceleration)
{
    if (joint_acceleration.size() != m_idyntreeModel.getNrOfDOFs())
    {
        if (m_verbose)
        {
            std::cerr << "iDynFor::KinDynComputationsTpl wrong size of joint_acceleration argument "
                         "(required: "
                      << m_idyntreeModel.getNrOfDOFs() << ", got: " << joint_acceleration.size() << ")"
                      << std::endl;
        }
        return false;
    }

    m_isAccelerationSet = true;

    this->invalidateCache();

    // Save acceleration
    m_base_acceleration = base_acceleration;
    m_joint_acceleration = joint_acceleration;

    return true;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
bool KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::getRobotAcceleration(
    Eigen::Ref<Vector6s> base_acceleration,
    Eigen::Ref<VectorXs> joint_acceleration) const
{
    if (joint_acceleration.size() != m_idyntreeModel.getNrOfDOFs())
    {
        std::cerr << "iDynFor::KinDynComputationsTpl wrong size of joint_acceleration argument "
                     "(required: "
                  << m_idyntreeModel.getNrOfDOFs() << ", got: " << joint_acceleration.size() << ")"
                  << std::endl;
        return false;
    }

    base_acceleration = m_base_acceleration;
    joint_acceleration = m_joint_acceleration;

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
    if (!m_isStateSet)
    {
        if (m_verbose)
        {
            std::cerr << "Error: iDynFor::KinDynComputationsTpl::getWorldTransform called before any call to setRobotState "
                      << std::endl;
        }
        return false;
    }

    this->computeFwdKinematics();

    // Convert iDynTree::FrameIndex to pinocchio::FrameIndex
    // TODO(traversaro): also support additional frames, not only frames associated to a link
    pinocchio::FrameIndex pinFrameIndex
        = m_idyntreeFrameIndex2PinocchioFrameIndex[frameIndex];

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

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
bool KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::getFrameVel(
    const iDynTree::FrameIndex frameIndex, Vector6s& frameVel)
{
    if (!m_isStateSet)
    {
        if (m_verbose)
        {
            std::cerr << "Error: iDynFor::KinDynComputationsTpl::getFrameVel called before any call to setRobotState "
                      << std::endl;
        }
        return false;
    }

    this->computeFwdKinematics();

    // Convert iDynTree::FrameIndex to pinocchio::FrameIndex
    pinocchio::FrameIndex pinFrameIndex
        = m_idyntreeFrameIndex2PinocchioFrameIndex[frameIndex];

    frameVel = pinocchio::getFrameVelocity(m_pinModel,
                                           m_pinData,
                                           pinFrameIndex,
                                           toPinocchio(m_frameVelRepr))
                   .toVector();

    return true;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
bool KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::getFrameFreeFloatingJacobian(const iDynTree::FrameIndex frameIndex,
                                      Matrix6Xs& outJacobian)
{
    if (!m_isStateSet)
    {
        if (m_verbose)
        {
            std::cerr << "Error: iDynFor::KinDynComputationsTpl::getFrameFreeFloatingJacobian called before any call to setRobotState "
                      << std::endl;
        }
        return false;
    }


    if (!m_idyntreeModel.isValidFrameIndex(frameIndex))
    {
        iDynTree::reportError("iDynFor::KinDynComputationsTpl",
                              "getFrameFreeFloatingJacobian","Frame index out of bounds");
        return false;
    }

    bool ok = (outJacobian.rows() == 6)
        && (outJacobian.cols() == m_idyntreeModel.getNrOfDOFs() + 6);

    if (!ok)
    {
        iDynTree::reportError("iDynFor::KinDynComputationsTpl",
                    "getFrameFreeFloatingJacobian",
                    "Wrong size in input outJacobian");
        return false;
    }

    this->computeFwdKinematics();

    // Convert iDynTree::FrameIndex to pinocchio::FrameIndex
    // TODO(traversaro): cache this information, there is no need to do a string search every time
    pinocchio::FrameIndex pinFrameIndex
        = m_idyntreeFrameIndex2PinocchioFrameIndex[frameIndex];

    // Compute Jacobian that on left has the right representation (as we pass it via toPinocchio(m_frameVelRepr))
    // but on the left accepts v^{pin} and not v^{idyn} (as defined in doc/theory_background.md)
    m_bufJacobian.setZero();
    pinocchio::computeFrameJacobian(m_pinModel,
                                    m_pinData,
                                    m_pin_model_position,
                                    pinFrameIndex,
                                    toPinocchio(m_frameVelRepr),
                                    m_bufJacobian);

    // Transform the left side from accepting v^{pin} to v^{idyn}
    convertLeftSideOfJacobianMatrixFromPinocchioToiDynTree(m_bufJacobian, outJacobian);

    return true;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
bool KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::getFrameAcc(
    const iDynTree::FrameIndex frameIndex, Eigen::Ref<Vector6s> frameAcc)
{
    if (!m_isStateSet)
    {
        if (m_verbose)
        {
            std::cerr << "Error: iDynFor::KinDynComputationsTpl::getFrameAcc called before any call to setRobotState "
                      << std::endl;
        }
        return false;
    }

    if (!m_isAccelerationSet)
    {
        if (m_verbose)
        {
            std::cerr << "Error: iDynFor::KinDynComputationsTpl::getFrameAcc called before any call to setRobotAcceleration "
                      << std::endl;
        }
        return false;
    }

    this->computeFwdKinematics();

    pinocchio::FrameIndex pinFrameIndex
        = m_idyntreeFrameIndex2PinocchioFrameIndex[frameIndex];

    // See section "Frame Acceleration" of doc/theory_background.md for details
    if (iDynTree::MIXED_REPRESENTATION == m_frameVelRepr)
    {
        frameAcc = pinocchio::getFrameClassicalAcceleration(m_pinModel,
                                                            m_pinData,
                                                            pinFrameIndex,
                                                            toPinocchio(m_frameVelRepr))
                       .toVector();
    } else
    {
        assert(iDynTree::INERTIAL_FIXED_REPRESENTATION == m_frameVelRepr
               || iDynTree::BODY_FIXED_REPRESENTATION == m_frameVelRepr);
        frameAcc = pinocchio::getFrameAcceleration(m_pinModel,
                                                   m_pinData,
                                                   pinFrameIndex,
                                                   toPinocchio(m_frameVelRepr))
                       .toVector();
    }

    return true;
}

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
bool KinDynComputationsTpl<Scalar, Options, JointCollectionTpl>::getFreeFloatingMassMatrix(MatrixXs& freeFloatingMassMatrix)
{
    if (!m_isStateSet)
    {
        if (m_verbose)
        {
            std::cerr << "Error: iDynFor::KinDynComputationsTpl::getFreeFloatingMassMatrix called before any call to setRobotState "
                      << std::endl;
        }
        return false;
    }

    this->convertModelStateFromiDynTreeToPinocchio();

    // Compute upper triangular part of the mass matrix
    pinocchio::crba(m_pinModel, m_pinData, m_pin_model_position);

    // Compute the full mass matrix
    m_pinData.M.template triangularView<Eigen::StrictlyLower>() = m_pinData.M.transpose().template triangularView<Eigen::StrictlyLower>();

    // Convert to idyntree formalism
    // M^{idyntree} = T^T M^{pin} T
    // decomposed as
    // buf = M^{pin} T
    this->convertLeftSideOfMassMatrixFromPinocchioToiDynTree(m_pinData.M, m_bufMassMatrix);
    // M^{idyntree} = T^T buf
    this->convertRightSideOfMassMatrixFromPinocchioToiDynTree(m_bufMassMatrix, freeFloatingMassMatrix);

    return true;
}


} // namespace iDynFor
