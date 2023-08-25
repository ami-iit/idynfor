// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNFOR_KINDYNCOMPUTATIONS_H
#define IDYNfOR_KINDYNCOMPUTATIONS_H

#include <memory>
#include <vector>

#include <iDynTree/Model/Model.h>
// This header provides the iDynTree::FrameVelocityRepresentation enum
#include <iDynTree/Model/FreeFloatingMatrices.h>

// TODO: Should we include pinocchio types in the public API?
// for now yes, as it simplifies implementation, in future
// we can look at something more fancy to re-introduce some form
// of pimpl
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace iDynFor
{

/*
 * \brief High level stateful class wrapping several kinematics and dynamics algorithms.
 *
 * The kinematics dynamics computations class is an high level class stateful to access
 * several algorithms related to kinematics and dynamics of free floating robot systems.
 *
 * The interface of the class is inspired and meant to be compatible with
 * the iDynTree::KinDynComputations class.
 *
 *
 */
template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
struct KinDynComputationsTpl
{
public:
    typedef pinocchio::SE3Tpl<Scalar, Options> SE3s;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> VectorXs;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options> MatrixXs;
    typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3s;
    typedef Eigen::Matrix<Scalar, 6, 1, Options> Vector6s;
    typedef Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options> Matrix6Xs;

private:
    // Internal Class State

    // iDynTree model
    iDynTree::Model m_idyntreeModel;
    // iDynTree::FrameVelocityRepresentation used
    // Many quantity are defined in a frame that depends on the used FrameVelocityRepresentation,
    // In particular we call velReprFrame this frame, that has value:
    // iDynTree::MIXED_REPRESENTATION : B[A]
    // iDynTree::INERTIAL_FIXED_FRAME : A
    // iDynTree::BODY_FIXED_FRAME : B
    // Where A is the absolute/world/universe frame, and B is the base link frame
    iDynTree::FrameVelocityRepresentation m_frameVelRepr = iDynTree::MIXED_REPRESENTATION;
    // Pinocchio model
    pinocchio::ModelTpl<Scalar, Options, JointCollectionTpl> m_pinModel;
    // Pinocchio data
    pinocchio::DataTpl<Scalar, Options, JointCollectionTpl> m_pinData;

    bool m_modelLoaded = false;

    // MultiBody Model State (iDynTree-formalism)

    // A: absolute/world frame
    // B: base frame
    // Base Position: {}^A H_B
    SE3s m_world_H_base;
    // Internal joint positions: s
    VectorXs m_joint_pos;
    // Base Velocity: {}^{velReprFrame} \mathrm{v}_{A,B} with linear/angular serialization
    Vector6s m_base_velocity;
    // Internal joint velocities: \dot{s}
    VectorXs m_joint_vel;
    // Base Acceleration: {}^{velReprFrame} \dot{\mathrm{v}}_{A,B} with linear/angular serialization
    Vector6s m_base_acceleration;
    // Internal joint accelerations: \ddot{s}
    VectorXs m_joint_acceleration;
    // Gravity expressed in absolute frame: {}^A g
    Vector3s m_world_gravity;

    // MultiBody Model State (pinocchio-formalism)

    // Base and internal joint position (q)
    VectorXs m_pin_model_position;

    // Base and internal joint velocities
    VectorXs m_pin_model_velocity;

    // Base and internal joint accelerations
    VectorXs m_pin_model_acceleration;

    // Conversion-related quantities
    std::vector<size_t> m_idyntreeDOFOffset2PinocchioJointPosOffset;
    std::vector<size_t> m_idyntreeDOFOffset2PinocchioJointVelOffset;
    std::vector<pinocchio::FrameIndex> m_idyntreeFrameIndex2PinocchioFrameIndex;

    // Permutation matrix, see "Model Position" section of doc/theory_background.md
    Eigen::PermutationMatrix<Eigen::Dynamic> m_pinocchio_P_idyntree;

    // Enable printed error messages
    bool m_verbose = true;

    // Cache-related flags methods
    bool m_isStateSet = false;
    bool m_isAccelerationSet = false;
    bool m_isConvertedStateUpdated = false;
    bool m_isConvertedAccelerationUpdated = false;
    bool m_isFwdKinematicsUpdated = false;
    bool m_areBiasAccelerationsUpdated = false;

    // Buffer for Jacobian-shaped quantities
    Matrix6Xs m_bufJacobian;

    // Buffer for MassMatrix-shaped quantities
    MatrixXs m_bufMassMatrix;

    // Invalidate the cache of intermediate results (called by setRobotState)
    void invalidateCache();

    // Compute forward kinematics, if required
    void computeFwdKinematics();

    // Get B_H_velReprFrame transform, where B is the base frame,
    // A is the absolute/world/universe frame and velReprFrame is:
    // velReprFrame = B[A] if m_frameVelRepr == iDynTree::MIXED_REPRESENTATION
    // velReprFrame = B if m_frameVelRepr == iDynTree::BODY_FIXED_REPRESENTATION
    // velReprFrame = A if m_frameVelRepr == iDynTree::INERTIAL_FIXED_REPRESENTATION
    SE3s getBaseHVelReprFrameTransform();

    Vector6s getBaseVelocityInBodyFixed();
    Vector6s getBaseAccelerationInBodyFixed();

    // Convert model state from iDynTree formalism to pinocchio formalism
    void convertModelStateFromiDynTreeToPinocchio();

    // Convert model acceleration from iDynTree formalism to pinocchio formalism
    void convertModelAccelerationFromiDynTreeToPinocchio();

    // Convert left-side of a jacobian matrix from accepting Pinocchio velocity to accepting iDynTree velocities
    void convertLeftSideOfJacobianMatrixFromPinocchioToiDynTree(const Matrix6Xs& pinocchioMatrixOnTheLeft,
                                                                      Matrix6Xs& idyntreeMatrixOnTheLeft);

    // Convert left-side of a mass matrix matrix from accepting Pinocchio velocity to accepting iDynTree velocities
    void convertLeftSideOfMassMatrixFromPinocchioToiDynTree(const MatrixXs& pinocchioMatrixOnTheLeft,
                                                                  MatrixXs& idyntreeMatrixOnTheLeft);

    // Convert right-side of a mass matrix matrix from returning Pinocchio generalized torques to accepting iDynTree generalized torques
    void convertRightSideOfMassMatrixFromPinocchioToiDynTree(const MatrixXs& pinocchioMatrixOnTheRight,
                                                                   MatrixXs& idyntreeMatrixOnTheRight);


public:
    /**
     * Constructor.
     */
    KinDynComputationsTpl()
    {
    }

    /**
     * Destructor.
     */
    virtual ~KinDynComputationsTpl()
    {
    }

    /**
     * Load the multibody model from a iDynTree::Model class.
     *
     * @param model the model to use in this class.
     * @return true if all went ok, false otherwise.
     */
    bool loadRobotModel(const iDynTree::Model& model);

    /**
     * Return true if the models for the robot have been correctly loaded.
     *
     * @return True if the class has been correctly configured, false otherwise.
     */
    bool isValid() const;

    /**
     * Get loaded model.
     */
    const iDynTree::Model& model() const;

    /**
     * Get loaded model.
     */
    const iDynTree::Model& getRobotModel() const;

    /**
     * @brief Set the used FrameVelocityRepresentation.
     *
     * @remark See iDynTree::FrameVelocityRepresentation documentation for more details.
     */
    bool setFrameVelocityRepresentation(const iDynTree::FrameVelocityRepresentation);

    /**
     * @brief Get the used FrameVelocityRepresentation.
     *
     * @remark See iDynTree::FrameVelocityRepresentation documentation for more details.
     */
    iDynTree::FrameVelocityRepresentation getFrameVelocityRepresentation() const;

    /**
     * Set the state for the robot (floating base)
     *
     * @param world_H_base  the homogeneous transformation that transforms position vectors
     * expressed in the base reference frame in position frames expressed in the world reference
     * frame (i.e. pos_world = world_H_base*pos_base .
     * @param s a vector of getNrOfDegreesOfFreedom() joint positions (in rad)
     * @param base_velocity The twist (linear/angular velocity) of the base, expressed with the
     * convention specified by the used FrameVelocityConvention.
     * @param s_dot a vector of getNrOfDegreesOfFreedom() joint velocities (in rad/sec)
     * @param world_gravity a 3d vector of the gravity acceleration vector, expressed in the
     * world/inertial frame.
     * @return true if all went well, false otherwise.
     */
    bool setRobotState(const SE3s& world_H_base,
                       const Eigen::Ref<const VectorXs>& joint_pos,
                       const Eigen::Ref<const Vector6s>& base_velocity,
                       const Eigen::Ref<const VectorXs>& joint_vel,
                       const Eigen::Ref<const Vector3s>& world_gravity);

    /**
     * Get the state for the robot (floating base)
     *
     * @param world_H_base  the homogeneous transformation that transforms position vectors
     * expressed in the base reference frame in position frames expressed in the world reference
     * frame (i.e. pos_world = world_H_base*pos_base .
     * @param s a vector of getNrOfDegreesOfFreedom() joint positions (in rad)
     * @param base_velocity The twist (linear/angular velocity) of the base, expressed with the
     * convention specified by the used FrameVelocityConvention.
     * @param s_dot a vector of getNrOfDegreesOfFreedom() joint velocities (in rad/sec)
     * @param world_gravity a 3d vector of the gravity acceleration vector, expressed in the
     * world/inertial frame.
     * @return true if all went well, false otherwise.
     */
    bool getRobotState(SE3s& world_H_base,
                       Eigen::Ref<VectorXs> s,
                       Eigen::Ref<Vector6s> base_velocity,
                       Eigen::Ref<VectorXs> s_dot,
                       Eigen::Ref<Vector3s> world_gravity) const;

    /**
     * Get the state for the robot (floating base)
     *
     * @param s a vector of getNrOfDegreesOfFreedom() joint positions (in rad)
     * @param s_dot a vector of getNrOfDegreesOfFreedom() joint velocities (in rad/sec)
     * @param world_gravity a 3d vector of the gravity acceleration vector, expressed in the
     * world/inertial frame.
     * @return true if all went well, false otherwise.
     */
    bool getRobotState(Eigen::Ref<VectorXs> s,
                       Eigen::Ref<VectorXs> s_dot,
                       Eigen::Ref<Vector3s> world_gravity) const;

    /**
     * Set the acceleration for the robot (floating base)
     *
     * @param base_acceleration The twist (linear/angular velocity) of the base, expressed with the
     * convention specified by the used FrameVelocityConvention.
     * @param joint_acceleration a vector of getNrOfDegreesOfFreedom() joint velocities (in rad/sec^2)
     *
     * @return true if all went well, false otherwise.
     */
    bool setRobotAcceleration(const Eigen::Ref<const Vector6s>& base_acceleration,
                              const Eigen::Ref<const VectorXs>& joint_acceleration);

    /**
     * Get the acceleration for the robot (floating base)
     *
     * @param base_acceleration The twist (linear/angular velocity) of the base, expressed with the
     * convention specified by the used FrameVelocityConvention.
     * @param joint_acceleration a vector of getNrOfDegreesOfFreedom() joint velocities (in rad/sec^2)
     *
     * @return true if all went well, false otherwise.
     */
    bool getRobotAcceleration(Eigen::Ref<Vector6s> base_acceleration,
                              Eigen::Ref<VectorXs> joint_acceleration) const;

    /**
     * Get the index corresponding to a given frame name.
     * @return a integer greater than or equal to zero if the frame exist,
     *         a negative integer otherwise.
     */
    int getFrameIndex(const std::string& frameName) const;

    /**
     * Get the frame name corresponding to a given frame index.
     *
     */
    std::string getFrameName(const iDynTree::FrameIndex frameIndex) const;

    /**
     * Return the transform where the frame is the frame
     * specified by frameIndex, and the reference frame is the world one
     * (world_H_frame).
     * @param world_H_frame a 4x4 matrix representing the homogeneous transformation that transforms
     * position vectors expressed in the 'frame' reference frame in position frames expressed in the
     * world reference frame (i.e. pos_world = world_H_frame * pos_frame).
     * @return true if all went well, false otherwise.
     */
    bool getWorldTransform(const iDynTree::FrameIndex frameIndex, SE3s& world_H_frame);

    /**
     * @name Methods to get frame velocity information given the current state.
     */
    //@{

    /**
     * Return the frame velocity, with the convention specified by getFrameVelocityRepresentation
     * and linear/angular serialization.
     *
     * See section "Frame velocity" of doc/theory_background.md document for more details.
     */
    bool getFrameVel(const iDynTree::FrameIndex frameIdx, Vector6s& frameVel);

    /**
     * Compute the free floating jacobian for a given frame with the convention specified by
     * getFrameVelocityRepresentation.
     *
     * In particular, the matrix returned by this method, if moltiplied by the vector obtained
     * stacking base_velocity and s_dot (setRobotState arguments), returns the output of
     * getFrameVel.
     *
     * @return true if all went well, false otherwise.
     */
    bool
    getFrameFreeFloatingJacobian(const iDynTree::FrameIndex frameIndex, Matrix6Xs& outJacobian);

    //@}

    /**
     * @name Methods to get frame acceleration information given the current state and acceleration.
     */
    //@{

    /**
     * Return the frame velocity, with the convention specified by getFrameVelocityRepresentation
     * and linear/angular serialization.
     *
     * See section "Frame Acceleration" of doc/theory_background.md document for more details.
     */
    bool getFrameAcc(const iDynTree::FrameIndex frameIdx, Eigen::Ref<Vector6s> frameAcc);

    //@}

    /**
      * @name Methods to get quantities related to unconstrained free floating equation of motions.
      *
      * This methods permits to compute several quantities related to free floating equation of methods.
      * Note that this equations needs to be coupled with a description of the interaction between the model
      * and the enviroment (such as a contant model, a bilateral constraint on some links or by considering
      * some external forces as inputs) to actually obtain a dynamical system description of the mechanical model evolution.
      *
      * The equations of motion of a free floating mechanical system under the effect of a uniform gravitational field are:
      * \f[
      * M(q) \dot{\nu} +
      * C(q, \nu) \nu +
      * G(q)
      * =
      * \begin{bmatrix}
      * 0_{6\times1} \newline
      * \tau
      * \end{bmatrix}
      * +
      * \sum_{L \in \mathcal{L}}
      * J_L^T \mathrm{f}_L^x
      * \f]
      *
      * where:
      *
      * * \f$n_{PC}\f$ is the value returned by Model::getNrOfPosCoords,
      * * \f$n_{DOF}\f$ is the value returned by Model::getNrOfDOFs,
      * * \f$n_{L}\f$ is the value returned by Model::getNrOfLinks,
      * * \f$q \in \mathbb{R}^3 \times \textrm{SO}(3) \times \mathbb{R}^{n_{PC}}\f$ is the robot position,
      * * \f$\nu \in \mathbb{R}^{6+n_{DOF}}\f$ is the robot velocity,
      * * \f$\dot{\nu} \in \mathbb{R}^{6+n_{DOF}}\f$ is the robot acceleration,
      * * \f$M(q) \in \mathbb{R}^{(6+n_{DOF}) \times (6+n_{DOF})}\f$ is the free floating mass matrix,
      * * \f$C(q, \nu)  \in \mathbb{R}^{(6+n_{DOF}) \times (6+n_{DOF})}\f$ is the coriolis matrix,
      * * \f$G(q) \in \mathbb{R}^{6+n_{DOF}}\f$ is the vector of gravity generalized forces,
      * * \f$\tau \in \mathbb{R}^6\f$ is the vector of torques applied on the joint of the multibody model,
      * * \f$\mathcal{L}\f$ is the set of all the links contained in the multibody model,
      * * \f$J_L \in \mathbb{R}^{6+n_{DOF}}\f$ is the free floating jacobian of link \f$L\f$ as obtained by KinDynComputations::getFrameFreeFloatingJacobian,
      * * \f$\mathrm{f}_L^x\f$ is the 6D force/torque applied by the enviroment on link \f$L\f$.
      *
      * The precise definition of each quantity (in particular the part related to the base) actually depends on the
      * choice of FrameVelocityRepresentation, specified with the setFrameVelocityRepresentation method.
      *
      */
    //@{

    /**
     * @brief Get the free floating mass matrix of the system.
     *
     * This method computes \f$M(q) \in \mathbb{R}^{(6+n_{DOF}) \times (6+n_{DOF})}\f$.
     *
     * The mass matrix depends on the joint positions, specified by the setRobotState methods.
     * If the chosen FrameVelocityRepresentation is MIXED_REPRESENTATION or INERTIAL_FIXED_REPRESENTATION,
     * the mass matrix depends also on the base orientation with respect to the inertial frame,
     * that is also set by the setRobotState methods.
     *
     * For more details on the structure of the free floating mass matrix, please check:
     * S. Traversaro, A. Saccon
     * Multibody Dynamics Notation
     * http://repository.tue.nl/849895
     *
     * @param[out] freeFloatingMassMatrix the (6+getNrOfDOFs()) times (6+getNrOfDOFs()) output mass matrix.
     * @return true if all went well, false otherwise.
     */
    bool getFreeFloatingMassMatrix(MatrixXs& freeFloatingMassMatrix);


    /**
     * @brief Compute the free floating inverse dynamics.
     *
     * This method computes \f$M(q) \dot{\nu} + C(q, \nu) \nu + G(q) - \sum_{L \in \mathcal{L}} J_L^T \mathrm{f}_L^x \in \mathbb{R}^{6+n_{DOF}}\f$.
     *
     * The semantics of baseAcc, the base part of baseForceAndJointTorques
     * and of the elements of linkExtWrenches depend of the chosen FrameVelocityRepresentation .
     *
     * The state is the one given set by the setRobotState method, and the acceleration is the one set by the setRobotAcceleration method.
     *
     * @param[in] linkExtForces the external wrenches excerted by the environment on the model
     * @param[out] baseForceAndJointTorques the output generalized torques
     * @return true if all went well, false otherwise
     */
    bool inverseDynamics(const LinkNetExternalWrenches & linkExtForces,
                               Eigen::Ref<VectorXs> baseForceAndJointTorques);

    /**
     * @brief Compute the getNrOfDOFS()+6 vector of generalized bias (gravity+coriolis) forces.
     *
     * This method computes \f$C(q, \nu) \nu + G(q) \in \mathbb{R}^{6+n_{DOF}}\f$.
     *
     * The semantics of the base part of generalizedBiasForces depend of the chosen FrameVelocityRepresentation .
     *
     * The state is the one given set by the setRobotState method.
     *
     * @param[out] generalizedBiasForces the output generalized bias forces
     * @return true if all went well, false otherwise
     */
    bool generalizedBiasForces(Eigen::Ref<VectorXs> generalizedBiasForces);


    /**
     * @brief Compute the getNrOfDOFS()+6 vector of generalized gravity forces.
     *
     * This method computes \f$G(q) \in \mathbb{R}^{6+n_{DOF}}\f$.
     *
     * The semantics of the base part of generalizedGravityForces depend of the chosen FrameVelocityRepresentation .
     *
     * The state is the one given set by the setRobotState method.
     *
     * @note generalizedGravityForces has to be a (6 + dofs)-d vector. The first 6 elements will
     * contain the bias forces related to the system base, while the last dofs elements
     * related to the joints.
     *
     * @warning the Span objects should point an already existing memory. Memory allocation and resizing cannot be achieved with this kind of objects.
     * @return true if all went well, false otherwise.
     */
    bool generalizedGravityForces(Eigen::Ref<VectorXs> generalizedGravityForces);

    /**
     * @brief Compute the getNrOfDOFS()+6 vector of generalized external forces.
     *
     * This method computes \f$ -\sum_{L \in \mathcal{L}} J_L^T \mathrm{f}_L^x \in \mathbb{R}^{6+n_{DOF}} \f$.
     *
     * @warning Note that this method returns the **negated** sum of the product of jacobian and the external force,
     *          consistently with how the generalized external forces are computed in the KinDynComputations::inverseDynamics method.
     *
     * The semantics of the base part of generalizedExternalForces
     * and of the elements of linkExtWrenches depend of the chosen FrameVelocityRepresentation .
     *
     * The state is the one given set by the setRobotState method.
     *
     * @param[out] generalizedExternalForces the output external generalized forces
     * @return true if all went well, false otherwise
     */
    bool generalizedExternalForces(const LinkNetExternalWrenches & linkExtForces,
                                         Eigen::Ref<VectorXs> & generalizedExternalForces);

    //@}


};

} // namespace iDynFor

#include <iDynFor/KinDynComputations.tpp>

#endif
