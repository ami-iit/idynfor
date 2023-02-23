/*
 * SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef IDYNFOR_KINDYNCOMPUTATIONS_H
#define IDYNfOR_KINDYNCOMPUTATIONS_H

#include <memory>

#include <iDynTree/Model/Model.h>

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
    typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3s;
    typedef Eigen::Matrix<Scalar, 6, 1, Options> Vector6s;


private:
    // Internal Class State

    // iDynTree model
    iDynTree::Model m_idyntreeModel;
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
    // Base Velocity: {}^B \mathrm{v}_{A,B}
    Vector6s m_base_velocity;
    // Internal joint velocities: \dot{s}
    VectorXs m_joint_vel;
    // Gravity expressed in absolute frame: {}^A g
    Vector3s m_world_gravity;

    // MultiBody Model State (pinocchio-formalism)

    // Base and internal joint position (q)
    VectorXs m_pin_model_position;

    // Cache-related flags methods
    bool m_isFwdKinematicsUpdated = false;

    // Invalidate the cache of intermediate results (called by setRobotState)
    void invalidateCache();

    // Compute forward kinematics, if required
    void computeFwdKinematics();

    // Convert model state from iDynTree formalism to pinocchio formalism
    void convertModelStateFromiDynTreeToPinocchio();

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
                       const VectorXs& s,
                       const Vector6s& base_velocity,
                       const VectorXs& v,
                       const Vector3s& world_gravity);

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
};

} // namespace iDynFor

#include <iDynFor/KinDynComputations.tpp>

#endif
