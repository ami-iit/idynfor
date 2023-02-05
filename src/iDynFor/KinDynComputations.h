/*
 * SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef IDYNFOR_KINDYNCOMPUTATIONS_H
#define IDYNfOR_KINDYNCOMPUTATIONS_H

#include <memory>

#include <iDynTree/Model/Model.h>

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
class KinDynComputations
{
private:
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;

public:
    /**
     * Constructor.
     */
    KinDynComputations();

    /**
     * Destructor.
     */
    virtual ~KinDynComputations();

    /**
     * Load the multibody model from a iDynTree::Model class.
     *
     * @param model the model to use in this class.
     * @return true if all went ok, false otherwise.
     */
    bool loadRobotModel(const iDynTree::Model & model);

    /**
     * Return true if the models for the robot have been correctly loaded.
     *
     * @return True if the class has been correctly configured, false otherwise.
     */
    bool isValid() const;

    /**
     * Get loaded model.
     */
    const iDynTree::Model & model() const;

    /**
     * Get loaded model.
     */
    const iDynTree::Model & getRobotModel() const;
};

}

#endif
