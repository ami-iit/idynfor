/*
 * SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia
 * SPDX-License-Identifier: BSD-3-Clause
 */

// pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/spatial/inertia.hpp>

// iDynTree
#include <iDynTree/Core/SpatialInertia.h>
#include <iDynTree/Model/Model.h>

namespace iDynFor
{
namespace details
{

// All this part is taken from
// https://github.com/stack-of-tasks/pinocchio/blob/v2.6.8/src/parsers/urdf/model.hxx Unfortunatly,
// it is not public, so it can't be reused
template <typename _Scalar, int Options> class iDynTreeModelVisitorBaseTpl
{
public:
    enum JointType
    {
        REVOLUTE,
        CONTINUOUS,
        PRISMATIC,
        FLOATING,
        PLANAR
    };
    typedef _Scalar Scalar;
    typedef pinocchio::SE3Tpl<Scalar, Options> SE3;
    typedef pinocchio::InertiaTpl<Scalar, Options> Inertia;
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;
    typedef Eigen::Ref<Vector> VectorRef;
    typedef Eigen::Ref<const Vector> VectorConstRef;
    virtual void setName(const std::string& name) = 0;
    virtual void addRootJoint(const Inertia& Y, const std::string& body_name) = 0;
    virtual void addFixedJointAndBody(const pinocchio::FrameIndex& parentFrameId,
                                      const pinocchio::SE3& joint_placement,
                                      const std::string& joint_name,
                                      const pinocchio::Inertia& Y,
                                      const std::string& body_name)
        = 0;
    iDynTreeModelVisitorBaseTpl()
        : log(NULL)
    {
    }
    template <typename T> iDynTreeModelVisitorBaseTpl& operator<<(const T& t)
    {
        if (log != NULL)
            *log << t;
        return *this;
    }
    std::ostream* log;
};

template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
class iDynTreeModelVisitor : public iDynTreeModelVisitorBaseTpl<Scalar, Options>
{
public:
    typedef iDynTreeModelVisitorBaseTpl<Scalar, Options> Base;
    typedef typename Base::JointType JointType;
    typedef typename Base::Vector3 Vector3;
    typedef typename Base::VectorConstRef VectorConstRef;
    typedef typename Base::SE3 SE3;
    typedef typename Base::Inertia Inertia;
    typedef pinocchio::ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef pinocchio::FrameTpl<Scalar, Options> Frame;
    Model& model;
    iDynTreeModelVisitor(Model& model)
        : model(model)
    {
    }
    void setName(const std::string& name)
    {
        model.name = name;
    }

    void appendBodyToJoint(
        const pinocchio::FrameIndex fid,
        const Inertia& Y,
        const SE3 & placement,
        const std::string & body_name)

    {
      const pinocchio::Frame & frame = model.frames[fid];
      const SE3 & p = frame.placement * placement;
      model.appendBodyToJoint(frame.parent, Y, p);
      model.addBodyFrame(body_name, frame.parent, p, (int)fid);

      // Reference to model.frames[fid] have changed because the vector
      // may have been reallocated.
    }

    virtual void addRootJoint(const Inertia& Y, const std::string& body_name)
    {
        // Inspired by https://github.com/stack-of-tasks/pinocchio/blob/v2.6.17/src/parsers/urdf/model.hxx#L343
        // The main difference is that iDynTree models always assume that the root joint is
        // always the floating one, so we hardcode the pinocchio::JointModelFreeFlyerTpl choice
        const pinocchio::Frame & frame = model.frames[0];

        // We give a name that starts with "idynfor_" to avoid collisions with a model
        // that already contains a joint named "root_joint"
        pinocchio::JointIndex idx = model.addJoint(frame.parent,
            pinocchio::JointModelFreeFlyer(),
            SE3::Identity(), "idynfor_root_joint");

        pinocchio::FrameIndex jointFrameId = model.addJointFrame(idx, 0);
        appendBodyToJoint(jointFrameId, Y, SE3::Identity(), body_name);
    }
    virtual void addFixedJointAndBody(const pinocchio::FrameIndex& parent_frame_id,
                                      const pinocchio::SE3& joint_placement,
                                      const std::string& joint_name,
                                      const pinocchio::Inertia& Y,
                                      const std::string& body_name)
    {
        const pinocchio::Frame& parent_frame = model.frames[parent_frame_id];
        const pinocchio::JointIndex parent_frame_parent = parent_frame.parent;

        const pinocchio::SE3 placement = parent_frame.placement * joint_placement;
        pinocchio::FrameIndex fid = model.addFrame(pinocchio::Frame(joint_name,
                                                                    parent_frame.parent,
                                                                    parent_frame_id,
                                                                    placement,
                                                                    pinocchio::FIXED_JOINT,
                                                                    Y));

        model.addBodyFrame(body_name, parent_frame_parent, placement, (int)fid);
    }

    virtual void addAdditionalFrame(const std::string& additionalFrameName,
                                    const std::string& parentLinkName,
                                    const pinocchio::SE3& link_H_additionalframe)
    {
        pinocchio::FrameIndex parentLinkIndex = model.getBodyId(parentLinkName);
        pinocchio::JointIndex parentJointIndex = model.frames[parentLinkIndex].parent;
        pinocchio::SE3 joint_H_link = model.frames[parentLinkIndex].placement;
        pinocchio::SE3 joint_H_additionalframe = joint_H_link*link_H_additionalframe;
        // The previousFrame attribute is only used to distinguish the interconnection
        // of multiple frames that are rigidly interconnected to the same joint,
        // see https://github.com/stack-of-tasks/pinocchio/blob/78d62096002ffa3790638e392f0b6e4a5efc3d34/src/algorithm/frames.hxx#L280
        // In this case we are building a Frame with zero inertia, so there previousFrame does not have any effect,
        // anyhow for consistency we mark the parentLinkIndex as previousFrame
        model.addFrame(Frame(additionalFrameName, parentJointIndex, parentLinkIndex, joint_H_additionalframe, pinocchio::OP_FRAME));
    }
};

typedef iDynTreeModelVisitorBaseTpl<double, 0> iDynTreeModelVisitorBase;
} // namespace details

/**
 * Convert an iDynTree::Transform to a pinocchio::SE3
 */
pinocchio::SE3 toPinocchio(const iDynTree::Transform& transform_idyntree);

/**
 * Convert a pinocchio::SE3 to iDynTree::Transform
 */
iDynTree::Transform fromPinocchio(const pinocchio::SE3& se3_pinocchio);

/**
 * Convert an iDynTree::SpatialInertia to a pinocchio::Inertia
 */
pinocchio::Inertia toPinocchio(const iDynTree::SpatialInertia& inertiaIDynTree);

/**
 * \brief Build the pinocchio model from a iDynTree::Model .
 *
 * \param[in] model The iDynTree::Model to load.
 * \param[in] verbose Print parsing info.
 * \param[out] model Reference model where to put the parsed information.
 * \return Return the reference on argument model for convenience.
 *
 * The signature of this function is inspired from the pinocchio::buildModel function
 */
template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
pinocchio::ModelTpl<Scalar, Options, JointCollectionTpl>&
buildPinocchioModelfromiDynTree(const iDynTree::Model& modelIDynTree,
                                pinocchio::ModelTpl<Scalar, Options, JointCollectionTpl>& modelPin,
                                const bool verbose = false)
{
    if (modelIDynTree.getNrOfLinks() > 1)
    {
        const std::string exception_message("iDynTree::Model has more than one link, only one link "
                                            "models are supported for now.");
        throw std::invalid_argument(exception_message);
    }

    // Build visitior
    iDynFor::details::iDynTreeModelVisitor<Scalar, Options, JointCollectionTpl> visitor(modelPin);

    // Once iDynTree::Model has a name pass the name along
    // See https://github.com/robotology/idyntree/issues/908
    visitor.setName("iDynForModel");

    // Extract root link from iDynTree
    iDynTree::LinkConstPtr defaultBaseLink
        = modelIDynTree.getLink(modelIDynTree.getDefaultBaseLink());
    std::string defaultBaseLinkName = modelIDynTree.getLinkName(modelIDynTree.getDefaultBaseLink());

    // Add root link
    visitor.addRootJoint(toPinocchio(defaultBaseLink->getInertia()), defaultBaseLinkName);

    // Add additional frames of root link
    std::vector<iDynTree::FrameIndex> localFrmIdxs;
    modelIDynTree.getLinkAdditionalFrames(modelIDynTree.getDefaultBaseLink(), localFrmIdxs);
    for(iDynTree::FrameIndex localFrmIdx: localFrmIdxs)
    {
        std::string additionalFrameName = modelIDynTree.getFrameName(localFrmIdx);
        iDynTree::Transform link_H_additionalFrame = modelIDynTree.getFrameTransform(localFrmIdx);
        visitor.addAdditionalFrame(additionalFrameName, defaultBaseLinkName,
                                   toPinocchio(link_H_additionalFrame));
    }

    return modelPin;
}

} // namespace iDynFor
