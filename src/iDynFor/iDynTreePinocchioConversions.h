/*
 * SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia
 * SPDX-License-Identifier: BSD-3-Clause
 */

// std
#include <deque>

// pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/multibody/joint/joint-collection.hpp>
#include <pinocchio/spatial/inertia.hpp>

// iDynTree
#include <iDynTree/Core/Axis.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/SpatialInertia.h>
#include <iDynTree/Model/FixedJoint.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/PrismaticJoint.h>
#include <iDynTree/Model/RevoluteJoint.h>
#include <iDynTree/Model/Traversal.h>

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
    };
    typedef _Scalar Scalar;
    typedef pinocchio::SE3Tpl<Scalar, Options> SE3;
    typedef pinocchio::InertiaTpl<Scalar, Options> Inertia;
    typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector;
    typedef Eigen::Ref<Vector> VectorRef;
    typedef Eigen::Ref<const Vector> VectorConstRef;
    virtual void setName(const std::string& name) = 0;
    virtual void addRootJoint(const Inertia& Y, const std::string& bodyName) = 0;
    virtual void addFixedJointAndBody(const pinocchio::FrameIndex& parentFrameId,
                                      const pinocchio::SE3& joint_placement,
                                      const std::string& joint_name,
                                      const pinocchio::Inertia& Y,
                                      const std::string& bodyName)
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
    typedef typename Model::JointCollection JointCollection;
    Model& model;
    iDynTreeModelVisitor(Model& model)
        : model(model)
    {
    }
    void setName(const std::string& name)
    {
        model.name = name;
    }

    void appendBodyToJoint(const pinocchio::FrameIndex fid,
                           const Inertia& Y,
                           const SE3& placement,
                           const std::string& bodyName)

    {
        const pinocchio::Frame& frame = model.frames[fid];
        const SE3& p = frame.placement * placement;
        model.appendBodyToJoint(frame.parent, Y, p);
        model.addBodyFrame(bodyName, frame.parent, p, static_cast<int>(fid));

        // Reference to model.frames[fid] have changed because the vector
        // may have been reallocated.
    }

    virtual void addRootJoint(const Inertia& Y, const std::string& bodyName)
    {
        // Inspired by
        // https://github.com/stack-of-tasks/pinocchio/blob/v2.6.17/src/parsers/urdf/model.hxx#L343
        // The main difference is that iDynTree models always assume that the root joint is
        // always the floating one, so we hardcode the pinocchio::JointModelFreeFlyerTpl choice
        const pinocchio::Frame& frame = model.frames[0];

        // We give a name that starts with "idynfor_" to avoid collisions with a model
        // that already contains a joint named "root_joint"
        pinocchio::JointIndex idx = model.addJoint(frame.parent,
                                                   pinocchio::JointModelFreeFlyer(),
                                                   SE3::Identity(),
                                                   "idynfor_root_joint");

        pinocchio::FrameIndex jointFrameId = model.addJointFrame(idx, 0);
        appendBodyToJoint(jointFrameId, Y, SE3::Identity(), bodyName);
    }

    virtual void addFixedJointAndBody(const pinocchio::FrameIndex& parent_frame_id,
                                      const pinocchio::SE3& joint_placement,
                                      const std::string& joint_name,
                                      const pinocchio::Inertia& Y,
                                      const std::string& bodyName)
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

        model.addBodyFrame(bodyName, parent_frame_parent, placement, (int)fid);
    }

    ///
    /// \brief The four possible cartesian types of an 3D axis.
    ///
    enum CartesianAxis
    {
        AXIS_X = 0,
        AXIS_Y = 1,
        AXIS_Z = 2,
        AXIS_UNALIGNED
    };

    ///
    /// \brief Extract the cartesian property of a particular 3D axis.
    ///
    /// \param[in] axis The input iDynTree axis.
    ///
    /// \return The property of the particular axis CartesianAxis.
    ///
    static inline CartesianAxis extractCartesianAxis(const Vector3& axis)
    {
        if (axis == Vector3(1., 0., 0.))
            return AXIS_X;
        else if (axis == Vector3(0., 1., 0.))
            return AXIS_Y;
        else if (axis == Vector3(0., 0., 1.))
            return AXIS_Z;
        else
            return AXIS_UNALIGNED;
    }

    template <typename TypeX, typename TypeY, typename TypeZ, typename TypeUnaligned>
    pinocchio::JointIndex addJoint(const Vector3& axis,
                                   const Frame& frame,
                                   const SE3& placement,
                                   const std::string& joint_name,
                                   const VectorConstRef& max_effort,
                                   const VectorConstRef& max_velocity,
                                   const VectorConstRef& min_config,
                                   const VectorConstRef& max_config,
                                   const VectorConstRef& friction,
                                   const VectorConstRef& damping)
    {
        CartesianAxis axisType = extractCartesianAxis(axis);
        switch (axisType)
        {
        case AXIS_X:
            return model.addJoint(frame.parent,
                                  TypeX(),
                                  frame.placement * placement,
                                  joint_name,
                                  max_effort,
                                  max_velocity,
                                  min_config,
                                  max_config,
                                  friction,
                                  damping);
            break;

        case AXIS_Y:
            return model.addJoint(frame.parent,
                                  TypeY(),
                                  frame.placement * placement,
                                  joint_name,
                                  max_effort,
                                  max_velocity,
                                  min_config,
                                  max_config,
                                  friction,
                                  damping);
            break;

        case AXIS_Z:
            return model.addJoint(frame.parent,
                                  TypeZ(),
                                  frame.placement * placement,
                                  joint_name,
                                  max_effort,
                                  max_velocity,
                                  min_config,
                                  max_config,
                                  friction,
                                  damping);
            break;

        case AXIS_UNALIGNED:
            return model.addJoint(frame.parent,
                                  TypeUnaligned(axis.normalized()),
                                  frame.placement * placement,
                                  joint_name,
                                  max_effort,
                                  max_velocity,
                                  min_config,
                                  max_config,
                                  friction,
                                  damping);
            break;
        }

        // Should not be reached
        assert(false);
        return static_cast<pinocchio::JointIndex>(-1);
    }

    void addJointAndBody(JointType type,
                         const Vector3& axis,
                         const pinocchio::FrameIndex& parentFrameId,
                         const SE3& parent_H_joint,
                         const SE3& joint_H_child,
                         const std::string& jointName,
                         const Inertia& Y,
                         const std::string& bodyName,
                         const VectorConstRef& maxEffort,
                         const VectorConstRef& maxVelocity,
                         const VectorConstRef& minConfig,
                         const VectorConstRef& maxConfig,
                         const VectorConstRef& friction,
                         const VectorConstRef& damping)
    {
        pinocchio::JointIndex jointId;
        const Frame& frame = model.frames[parentFrameId];

        switch (type)
        {
        case Base::REVOLUTE:
            jointId
                = addJoint<typename JointCollection::JointModelRX,
                           typename JointCollection::JointModelRY,
                           typename JointCollection::JointModelRZ,
                           typename JointCollection::JointModelRevoluteUnaligned>(axis,
                                                                                  frame,
                                                                                  parent_H_joint,
                                                                                  jointName,
                                                                                  maxEffort,
                                                                                  maxVelocity,
                                                                                  minConfig,
                                                                                  maxConfig,
                                                                                  friction,
                                                                                  damping);
            break;
        case Base::CONTINUOUS:
            jointId = addJoint<
                typename JointCollection::JointModelRUBX,
                typename JointCollection::JointModelRUBY,
                typename JointCollection::JointModelRUBZ,
                typename JointCollection::JointModelRevoluteUnboundedUnaligned>(axis,
                                                                                frame,
                                                                                parent_H_joint,
                                                                                jointName,
                                                                                maxEffort,
                                                                                maxVelocity,
                                                                                minConfig,
                                                                                maxConfig,
                                                                                friction,
                                                                                damping);
            break;
        case Base::PRISMATIC:
            jointId
                = addJoint<typename JointCollection::JointModelPX,
                           typename JointCollection::JointModelPY,
                           typename JointCollection::JointModelPZ,
                           typename JointCollection::JointModelPrismaticUnaligned>(axis,
                                                                                   frame,
                                                                                   parent_H_joint,
                                                                                   jointName,
                                                                                   maxEffort,
                                                                                   maxVelocity,
                                                                                   minConfig,
                                                                                   maxConfig,
                                                                                   friction,
                                                                                   damping);
            break;
        };

        pinocchio::FrameIndex jointFrameId = model.addJointFrame(jointId, (int)parentFrameId);
        appendBodyToJoint(jointFrameId, Y, joint_H_child, bodyName);
    }

    virtual void addAdditionalFrame(const std::string& additionalFrameName,
                                    const std::string& parentLinkName,
                                    const pinocchio::SE3& link_H_additionalframe)
    {
        pinocchio::FrameIndex parentLinkIndex = model.getBodyId(parentLinkName);
        pinocchio::JointIndex parentJointIndex = model.frames[parentLinkIndex].parent;
        pinocchio::SE3 joint_H_link = model.frames[parentLinkIndex].placement;
        pinocchio::SE3 joint_H_additionalframe = joint_H_link * link_H_additionalframe;
        // The previousFrame attribute is only used to distinguish the interconnection
        // of multiple frames that are rigidly interconnected to the same joint,
        // see
        // https://github.com/stack-of-tasks/pinocchio/blob/78d62096002ffa3790638e392f0b6e4a5efc3d34/src/algorithm/frames.hxx#L280
        // In this case we are building a Frame with zero inertia, so there previousFrame does not
        // have any effect, anyhow for consistency we mark the parentLinkIndex as previousFrame
        model.addFrame(Frame(additionalFrameName,
                             parentJointIndex,
                             parentLinkIndex,
                             joint_H_additionalframe,
                             pinocchio::OP_FRAME));
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

// We vendor here the construction of the traversal as apparently:
// * Pinocchio requires that models are built in a strictly Depth First order
// * Despite what the documentaton says, the `Model::computeFullTreeTraversal`
// does not built the traversal in DF order
struct stackEl
{
    iDynTree::LinkConstPtr link = nullptr;
    iDynTree::LinkConstPtr parentLink = nullptr;
    iDynTree::IJointConstPtr parentJoint = nullptr;
};

inline void iDynFor_addBaseLinkToStack(const iDynTree::Model& model,
                                       iDynTree::Traversal& traversal,
                                       iDynTree::LinkIndex linkToAdd,
                                       std::deque<stackEl>& linkToVisit)
{
    stackEl el;
    el.link = model.getLink(linkToAdd);
    linkToVisit.push_back(el);
}

inline void iDynFor_addLinkToStack(const iDynTree::Model& model,
                                   iDynTree::Traversal& traversal,
                                   iDynTree::LinkIndex linkToAdd,
                                   iDynTree::JointIndex parentJointToAdd,
                                   iDynTree::LinkIndex parentLinkToAdd,
                                   std::deque<stackEl>& linkToVisit)
{

    stackEl el;
    el.link = model.getLink(linkToAdd);
    el.parentLink = model.getLink(parentLinkToAdd);
    el.parentJoint = model.getJoint(parentJointToAdd);
    linkToVisit.push_back(el);
}

inline bool iDynFor_computeFullTreeTraversal(const iDynTree::Model& model,
                                             iDynTree::Traversal& traversal,
                                             const iDynTree::LinkIndex traversalBase)
{
    if (traversalBase < 0 || traversalBase >= (iDynTree::LinkIndex)model.getNrOfLinks())
    {
        // reportError("Model","computeFullTreeTraversal","requested traversalBase is out of
        // bounds");
        return false;
    }

    // Resetting the traversal for populating it
    traversal.reset(model);

    // A link is considered visit when all its child (given the traversalBase)
    // have been added to the traversal
    std::deque<stackEl> linkToVisit;

    // We add as first link the stack
    iDynFor_addBaseLinkToStack(model, traversal, traversalBase, linkToVisit);

    // while there is some link still to visit
    while (linkToVisit.size() > 0)
    {
        assert(linkToVisit.size() <= model.getNrOfLinks());

        // DPS : we use linkToVisit as a stack
        iDynTree::LinkConstPtr visitedLink = linkToVisit.back().link;
        iDynTree::LinkConstPtr visitedLinkParent = linkToVisit.back().parentLink;
        iDynTree::IJointConstPtr visitedLinkParentJoint = linkToVisit.back().parentJoint;
        iDynTree::LinkIndex visitedLinkIndex = visitedLink->getIndex();
        linkToVisit.pop_back();

        // Add element extracted from the stack to Traversal
        if (!visitedLinkParent)
        {
            traversal.addTraversalBase(visitedLink);
        } else
        {
            traversal.addTraversalElement(visitedLink, visitedLinkParentJoint, visitedLinkParent);
        }

        for (unsigned int neigh_i = 0; neigh_i < model.getNrOfNeighbors(visitedLinkIndex);
             neigh_i++)
        {
            // add to the stack all the neighbors, except for parent link
            // (if the visited link is the base one, add all the neighbors)
            // the visited link is already in the Traversal, so we can use it
            // to check for its parent
            iDynTree::Neighbor neighb = model.getNeighbor(visitedLinkIndex, neigh_i);
            if (visitedLinkParent == 0 || neighb.neighborLink != visitedLinkParent->getIndex())
            {
                iDynFor_addLinkToStack(model,
                                       traversal,
                                       neighb.neighborLink,
                                       neighb.neighborJoint,
                                       visitedLink->getIndex(),
                                       linkToVisit);
            }
        }
    }

    return true;
}

/**
 * \brief Build the pinocchio model from a iDynTree::Model .
 *
 * \param[in] model The iDynTree::Model to load.
 * \param[in] verbose Print parsing info.
 * \param[out] model Reference model where to put the parsed information.
 * \return Return true if all went well, false otherwise.
 *
 * The signature of this function is inspired from the pinocchio::buildModel function
 */
template <typename Scalar, int Options, template <typename, int> class JointCollectionTpl>
bool buildPinocchioModelfromiDynTree(
    const iDynTree::Model& modelIDynTree,
    pinocchio::ModelTpl<Scalar, Options, JointCollectionTpl>& modelPin,
    const bool verbose = false)
{
    // Build visitior
    iDynFor::details::iDynTreeModelVisitor<Scalar, Options, JointCollectionTpl> visitor(modelPin);

    // Once iDynTree::Model has a name pass the name along
    // See https://github.com/robotology/idyntree/issues/908
    visitor.setName("iDynForModel");

    // Generate traversal to follow to convert iDynTree::Model to pinocchio::Model
    // At the moment, we just hardcode the use of getDefaultBaseLink(), but we could
    // take that as an option in the future
    iDynTree::Traversal traversal;
    bool ok = iDynFor_computeFullTreeTraversal(modelIDynTree,
                                               traversal,
                                               modelIDynTree.getDefaultBaseLink());
    if (!ok)
    {
        if (verbose)
        {
            std::cerr << "iDynFor::buildPinocchioModelfromiDynTree: error in calling "
                         "computeFullTreeTraversal on input model"
                      << std::endl;
        }
        return false;
    }

    for (unsigned int traversalEl = 0; traversalEl < traversal.getNrOfVisitedLinks(); traversalEl++)
    {
        iDynTree::LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        std::string visitedLinkName = modelIDynTree.getLinkName(visitedLink->getIndex());
        iDynTree::LinkConstPtr parentLink = traversal.getParentLink(traversalEl);
        iDynTree::IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        if (parentLink == 0)
        {
            // If the visited link is the base, the base has no parent.
            // Add root link
            std::string baseLinkName = modelIDynTree.getLinkName(visitedLink->getIndex());
            visitor.addRootJoint(toPinocchio(visitedLink->getInertia()), baseLinkName);
        } else
        {
            // For non-base links, add the link and the joint connecting it to its parent

            // Part of this code is inspired from
            // https://github.com/stack-of-tasks/pinocchio/blob/v2.6.17/src/parsers/urdf/model.cpp#L88
            std::string parentLinkName = modelIDynTree.getLinkName(parentLink->getIndex());
            std::string jointName = modelIDynTree.getJointName(toParentJoint->getIndex());

            pinocchio::FrameIndex parentFrameId = visitor.model.getBodyId(parentLinkName);

            const pinocchio::Inertia Y = toPinocchio(visitedLink->getInertia());

            Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> maxEffort(1), maxVelocity(1),
                minConfig(1), maxConfig(1);
            Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> friction(1), damping(1);

            const Scalar infty = std::numeric_limits<Scalar>::infinity();

            // Extract joint type
            // For now, only fixed is supported
            enum
            {
                Revolute,
                Prismatic,
                Fixed,
                NotSupported
            } parsedJointType;

            parsedJointType = NotSupported;

            if (dynamic_cast<const iDynTree::FixedJoint*>(toParentJoint))
            {
                parsedJointType = Fixed;
            }

            if (dynamic_cast<const iDynTree::RevoluteJoint*>(toParentJoint))
            {
                parsedJointType = Revolute;
            }

            if (dynamic_cast<const iDynTree::PrismaticJoint*>(toParentJoint))
            {
                parsedJointType = Prismatic;
            }

            switch (parsedJointType)
            {
                // Code common to 1-dof joints
            case Revolute:
            case Prismatic: {
                typename details::iDynTreeModelVisitorBaseTpl<Scalar, Options>::JointType
                    pinocchioJointType;
                iDynTree::Axis axisJoint;

                if (parsedJointType == Revolute)
                {
                    pinocchioJointType
                        = details::iDynTreeModelVisitorBaseTpl<Scalar, Options>::JointType::REVOLUTE;
                    const iDynTree::RevoluteJoint* revJoint
                        = dynamic_cast<const iDynTree::RevoluteJoint*>(toParentJoint);
                    axisJoint = revJoint->getAxis(visitedLink->getIndex());
                }

                if (parsedJointType == Prismatic)
                {
                    pinocchioJointType
                        = details::iDynTreeModelVisitorBaseTpl<Scalar,
                                                               Options>::JointType::PRISMATIC;
                    const iDynTree::PrismaticJoint* prismaticJoint
                        = dynamic_cast<const iDynTree::PrismaticJoint*>(toParentJoint);
                    axisJoint = prismaticJoint->getAxis(visitedLink->getIndex());
                }

                // Extract axis
                Eigen::Matrix<Scalar, 3, 1, Options> axisDirection_wrt_visitedLink
                    = iDynTree::toEigen(axisJoint.getDirection());
                Eigen::Matrix<Scalar, 3, 1, Options> axisOrigin_wrt_visitedLink
                    = iDynTree::toEigen(axisJoint.getOrigin());

                // See theory_background " Convert from iDynTree to pinocchio" Section for details
                // on this
                iDynTree::Transform parentLink_H_childLink_rest
                    = toParentJoint->getRestTransform(parentLink->getIndex(),
                                                      visitedLink->getIndex());
                iDynTree::Transform childLink_H_jointFrame_rest
                    = iDynTree::Transform(iDynTree::Rotation::Identity(), axisJoint.getOrigin());
                iDynTree::Transform parentLink_H_jointFrame_rest
                    = parentLink_H_childLink_rest * childLink_H_jointFrame_rest;
                iDynTree::Transform jointFrame_H_childLink_rest
                    = childLink_H_jointFrame_rest.inverse();

                const pinocchio::SE3 parentLink_H_jointFrame
                    = toPinocchio(parentLink_H_jointFrame_rest);
                const pinocchio::SE3 jointFrame_H_childLink
                    = toPinocchio(jointFrame_H_childLink_rest);

                // Handle limits
                if (toParentJoint->hasPosLimits())
                {
                    minConfig[0] = toParentJoint->getMinPosLimit(0);
                    maxConfig[0] = toParentJoint->getMaxPosLimit(0);
                } else
                {
                    minConfig[0] = -infty;
                    maxConfig[0] = infty;
                }

                // This quantities are not exposed by iDynTree, as of 8.0.1
                friction[0] = 0.0;
                damping[0] = 0.0;
                maxEffort[0] = infty;
                maxVelocity[0] = infty;

                visitor.addJointAndBody(pinocchioJointType,
                                        axisDirection_wrt_visitedLink,
                                        parentFrameId,
                                        parentLink_H_jointFrame,
                                        jointFrame_H_childLink,
                                        jointName,
                                        Y,
                                        visitedLinkName,
                                        maxEffort,
                                        maxVelocity,
                                        minConfig,
                                        maxConfig,
                                        friction,
                                        damping);
            }
            break;
            case Fixed: {
                // Transformation from the parent link to the joint origin
                // In the fixed joint case, the fixed joint frame corresponds to the child link
                // frame
                iDynTree::Transform parentLink_H_fixedJointFrame_rest
                    = toParentJoint->getRestTransform(parentLink->getIndex(),
                                                      visitedLink->getIndex());
                const pinocchio::SE3 fixedJointPlacement
                    = toPinocchio(parentLink_H_fixedJointFrame_rest);

                // In case of fixed joint, if link has inertial tag:
                //    -add the inertia of the link to his parent in the model
                // Otherwise do nothing.
                // In all cases:
                //    -let all the children become children of parent
                //    -inform the parser of the offset to apply
                //    -add fixed body in model for visualization purpouses
                visitor.addFixedJointAndBody(parentFrameId,
                                             fixedJointPlacement,
                                             jointName,
                                             Y,
                                             visitedLinkName);
            }
            break;

            case NotSupported:
            default:
                if (verbose)
                {
                    std::cerr << "iDynFor::buildPinocchioModelfromiDynTree: unsupported type for "
                                 "joint "
                              << jointName << std::endl;
                }
                return false;
                break;
            }
        }

        // Add additional frames of the visited link
        std::vector<iDynTree::FrameIndex> localFrmIdxs;
        modelIDynTree.getLinkAdditionalFrames(visitedLink->getIndex(), localFrmIdxs);
        for (iDynTree::FrameIndex localFrmIdx : localFrmIdxs)
        {
            std::string additionalFrameName = modelIDynTree.getFrameName(localFrmIdx);
            iDynTree::Transform link_H_additionalFrame
                = modelIDynTree.getFrameTransform(localFrmIdx);
            visitor.addAdditionalFrame(additionalFrameName,
                                       visitedLinkName,
                                       toPinocchio(link_H_additionalFrame));
        }
    }

    // Do a sanity check before returning the model
    // This is expensive, let's do it only in Debug model
#ifndef NDEBUG
    auto dataPin = pinocchio::DataTpl<Scalar, Options, JointCollectionTpl>(modelPin);
    if (!modelPin.check(dataPin))
    {
        if (verbose)
        {
            std::cerr << "iDynFor::buildPinocchioModelfromiDynTree: error in calling "
                         "the Data::check on the generated pinocchio model"
                      << std::endl;
        }
        return false;
    }
#endif

    return true;
}

} // namespace iDynFor
