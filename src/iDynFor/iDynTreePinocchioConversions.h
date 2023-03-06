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
    Model& model;
    iDynTreeModelVisitor(Model& model)
        : model(model)
    {
    }
    void setName(const std::string& name)
    {
        model.name = name;
    }
    virtual void addRootJoint(const Inertia& Y, const std::string& body_name)
    {
        addFixedJointAndBody(0, SE3::Identity(), "root_joint", Y, body_name);
        // TODO: change for the correct behavior, see
        //   https://github.com/stack-of-tasks/pinocchio/pull/1102 for discussions on the topic
        //   and
        //   https://github.com/stack-of-tasks/pinocchio/blob/280005c8d99c2485ee942b6a38c4dc0edf75c706/src/parsers/urdf/model.hxx#L117
        // appendBodyToJoint(0,Y,SE3::Identity(),body_name);
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

    return modelPin;
}

} // namespace iDynFor
