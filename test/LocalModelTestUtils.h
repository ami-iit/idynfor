#ifndef IDYNFOR_LOCAL_MODEL_TEST_UTILS_H
#define IDYNFOR_LOCAL_MODEL_TEST_UTILS_H

// Functions vendored from
// https://github.com/robotology/idyntree/blob/4e9d8097753dc146914e55f5656b465d00e6b25f/src/model/include/iDynTree/Model/ModelTestUtils.h#L118
// As we currently need to customize them until iDynFor support all features of iDynTree::Model
inline void iDynFor_addRandomLinkToModel(iDynTree::Model& model,
                                         std::string parentLink,
                                         std::string newLinkName,
                                         bool noFixed = false)
{
    // Add Link
    iDynTree::LinkIndex newLinkIndex = model.addLink(newLinkName, iDynTree::getRandomLink());

    // Now add joint
    iDynTree::LinkIndex parentLinkIndex = model.getLinkIndex(parentLink);

    // Only consider fixed (0) and revolute (1)
    int nrOfJointTypes = 2;

    int jointType = rand() % nrOfJointTypes;

    if (noFixed)
        jointType = 1;

    if (jointType == 0)
    {
        iDynTree::FixedJoint fixJoint(parentLinkIndex,
                                      newLinkIndex,
                                      iDynTree::getRandomTransform());
        model.addJoint(newLinkName + "joint", &fixJoint);
    } else if (jointType == 1)
    {
        iDynTree::RevoluteJoint revJoint;
        revJoint.setAttachedLinks(parentLinkIndex, newLinkIndex);
        revJoint.setRestTransform(iDynTree::getRandomTransform());
        revJoint.setAxis(iDynTree::getRandomAxis(), newLinkIndex);
        model.addJoint(newLinkName + "joint", &revJoint);
    } else
    {
        assert(false);
    }
}

inline iDynTree::Model
iDynFor_getRandomModel(unsigned int nrOfJoints, size_t nrOfAdditionalFrames = 10)
{
    iDynTree::Model model;

    model.addLink("baseLink", iDynTree::getRandomLink());

    for (unsigned int i = 0; i < nrOfJoints; i++)
    {
        std::string parentLink = getRandomLinkOfModel(model);
        std::string linkName = "link" + iDynTree::int2string(i);
        iDynFor_addRandomLinkToModel(model, parentLink, linkName);
    }

    for (unsigned int i = 0; i < nrOfAdditionalFrames; i++)
    {
        std::string parentLink = getRandomLinkOfModel(model);
        std::string frameName = "additionalFrame" + iDynTree::int2string(i);
        addRandomAdditionalFrameToModel(model, parentLink, frameName);
    }

    return model;
}

#endif
